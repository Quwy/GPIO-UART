
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdint.h>

#include "UART.h"

#define UART_TIMER_CORRECTION (+2)
#define UART_TIMER_TOP        (((F_CPU) / (1 << (UART_TIMER_SCALE - 1)) / (UART_BAUD_RATE)) + (UART_TIMER_CORRECTION))

#if UART_TIMER_TOP < 10 // minimal accuracy of the timer for 10-bit packet, lower value may cause synchronization fail on last bits
#error "Invalid parameters combination: decrease UART_TIMER_SCALE, decrease UART_BAUD_RATE, or increase CPU clock"
#elif UART_TIMER_TOP >= 255
#error "Invalid parameters combination: increase UART_TIMER_SCALE, increase UART_BAUD_RATE, or decrease CPU clock"
#endif

#define UART_CURRENT_BIT_IDLE  (0xFF)
#define UART_CURRENT_BIT_START (UART_CURRENT_BIT_IDLE - 1)
#define UART_CURRENT_BIT_STOP  (UART_CURRENT_BIT_IDLE - 2)

static volatile uint8_t uart_tx_current_byte; // buffer for current byte TX
static volatile uint8_t uart_rx_current_byte; // buffer for current byte RX
static volatile uint8_t uart_tx_current_bit = UART_CURRENT_BIT_IDLE; // current TX bit of uart_tx_current_byte (UART_CURRENT_BIT_IDLE mean no TX in progress)
static volatile uint8_t uart_rx_current_bit = UART_CURRENT_BIT_IDLE; // current RX bit of uart_rx_current_byte (UART_CURRENT_BIT_IDLE mean no RX in progress)
static volatile uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE]; // TX buffer
static volatile uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE]; // RX buffer
static volatile uint8_t uart_tx_buffer_size = 0; // actual size of TX data
static volatile uint8_t uart_rx_buffer_size = 0; // actual size of RX data
static volatile uint8_t uart_prev_PINB; // previous PINB value (needed to know which pin was changed in the interrupt handler)

inline void uart_init(void) {
    // Pins init
    PORTB |= (1 << PB1); // set TX pin to "high" idle state
    DDRB |= (1 << DDB1); // set TX pin as output
    PORTB &= ~(1 << PB0); // disable pull-up for RX pin if any
    DDRB &= ~(1 << DDB0); // set RX pin as input

    // Interrupts init
    GIMSK |= (1 << PCIE); // enable pin change interrupt
    PCMSK |= (1 << PCINT0); // unmask RX pin change interrupt

    // Timer1 init
    TCCR1 = (1 << CTC1); // reset timer on compare match with OCR1C, Pulse Width Modulator A disable, Timer/Counter Comparator A disconnected from output pin OC1A, disable clock
    GTCCR &= ~(1 << PWM1B) & //  Pulse Width Modulator B disable
             ~(1 << COM1B1) & ~(1 << COM1B0) & // Timer1 Comparator B disconnected from output pin OC1B
             ~(1 << PSR1); // prescaler reset
    TCNT1 = 0; // initial caounter value
    OCR1C = UART_TIMER_TOP; // top value (by default: 8 000 000 / 8 / 9600 = 104)
    TIMSK &= ~(1 << OCIE1A) & ~(1 << OCIE1B) & ~(1 << TOIE1); // disable all compare match and overflow interrupts
    
    // locals init
    uart_prev_PINB = PINB;
}

// Get lower byte from buffer and shift rest of bytes left
// !!! NO ACTUAL BUFFER LENGTH CHECKED !!!
static uint8_t uart_get_next_byte(volatile uint8_t *const buffer, volatile uint8_t *const buffer_size) {
    const uint8_t result = buffer[0]; // make a copy of the first byte
        
    --(*buffer_size); // decrease size
        
    // shift rest of the buffer left
    for(uint8_t i = 0; i < *buffer_size; ++i) {
        buffer[i] = buffer[i + 1];
    }

    return result;
}

// Append byte to the buffer end
// Free buffer space is checked before action
static bool uart_set_next_byte(const uint8_t byte, volatile uint8_t *const buffer, volatile uint8_t *const buffer_size, const uint8_t max_buffer_size) {
    if(*buffer_size < max_buffer_size) {
        buffer[(*buffer_size)++] = byte; // write byte and increase size
        return true;
    } else {
        return false; // return false if no room
    }
}

static inline void uart_stop_timer(void) {
    TCCR1 &= ~(1 << CS10) & ~(1 << CS11) & ~(1 << CS12) & ~(1 << CS13); // Stop Timer1
}

// check for timer now stopped
static inline bool uart_no_timer(void) {
    return ((TCCR1 & ((1 << CS10) | (1 << CS11) | (1 << CS12) | (1 << CS13))) == 0);
}

static void uart_start_timer(void) {
    if(uart_no_timer()) {
        GTCCR |= (1 << PSR1); // reset Timer1 prescaler

        TCCR1 = (TCCR1 & ~(1 << CS10) & ~(1 << CS11) & ~(1 << CS12) & ~(1 << CS13)) | (UART_TIMER_SCALE & ((1 << CS10) | (1 << CS11) | (1 << CS12) | (1 << CS13))); // start Timer1 with selected prescaler
    }
}

// begin transmitting of uart_tx_current_byte
static void uart_start_byte_tx(void) {
    PORTB &= ~(1 << PB1); // pull TX "low" for start bit
    OCR1A = TCNT1; // fix strobe
    #ifdef PROTEUS
    if(OCR1A == 0) OCR1A = OCR1C; // Proteus does not call TIMER1_COMPA_vect if OCR1A = 0
    #endif
    TIFR |= (1 << OCF1A); // clear interrupt flag to prevent immediate call of interrupt vector
    TIMSK |= (1 << OCIE1A); // enable compare match OCR1A interrupt
    uart_tx_current_bit = UART_CURRENT_BIT_START; // start bit pending
    uart_start_timer(); // start timer if stopped
}

// begin recieving of byte to uart_rx_current_byte
static void uart_start_byte_rx(void) {
    OCR1B = TCNT1; // fix strobe
    #ifdef PROTEUS
    if(OCR1B == 0) OCR1B = OCR1C; // Proteus does not call TIMER1_COMPB_vect if OCR1B = 0
    #endif
    TIFR |= (1 << OCF1B); // clear interrupt flag to prevent immediate call of interrupt vector
    TIMSK |= (1 << OCIE1B); // enable compare match OCR1B interrupt
    uart_rx_current_bit = UART_CURRENT_BIT_START; // start bit pending
    uart_start_timer(); // start timer if stopped
}

// stop byte TX
static void uart_stop_byte_tx(void) {
    TIMSK &= ~(1 << OCIE1A); // disable compare match OCR1A interrupt
    uart_tx_current_bit = UART_CURRENT_BIT_IDLE;
    if(uart_rx_current_bit == UART_CURRENT_BIT_IDLE) {
        uart_stop_timer();
    }
}

// stop byte RX
static void uart_stop_byte_rx(void) {
    TIMSK &= ~(1 << OCIE1B); // disable compare match OCR1B interrupt
    uart_rx_current_bit = UART_CURRENT_BIT_IDLE;
    if(uart_tx_current_bit == UART_CURRENT_BIT_IDLE) {
        uart_stop_timer();
    }
}

// transmit one byte
void uart_tx(const uint8_t byte) {
    if(uart_tx_current_bit == UART_CURRENT_BIT_IDLE) { // start TX only if not already running
        uart_tx_current_byte = byte; // extract next byte from TX buffer
        uart_start_byte_tx();
    } else {
        while(uart_tx_buffer_size == UART_TX_BUFFER_SIZE); // wait for buffer room
        uart_set_next_byte(byte, uart_tx_buffer, &uart_tx_buffer_size, UART_TX_BUFFER_SIZE);
    }
}

// transmit string from flash address space
void uart_tx_str(__flash const char *const str) {
    for(uint8_t i = 0; str[i] != '\0'; ++i) {
        uart_tx(str[i]);
    }
}

// get received bytes count
uint8_t uart_rx_count(void) {
    return uart_rx_buffer_size;
}

// get one byte from RX buffer
// !!! NO ACTUAL BUFFER LENGTH CHECKED !!!
// !!! CALL uart_rx_count() BEFORE !!!
inline uint8_t uart_rx(void) {
    return uart_get_next_byte(uart_rx_buffer, &uart_rx_buffer_size);
}

// shift timer top by half of period
static inline uint8_t uart_shift_strobe_time(const uint8_t OCR1) {
    return (OCR1 >= (UART_TIMER_TOP / 2)) ? (OCR1 - (UART_TIMER_TOP / 2)) : (OCR1 + (UART_TIMER_TOP / 2));
}

// pin interrupt
ISR(PCINT0_vect) {
    if(uart_rx_current_bit == UART_CURRENT_BIT_IDLE && (uart_prev_PINB & (1 << PINB0)) != 0 && (PINB & (1 << PINB0)) == 0) { // no RX in progress and RX line go "low" (start condition)
        uart_start_byte_rx();
    }

    uart_prev_PINB = PINB;
}

// TX timer compare match
ISR(TIMER1_COMPA_vect) {
    switch(uart_tx_current_bit) {
        case UART_CURRENT_BIT_IDLE: // must not be here
            uart_stop_byte_tx(); // stop any TX progress
            break;

        case UART_CURRENT_BIT_STOP: // stop bit sent
            if(uart_tx_buffer_size == 0) { // if no more bytes in the buffer
                uart_stop_byte_tx();
            } else { // TX next byte without an pause
                uart_tx_current_byte = uart_get_next_byte(uart_tx_buffer, &uart_tx_buffer_size); // extract next byte from TX buffer
                PORTB &= ~(1 << PB1); // pull TX "low" for start bit
                uart_tx_current_bit = UART_CURRENT_BIT_START;
            }
            break;

        case UART_CURRENT_BIT_START: // start bit sent
            uart_tx_current_bit = 0;
            // no break, go TX first data bit now

        default:
            if(uart_tx_current_bit == 8) {
                PORTB |= (1 << PB1); // push TX "high" for stop bit
                uart_tx_current_bit = UART_CURRENT_BIT_STOP;
            } else {
                if((uart_tx_current_byte & 0b00000001) != 0) {
                    PORTB |= (1 << PB1);
                } else {
                    PORTB &= ~(1 << PB1);
                }

                ++uart_tx_current_bit;
                uart_tx_current_byte >>= 1;
            }

    }
}

// RX timer compare match
ISR(TIMER1_COMPB_vect) {
    switch(uart_rx_current_bit) {
        case UART_CURRENT_BIT_IDLE: // must not be here
            uart_stop_byte_rx(); // stop any RX progress
            break;

        case UART_CURRENT_BIT_START:  // time of start bit ending, charge timer to strobe data bits
            OCR1B = uart_shift_strobe_time(OCR1B); // shift strobe time by TOP/2 to the center of the bit
            #ifdef PROTEUS
            if(OCR1B == 0) OCR1B = OCR1C; // Proteus does not call TIMER1_COMPB_vect if OCR1B = 0
            #endif
            uart_rx_current_bit = 0; // ready for receive data bits
            break;

        case UART_CURRENT_BIT_STOP: // stop bit strobe
            if((PINB & (1 << PINB0)) != 0) { // stop bit is "high"?
                uart_set_next_byte(uart_rx_current_byte, uart_rx_buffer, &uart_rx_buffer_size, UART_RX_BUFFER_SIZE); // save byte to the buffer
            }
            uart_stop_byte_rx(); // stop RX and go ready to new incoming byte
            break;

        default: // data bit
            uart_rx_current_byte = (uart_rx_current_byte >> 1) | ((PINB << (7 - PINB0)) & 0b10000000); // set left bit of the RX byte by pin 5 (PB0) value
            if(uart_rx_current_bit == 7) {
                uart_rx_current_bit = UART_CURRENT_BIT_STOP;
            } else {
                ++uart_rx_current_bit;
            }
    }
}

