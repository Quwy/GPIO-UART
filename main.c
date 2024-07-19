
#include <avr/sleep.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#include "Defines.h"
#include "UART.h"

static inline void mcu_init(void) {
    sleep_enable();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sei();
}

static inline void do_yield(void) {
    sleep_cpu();
}

static inline void do_work(void) {
    uint8_t byte;

    while(uart_rx_count() > 0) {
        byte = uart_rx();

        // Upcase latin before send it back
        if(byte >= 'a' && byte <= 'z') {
            byte -= 'a' - 'A';
        }

        uart_tx(byte);
    }
}

__flash const char DEMO_STRING[] = "GPIO-UART DEMO\r\r"; // leave string in the flash address space

int main(void) {
    mcu_init();
    uart_init();

    uart_tx_str(DEMO_STRING);

    while(true) {
        do_work();
        do_yield();
    }
}
