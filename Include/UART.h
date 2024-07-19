#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>

#include "Defines.h"

#ifndef F_CPU
#error "F_CPU is mandatory for this code"
#endif

#define UART_TX_BUFFER_SIZE 16
#define UART_RX_BUFFER_SIZE 8

#define UART_BAUD_RATE   9600 // allowed any value
#define UART_TIMER_SCALE 4    // default 4 = f/8, allowed only values 1..15 (1=f/1, 2=f/2, 3=f/4, 4=f/8, 5=f/16, 6=f/32, 7=f/64, 8=f/128, 9=f/256, 10=f/512, 11=f/1024, 12=f/2048, 13=f/4096, 14=f/8192, 15=f/16384)

// transmit one byte
void uart_tx(const uint8_t byte);

// transmit string from flash address space
void uart_tx_str(__flash const char *const str);

// get received bytes count
uint8_t uart_rx_count(void);

// get one byte from RX buffer
// !!! NO ACTUAL BUFFER LENGTH CHECKED !!!
// !!! CALL uart_rx_count() BEFORE !!!
uint8_t uart_rx(void);

void uart_init(void);

#endif /* __UART_H__ */
