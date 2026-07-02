#ifndef __UART_COMM_H__
#define __UART_COMM_H__

#include <stdint.h>

void uart_comm_init(void);
void uart_comm_poll(void);
void uart_comm_rx_enable(void);
uint16_t uart_rx_available(void);
uint8_t uart_rx_pop(uint8_t *byte);

#endif
