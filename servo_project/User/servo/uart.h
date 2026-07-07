#ifndef __UART_COMM_H__
#define __UART_COMM_H__

#include <stdint.h>

#define UART_RX_RING_SIZE   256U

typedef struct {
    uint8_t buf[UART_RX_RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} uart_rx_ring_t;

extern uart_rx_ring_t g_uart_rx_ring;

void uart_comm_init(void);
void uart_comm_poll(void);
void uart_comm_rx_enable(void);
void uart_comm_tx_begin(void);
void uart_comm_tx_end(void);
void uart_comm_rx_flush(void);
void uart_comm_tx(uint8_t *data, uint8_t length);
uint16_t uart_rx_available(void);
uint8_t uart_rx_pop(uint8_t *byte);


#endif
