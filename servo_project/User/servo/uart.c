#include "bsp/bap_uart.h"
#include "uart.h"
#include "sts_proto.h"

uart_rx_ring_t g_uart_rx_ring;
static volatile uint8_t s_uart_rx_blocked;

static bool ring_put(uint8_t byte)
{
    uint16_t next = (uint16_t)((g_uart_rx_ring.head + 1U) % UART_RX_RING_SIZE);

    if (next == g_uart_rx_ring.tail) {
        return FALSE;
    }
    g_uart_rx_ring.buf[g_uart_rx_ring.head] = byte;
    g_uart_rx_ring.head = next;
    return TRUE;
}
static bool ring_get(uint8_t *byte)
{
    if (g_uart_rx_ring.head == g_uart_rx_ring.tail) {
        return FALSE;
    }
    *byte = g_uart_rx_ring.buf[g_uart_rx_ring.tail];
    g_uart_rx_ring.tail = (uint16_t)((g_uart_rx_ring.tail + 1U) % UART_RX_RING_SIZE);
    return TRUE;
}

void uart_comm_init(void)
{
    g_uart_rx_ring.head = 0U;
    g_uart_rx_ring.tail = 0U;
    bap_uart_init();
    sts_proto_init();
    uart_comm_rx_enable();
}

void uart_comm_poll(void)
{
    sts_proto_poll();
}

void uart_comm_rx_enable(void)
{
    usart_interrupt_enable(COMM_USART, USART_INT_RBNE);
    nvic_irq_enable(COMM_USART_IRQn, 2U, 0U);
}

void uart_comm_tx_begin(void)
{
    s_uart_rx_blocked = 1U;
    usart_interrupt_disable(COMM_USART, USART_INT_RBNE);
    while (RESET != usart_flag_get(COMM_USART, USART_FLAG_RBNE)) {
        (void)usart_data_receive(COMM_USART);
    }
}

void uart_comm_tx_end(void)
{
    while (RESET == usart_flag_get(COMM_USART, USART_FLAG_TC)) {
    }
    usart_flag_clear(COMM_USART, USART_FLAG_TC);
    while (RESET != usart_flag_get(COMM_USART, USART_FLAG_RBNE)) {
        (void)usart_data_receive(COMM_USART);
    }
    s_uart_rx_blocked = 0U;
    usart_interrupt_enable(COMM_USART, USART_INT_RBNE);
}

void uart_comm_tx(uint8_t *data, uint8_t length)
{
    uart_comm_tx_begin();
    bap_uart_send(data, length);
    uart_comm_tx_end();
}

void uart_comm_rx_flush(void)
{
    g_uart_rx_ring.head = 0U;
    g_uart_rx_ring.tail = 0U;
    while (RESET != usart_flag_get(COMM_USART, USART_FLAG_RBNE)) {
        (void)usart_data_receive(COMM_USART);
    }
}

void USART1_IRQHandler(void)
{
    if (RESET != usart_interrupt_flag_get(COMM_USART, USART_INT_FLAG_RBNE)) {
        uint8_t byte = (uint8_t)usart_data_receive(COMM_USART);
        if (s_uart_rx_blocked != 0U) {
            return;
        }
        ring_put(byte);
    }
}

uint16_t uart_rx_available(void)
{
    uint16_t head = g_uart_rx_ring.head;
    uint16_t tail = g_uart_rx_ring.tail;

    if (head >= tail) {
        return (uint16_t)(head - tail);
    }
    return (uint16_t)(UART_RX_RING_SIZE - tail + head);
}

uint8_t uart_rx_pop(uint8_t *byte)
{
    return ring_get(byte) ? 1U : 0U;
}
