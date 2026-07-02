#include "bsp/bap_uart.h"
#include "uart.h"

#define UART_RX_RING_SIZE   256U

typedef struct {
    uint8_t buf[UART_RX_RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} uart_rx_ring_t;

static uart_rx_ring_t s_rx_ring;

static bool ring_put(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rx_ring.head + 1U) % UART_RX_RING_SIZE);

    if (next == s_rx_ring.tail) {
        return FALSE;
    }
    s_rx_ring.buf[s_rx_ring.head] = byte;
    s_rx_ring.head = next;
    return TRUE;
}

static bool ring_get(uint8_t *byte)
{
    if (s_rx_ring.head == s_rx_ring.tail) {
        return FALSE;
    }
    *byte = s_rx_ring.buf[s_rx_ring.tail];
    s_rx_ring.tail = (uint16_t)((s_rx_ring.tail + 1U) % UART_RX_RING_SIZE);
    return TRUE;
}

void uart_comm_init(void)
{
    s_rx_ring.head = 0U;
    s_rx_ring.tail = 0U;
    bap_uart_init();
    /* JustFloat 仅 TX：暂不开启 RBNE，避免半双工自收 + I2C 抢中断 */
}

void uart_comm_rx_enable(void)
{
    usart_interrupt_enable(COMM_USART, USART_INT_RBNE);
    nvic_irq_enable(COMM_USART_IRQn, 2U, 0U);
}

void USART1_IRQHandler(void)
{
    if (RESET != usart_interrupt_flag_get(COMM_USART, USART_INT_FLAG_RBNE)) {
        uint8_t byte = (uint8_t)usart_data_receive(COMM_USART);
        ring_put(byte);
    }
}

uint16_t uart_rx_available(void)
{
    uint16_t head = s_rx_ring.head;
    uint16_t tail = s_rx_ring.tail;

    if (head >= tail) {
        return (uint16_t)(head - tail);
    }
    return (uint16_t)(UART_RX_RING_SIZE - tail + head);
}

uint8_t uart_rx_pop(uint8_t *byte)
{
    return ring_get(byte) ? 1U : 0U;
}

void uart_comm_poll(void)
{
    /* C-01 协议：在此从环形缓冲取字节组帧、解析命令 */
}
