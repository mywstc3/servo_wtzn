#include "bsp/bap_uart.h"
#include "uart.h"
#include "sts_proto.h"

uart_rx_ring_t g_uart_rx_ring;
static volatile uint8_t s_uart_rx_blocked;

volatile uint32_t g_uart_rx_irq_cnt;
volatile uint32_t g_uart_rx_put_cnt;
volatile uint32_t g_uart_rx_skip_cnt;
volatile uint32_t g_uart_rx_drop_cnt;
volatile uint32_t g_uart_dbg_blocked;
volatile uint32_t g_uart_dbg_stat;
volatile uint32_t g_uart_dbg_ctl0;

static void uart_delay_us(uint32_t us)
{
    uint32_t i;

    for (i = 0U; i < (us * 72U); i++) {
        __NOP();
    }
}

static void uart_dbg_refresh(void)
{
    g_uart_dbg_stat = USART_STAT(COMM_USART);
    g_uart_dbg_ctl0 = USART_CTL0(COMM_USART);
    g_uart_dbg_blocked = (uint32_t)s_uart_rx_blocked;
}

static void uart_hw_clear_errors(void)
{
    if (RESET != usart_flag_get(COMM_USART, USART_FLAG_ORERR)) {
        (void)usart_data_receive(COMM_USART);
        usart_flag_clear(COMM_USART, USART_FLAG_ORERR);
    }
    if (RESET != usart_flag_get(COMM_USART, USART_FLAG_FERR)) {
        (void)usart_data_receive(COMM_USART);
        usart_flag_clear(COMM_USART, USART_FLAG_FERR);
    }
    if (RESET != usart_flag_get(COMM_USART, USART_FLAG_NERR)) {
        (void)usart_data_receive(COMM_USART);
        usart_flag_clear(COMM_USART, USART_FLAG_NERR);
    }
    if (RESET != usart_flag_get(COMM_USART, USART_FLAG_PERR)) {
        (void)usart_data_receive(COMM_USART);
        usart_flag_clear(COMM_USART, USART_FLAG_PERR);
    }
}

static void uart_hw_init_recover(void)
{
    uart_hw_clear_errors();
    while (RESET != usart_flag_get(COMM_USART, USART_FLAG_RBNE)) {
        (void)usart_data_receive(COMM_USART);
    }
    usart_receive_config(COMM_USART, USART_RECEIVE_ENABLE);
    usart_transmit_config(COMM_USART, USART_TRANSMIT_ENABLE);
    usart_interrupt_enable(COMM_USART, USART_INT_RBNE);
    nvic_irq_enable(COMM_USART_IRQn, 2U, 0U);
    uart_dbg_refresh();
}

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
    g_uart_rx_irq_cnt = 0U;
    g_uart_rx_put_cnt = 0U;
    g_uart_rx_skip_cnt = 0U;
    g_uart_rx_drop_cnt = 0U;
    s_uart_rx_blocked = 0U;
    bap_uart_init();
    uart_hw_init_recover();
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

/*
 * HDEN 半双工勿切换 REN/TEN（与 HDEN 冲突）。
 * 发期间保持 RBNE，用 s_uart_rx_blocked 丢弃自发回波。
 */
void uart_comm_tx_begin(void)
{
    s_uart_rx_blocked = 1U;
    uart_dbg_refresh();
}

void uart_comm_tx_end(void)
{
    uart_delay_us(50U);
    uart_hw_clear_errors();
    s_uart_rx_blocked = 0U;
    uart_dbg_refresh();
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
    uart_hw_init_recover();
}

void USART1_IRQHandler(void)
{
    if (RESET != usart_flag_get(COMM_USART, USART_FLAG_ORERR)) {
        (void)usart_data_receive(COMM_USART);
        usart_flag_clear(COMM_USART, USART_FLAG_ORERR);
    }

    if (RESET != usart_interrupt_flag_get(COMM_USART, USART_INT_FLAG_RBNE)) {
        uint8_t byte = (uint8_t)usart_data_receive(COMM_USART);
        g_uart_rx_irq_cnt++;
        if (s_uart_rx_blocked != 0U) {
            g_uart_rx_skip_cnt++;
            return;
        }
        if (ring_put(byte)) {
            g_uart_rx_put_cnt++;
        } else {
            g_uart_rx_drop_cnt++;
        }
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
