#include "bap_uart.h"
#include "gd32f1x0_dma.h"
#include <stddef.h>

static volatile uint8_t s_tx_dma_busy;
static volatile uint8_t s_tx_dma_wait_tc;

static void bap_uart_tx_dma_finish(void)
{
    if (RESET != usart_flag_get(COMM_USART, USART_FLAG_TC)) {
        usart_flag_clear(COMM_USART, USART_FLAG_TC);
        s_tx_dma_wait_tc = 0U;
        s_tx_dma_busy = 0U;
    }
}

static void bap_uart_tx_dma_hw_init(void)
{
    dma_parameter_struct dma_init_struct;

    rcu_periph_clock_enable(RCU_DMA);
    dma_deinit(DMA_CH3);
    dma_struct_para_init(&dma_init_struct);
    dma_init_struct.periph_addr  = (uint32_t)&USART_TDATA(COMM_USART);
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.number       = 1U;
    dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.priority     = DMA_PRIORITY_LOW;
    dma_init(DMA_CH3, &dma_init_struct);
    dma_circulation_disable(DMA_CH3);

    dma_interrupt_enable(DMA_CH3, DMA_INT_FTF);
    nvic_irq_enable(DMA_Channel3_4_IRQn, 2U, 0U);
    usart_dma_transmit_config(COMM_USART, USART_DENT_ENABLE);
}

void bap_uart_init(void)
{
    s_tx_dma_busy = 0U;
    s_tx_dma_wait_tc = 0U;

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(COMM_USART_RCU);

    gpio_af_set(COMM_GPIO_PORT, COMM_GPIO_AF, COMM_GPIO_PIN);
    gpio_mode_set(COMM_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, COMM_GPIO_PIN);
    gpio_output_options_set(COMM_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, COMM_GPIO_PIN);

    usart_deinit(COMM_USART);
    usart_baudrate_set(COMM_USART, COMM_BAUDRATE);
    usart_parity_config(COMM_USART, USART_PM_NONE);
    usart_word_length_set(COMM_USART, USART_WL_8BIT);
    usart_stop_bit_set(COMM_USART, USART_STB_1BIT);
    usart_halfduplex_enable(COMM_USART);
    usart_transmit_config(COMM_USART, USART_TRANSMIT_ENABLE);
    usart_receive_config(COMM_USART, USART_RECEIVE_ENABLE);
    usart_enable(COMM_USART);

    bap_uart_tx_dma_hw_init();
}

void usart_putc(uint32_t usart, uint8_t ch)
{
    while (RESET == usart_flag_get(usart, USART_FLAG_TBE)) { }
    usart_data_transmit(usart, ch);
    while (RESET == usart_flag_get(usart, USART_FLAG_TC)) { }
    usart_flag_clear(usart, USART_FLAG_TC);
}

bool usart_getc_timeout(uint32_t usart, uint8_t *ch, uint32_t timeout_loops)
{
    while (RESET == usart_flag_get(usart, USART_FLAG_RBNE)) {
        if (timeout_loops-- == 0U) {
            return FALSE;
        }
    }
    *ch = (uint8_t)usart_data_receive(usart);
    return TRUE;
}

void bap_uart_send(uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0U; i < length; i++) {
        usart_putc(COMM_USART, data[i]);
    }
}

uint8_t bap_uart_receive(uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0U; i < length; i++) {
        if (!usart_getc_timeout(COMM_USART, &data[i], 100000U)) {
            return (uint8_t)i;
        }
    }
    return (uint8_t)length;
}

bool bap_uart_tx_dma_busy(void)
{
    if (s_tx_dma_wait_tc != 0U) {
        bap_uart_tx_dma_finish();
    }
    return s_tx_dma_busy != 0U;
}

bool bap_uart_tx_dma_start(const uint8_t *data, uint16_t length)
{
    if (s_tx_dma_busy || data == NULL || length == 0U) {
        return FALSE;
    }

    s_tx_dma_busy = 1U;
    dma_channel_disable(DMA_CH3);
    dma_memory_address_config(DMA_CH3, (uint32_t)data);
    dma_transfer_number_config(DMA_CH3, length);
    dma_interrupt_flag_clear(DMA_CH3, DMA_INT_FLAG_FTF);
    dma_channel_enable(DMA_CH3);
    return TRUE;
}

void DMA_Channel3_4_IRQHandler(void)
{
    if (RESET != dma_interrupt_flag_get(DMA_CH3, DMA_INT_FLAG_FTF)) {
        dma_interrupt_flag_clear(DMA_CH3, DMA_INT_FLAG_FTF);
        dma_channel_disable(DMA_CH3);
        s_tx_dma_wait_tc = 1U;
    }
}
