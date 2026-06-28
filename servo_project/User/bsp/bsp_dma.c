#include "bsp_dma.h"
#include "bsp_adc.h"

extern uint16_t adc_dma_buf[ADC_DMA_BUF_LEN];
void DMA_init(void)
{
    dma_parameter_struct dma_init_struct;

    rcu_periph_clock_enable(RCU_DMA);
    dma_deinit(DMA_CH0);
    dma_struct_para_init(&dma_init_struct);
    
    dma_init_struct.periph_addr  = (uint32_t)&ADC_RDATA;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_addr  = (uint32_t)adc_dma_buf;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.number       = ADC_DMA_BUF_LEN;
    dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.priority     = DMA_PRIORITY_HIGH;
    
    dma_init(DMA_CH0, &dma_init_struct);
    dma_circulation_enable(DMA_CH0);
    dma_channel_enable(DMA_CH0);
}