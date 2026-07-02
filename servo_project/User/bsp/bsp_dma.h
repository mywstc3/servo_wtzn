#ifndef BSP_DMA_H
#define BSP_DMA_H

#include "gd32f1x0.h"

#define ADC_DMA_BUF_LEN    2U    /* 扫描 2 通道：CH3 + CH4 */

extern uint16_t adc_dma_buf[ADC_DMA_BUF_LEN];

void DMA_init(void);

#endif
