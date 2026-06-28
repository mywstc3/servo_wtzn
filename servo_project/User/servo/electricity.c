#include "electricity.h"
#include "bsp/bsp_adc.h"

extern uint16_t adc_dma_buf[ADC_DMA_BUF_LEN];
void ADC_CMP_IRQHandler(void)
{
    if (RESET != adc_interrupt_flag_get(ADC_INT_FLAG_EOC)) {
        adc_interrupt_flag_clear(ADC_INT_FLAG_EOC);

        uint16_t i_raw = adc_dma_buf[0];   /* CH3 电流 */
        uint16_t v_raw = adc_dma_buf[1];   /* CH4 电压 */

        /* 1. raw → i_bus / v_bus（标定公式见 M-02 笔记） */
        /* 2. 电流环 PI：i_target → duty */
        /* 3. motor_set_duty(duty) 或写 TIMER2 CCR（§4.10 影子） */
        /* 4. 可选：过流/欠压判断 → motor_disable() */
    }
}