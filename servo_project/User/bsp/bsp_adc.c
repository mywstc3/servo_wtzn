#include "bsp_adc.h"
#include "bsp_dma.h"

void adc_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_ADC);
    rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);

    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_3 | GPIO_PIN_4);

    adc_deinit();
    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
    adc_special_function_config(ADC_SCAN_MODE, ENABLE);
    adc_channel_length_config(ADC_REGULAR_CHANNEL, ADC_DMA_BUF_LEN);
    adc_regular_channel_config(0U, ADC_CHANNEL_3, ADC_SAMPLETIME_55POINT5);
    adc_regular_channel_config(1U, ADC_CHANNEL_4, ADC_SAMPLETIME_55POINT5);

    adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_T2_TRGO);
    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);

    adc_enable();
    for (volatile uint32_t i = 0U; i < 10000U; i++) { }
    adc_calibration_enable();

    DMA_init();
    adc_dma_mode_enable();
}
