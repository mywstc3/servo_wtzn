#include "bsp_pwm.h"

void pwm_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_TIMER2);

    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_7);
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_1);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

    timer_parameter_struct timer_initpara;
    timer_oc_parameter_struct timer_ocinitpara;

    timer_deinit(TIMER2);
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler         = 0U;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_DOWN;   /* §4.10：中心对齐 */
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 1799U;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0U;
    timer_init(TIMER2, &timer_initpara);

    timer_channel_output_struct_para_init(&timer_ocinitpara);
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER2, TIMER_CH_1, &timer_ocinitpara);
    timer_channel_output_config(TIMER2, TIMER_CH_3, &timer_ocinitpara);
    timer_channel_output_mode_config(TIMER2, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER2, TIMER_CH_3, TIMER_OC_MODE_PWM0);

    /* TRGO → ADC；CCR/ARR 影子 → ISR 写 duty 下一周期生效 */
    timer_master_output_trigger_source_select(TIMER2, TIMER_TRI_OUT_SRC_UPDATE);
    timer_auto_reload_shadow_enable(TIMER2);
    timer_channel_output_shadow_config(TIMER2, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_shadow_config(TIMER2, TIMER_CH_3, TIMER_OC_SHADOW_ENABLE);

    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, 0U);
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_3, 0U);

    /* 不在此处 timer_enable：等 adc_init + DMA_init 完成后再 pwm_start() */
}

void pwm_start(void)
{
    timer_enable(TIMER2);
}

void pwm_set_duty(int16_t duty)
{
    uint16_t magnitude;

    if (duty > (int16_t)1799U) {
        duty = (int16_t)1799U;
    } else if (duty < -(int16_t)1799U) {
        duty = -(int16_t)1799U;
    }

    if (duty > 0) {
        magnitude = (uint16_t)duty;
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, magnitude);
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_3, 1U);
    } else if (duty < 0) {
        magnitude = (uint16_t)(-duty);
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, 1U);
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_3, magnitude);
    } else {
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, 0U);
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_3, 0U);
    }
}
