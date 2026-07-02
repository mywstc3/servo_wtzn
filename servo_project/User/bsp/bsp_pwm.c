#include "bsp_pwm.h"

#define MOTOR_PWM_PERIOD    3599U
#define MOTOR_PWM_IDLE_CCR  1U

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
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = MOTOR_PWM_PERIOD;
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
    timer_channel_output_shadow_config(TIMER2, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_shadow_config(TIMER2, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);

    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, 0U);
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_3, 0U);

    /* PWM 周期 UPDATE 触发 ADC 扫描（见 bsp_adc.c / §7.12） */
    timer_master_output_trigger_source_select(TIMER2, TIMER_TRI_OUT_SRC_UPDATE);
    timer_enable(TIMER2);
}

void pwm_set_duty(int16_t duty)
{
    uint16_t magnitude;

    if (duty > (int16_t)MOTOR_PWM_PERIOD) {
        duty = (int16_t)MOTOR_PWM_PERIOD;
    } else if (duty < -(int16_t)MOTOR_PWM_PERIOD) {
        duty = -(int16_t)MOTOR_PWM_PERIOD;
    }

    if (duty > 0) {
        magnitude = (uint16_t)duty;
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, MOTOR_PWM_IDLE_CCR);
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_3, magnitude);
    } else if (duty < 0) {
        magnitude = (uint16_t)(-duty);
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, magnitude);
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_3, MOTOR_PWM_IDLE_CCR);
    } else {
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, 0U);
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_3, 0U);
    }
}
