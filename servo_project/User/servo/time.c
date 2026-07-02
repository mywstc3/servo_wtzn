#include "time.h"
#include "motor.h"
#include "encoder.h"
#include "servo_config.h"

volatile uint32_t g_control_tick;

void time_init(void)
{
    timer_parameter_struct timer_initpara;

    g_control_tick = 0U;
    motor_context.flag.motor_encoder_flag = TRUE;

    rcu_periph_clock_enable(CONTROL_TIMER_RCU);

    timer_deinit(CONTROL_TIMER);
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler         = 71U;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 999U;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0U;
    timer_init(CONTROL_TIMER, &timer_initpara);

    timer_interrupt_flag_clear(CONTROL_TIMER, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(CONTROL_TIMER, TIMER_INT_UP);
    nvic_irq_enable(CONTROL_TIMER_IRQn, 1U, 0U);

    timer_enable(CONTROL_TIMER);
}

void TIMER13_IRQHandler(void)
{
    if (RESET != timer_interrupt_flag_get(CONTROL_TIMER, TIMER_INT_FLAG_UP)) {
        timer_interrupt_flag_clear(CONTROL_TIMER, TIMER_INT_FLAG_UP);
        g_control_tick++;

        if ((g_control_tick % (CONTROL_LOOP_HZ / ELECTRICITY_UPDATE_HZ)) == 0U) {
            motor_context.flag.motor_adc_flag = TRUE;
        }

        if ((g_control_tick % (CONTROL_LOOP_HZ / ENCODER_UPDATE_HZ)) == 0U) {
            motor_context.flag.motor_encoder_flag = TRUE;
        }

        if ((g_control_tick % (CONTROL_LOOP_HZ / SPEED_UPDATE_HZ)) == 0U) {
            motor_context.flag.motor_speed_flag = TRUE;
        }

        if ((g_control_tick % (CONTROL_LOOP_HZ / SPEED_PID_UPDATE_HZ)) == 0U) {
            motor_context.flag.motor_speed_pid_flag = TRUE;
        }

        if ((g_control_tick % (CONTROL_LOOP_HZ / POSITION_PID_UPDATE_HZ)) == 0U) {
            motor_context.flag.motor_location_pid_flag = TRUE;
        }

        // motor_set_duty(motor_context.control.target_duty);
    }
}
