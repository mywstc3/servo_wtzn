#include "motor.h"
#include "bsp/bsp_gpio.h"
#include "bsp/bsp_pwm.h"

void motor_init(void)
{
    pwm_init();
    motor_enable();
    pwm_start();
}

void motor_enable(void)
{
    gpio_bit_set(GPIOA, GPIO_PIN_6);
}

void motor_disable(void)
{
    pwm_set_duty(0);
    gpio_bit_reset(GPIOA, GPIO_PIN_6);
}

void motor_set_duty(int16_t duty)
{
    pwm_set_duty(duty);
}
