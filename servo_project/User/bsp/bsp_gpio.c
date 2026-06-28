#include "bsp_gpio.h"

void gpio_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOF);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_mode_set(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_0);
    gpio_bit_reset(GPIOF, GPIO_PIN_0);
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_6);
    gpio_bit_reset(GPIOA, GPIO_PIN_6);
}

void LED_ON(void)
{
    gpio_bit_reset(GPIOF, GPIO_PIN_0);
}

void LED_OFF(void)
{
    gpio_bit_set(GPIOF, GPIO_PIN_0);
}

void GPIO_SET(uint32_t gpio_periph, uint32_t pin)
{
    gpio_bit_set(gpio_periph, pin);
}

void GPIO_RESET(uint32_t gpio_periph, uint32_t pin)
{
    gpio_bit_reset(gpio_periph, pin);
}