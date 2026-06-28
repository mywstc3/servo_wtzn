#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "gd32f1x0.h"

void gpio_init(void);
void LED_ON(void);
void LED_OFF(void);
void GPIO_SET(uint32_t gpio_periph, uint32_t pin);
void GPIO_RESET(uint32_t gpio_periph, uint32_t pin);

#endif