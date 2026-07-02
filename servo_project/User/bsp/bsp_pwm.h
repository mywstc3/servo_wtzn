#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include "gd32f1x0.h"

void pwm_init(void);
void pwm_set_duty(int16_t duty);

#endif
