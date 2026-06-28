#ifndef __MOTOR_H
#define __MOTOR_H

#include "gd32f1x0.h"

void motor_init(void);
void motor_enable(void);
void motor_disable(void);
void motor_set_duty(int16_t duty);

#endif