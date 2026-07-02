#ifndef __MOTOR_H
#define __MOTOR_H

#include "gd32f1x0.h"
#include "servo_config.h"

void motor_init(void);
void motor_enable(void);
void motor_disable(void);
void motor_set_duty(int16_t duty);
int16_t motor_get_duty(void);

void speed_control_plan_reset(motor_control_context_t *ctx, float current_speed);
void angle_control_plan_reset(motor_control_context_t *ctx, float current_angle);
void speed_control_plan(motor_control_context_t *ctx);
void angle_control_plan(motor_control_context_t *ctx,
                        float *motor_angle_multi_degree,
                        float *current_speed_degree);
void motor_sync_target_to_actual(motor_context_t *ctx);
void motor_position_home(void);
void motor_control(motor_context_t *context);

#endif