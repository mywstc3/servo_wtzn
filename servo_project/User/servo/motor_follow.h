#ifndef __MOTOR_FOLLOW_H
#define __MOTOR_FOLLOW_H

#include "servo_config.h"
#include <stdint.h>

void motor_follow_init(void);
void motor_follow_reset(void);
void motor_follow_poll(void);

uint8_t motor_follow_is_active(void);
float motor_follow_get_est_speed(void);

/*
 * 高频 GOAL_POS 流检测与跟随处理。
 * 返回 1：已按跟随模式更新（调用方跳过 angle_control_plan_reset）。
 * 返回 0：应按原点到点逻辑 reset。
 */
uint8_t motor_follow_on_goal_pos(motor_control_context_t *ctrl,
    float new_target,
    float actual_speed);

#endif
