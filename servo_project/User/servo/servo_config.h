#ifndef __SERVO_CONFIG_H
#define __SERVO_CONFIG_H

#include "gd32f1x0.h"
#include <stdint.h>

#define POS_PLAN_DEADZONE_DEG   1.0f    /* 进入精调区的位置误差 (°) */
#define POS_STOP_HYSTERESIS_DEG 0.25f    /* 退出精调区滞回 (°) */
#define POS_TRIM_SPEED_MAX      60.0f   /* 精调区位置 PD 最大速度输出 (°/s) */

/* Stribeck 摩擦补偿：静止≈start_duty，随实际速度指数衰减；仅填补 PID 不足 */
#define MOTOR_STRIBECK_SPEED_DEG     8.0f   /* 衰减特征速度 (°/s) */
#define MOTOR_STRIBECK_MOVE_DUTY       0U     /* 运动摩擦 duty */
#define MOTOR_STRIBECK_PLAN_THRESH     3.0f   /* |plan_speed| 低于此值不补偿 (°/s) */
#define MOTOR_STRIBECK_PLAN_FULL_DEG   45.0f  /* plan 达此速度时用满补偿 */
#define MOTOR_STRIBECK_TRACK_RATIO     0.75f  /* 实际速度达 plan 此比例后退出 */

typedef struct
{
    float kp;
    float ki;
    float kd;
    float target;
    float current;
    float error;
    float last_error;
    float previous_error;
    float integral;
    float output;
    float delta_output;
    float last_output;
    float input_max;      /* 目标输入绝对值上限，0=不限制 */
    float integral_max;   /* 积分项绝对值上限，0=不限制 */
    float output_max;     /* 输出绝对值上限，0=不限制 */
} pid_t;

typedef struct
{
    pid_t motor_electricity_pid;
    pid_t motor_speed_pid;
    pid_t motor_angle_pid;
} motor_pid_t;

typedef struct
{
    bool angle_start_flag;
    bool motor_start_flag;
    bool motor_uart_flag;
    bool motor_speed_flag;
    bool motor_speed_pid_flag;
    bool motor_location_pid_flag;
    bool motor_adc_flag;
    bool motor_encoder_flag;
}flag_t;

typedef struct
{
    float motor_adc_i_raw;
    float motor_adc_v_raw;
    float motor_adc_i_bus;
    float motor_adc_v_bus;
    float motor_adc_i_bus_offset;
    float motor_adc_v_bus_offset;
    float motor_encoder_raw;
    float motor_encoder_radian;
    float motor_encoder_degree;
    float start_angle_radian;
    float angle_radian_diff;
    float motor_angle_multi_degree;
    float motor_speed_raw;
    float motor_speed_degree;
    float motor_speed_radian;
} sensor_data_t;

typedef enum
{
    motor_control_mode_null,
    motor_control_mode_torque,
    motor_control_mode_speed_torque,
    motor_control_mode_position_speed_torque,
} motor_control_mode_t;

typedef struct
{
    motor_control_mode_t mode;
    float target_duty;
    float target_angle;
    float target_speed;
    float target_a_speed;
    float target_electricity;
    float plan_speed;
    float plan_angle;
    float plan_start_angle;
} motor_control_context_t;



typedef struct {
    sensor_data_t sensor;
    motor_control_context_t control;
    flag_t flag;
    motor_pid_t motor_pid;
} motor_context_t;

extern motor_context_t motor_context;





#endif

