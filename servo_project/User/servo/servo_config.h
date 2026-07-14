#ifndef __SERVO_CONFIG_H
#define __SERVO_CONFIG_H

#include "gd32f1x0.h"
#include <stdint.h>

#define POS_PLAN_DEADZONE_DEG   1.0f    /* 进入精调区的位置误差 (°) */
#define POS_STOP_HYSTERESIS_DEG 0.25f   /* 退出精调区滞回 (°) */
#define POS_TRIM_SPEED_MAX      60.0f   /* 精调区位置 PD 最大速度输出 (°/s) */
#define POS_PLAN_ARRIVE_DEG     0.05f   /* 粗调规划角到达判定 (°) */
#define POS_DECEL_HYSTERESIS    1.05f   /* 减速区滞回，减轻加减速边界抖振 */

/* 高频 GOAL 跟随子模式：协议仍为位置模式，内部不 reset、禁用精调 */
#define MOTOR_FOLLOW_STREAM_DT_MAX_MS  100U
#define MOTOR_FOLLOW_IDLE_DT_MS        400U
#define MOTOR_FOLLOW_STEP_MAX_DEG      10.0f
#define MOTOR_FOLLOW_STEP_MIN_DEG      0.05f
#define MOTOR_FOLLOW_JUMP_DEG          25.0f
#define MOTOR_FOLLOW_ENTER_COUNT       4U
#define MOTOR_FOLLOW_EXIT_COUNT        2U
#define MOTOR_FOLLOW_V_LPF_ALPHA       0.35f
#define MOTOR_FOLLOW_V_MARGIN          1.3f
#define MOTOR_FOLLOW_V_MIN_DEG         5.0f
#define MOTOR_FOLLOW_V_CAP_DEG         120.0f

/* Stribeck：仅“零速启动 / 换向”边沿 oneshot；位置精调区强制关闭 */
#define MOTOR_STRIBECK_PLAN_THRESH       2.0f   /* |plan| 边沿判定阈 (°/s) */
#define MOTOR_STRIBECK_ENTRY_SPEED_DEG   5.0f   /* 零速启动：|actual| 须低于此 */
#define MOTOR_STRIBECK_EXIT_SPEED_DEG    12.0f  /* 同向超此速度退出助推 */
#define MOTOR_STRIBECK_MIN_HOLD_MS       20U    /* 最短保持 */
#define MOTOR_STRIBECK_RAMP_DUTY_S       6000.0f /* 退出斜坡；入助推首拍满量 */

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
    uint16_t motor_encoder_raw_u16;
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

