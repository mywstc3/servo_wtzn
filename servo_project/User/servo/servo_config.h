#ifndef __SERVO_CONFIG_H
#define __SERVO_CONFIG_H

#include "stdbool.h"
#include "stdint.h"

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
    bool motor_location_flag;
    bool motor_adc_flag;
    bool motor_encoder_flag;
    bool motor_current_flag;
    bool motor_voltage_flag;
}flag_t;

typedef struct
{
    float motor_adc_i_raw;
    float motor_adc_v_raw;
    float motor_encoder_raw;
    float motor_encoder_radian;
    float motor_encoder_degree;
    float start_angle_radian;
    float angle_radian_diff;
    float motor_angle_multi_degree;
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
    float target_angle;
    float target_speed;
    float target_a_speed;
    float target_electricity;
    float plan_speed;
    float plan_angle;
} motor_control_context_t;



typedef struct {
    sensor_data_t sensor;
    motor_control_context_t control;
    flag_t flag;
    motor_pid_t motor_pid;
} motor_context_t;

extern motor_context_t motor_context;





#endif

