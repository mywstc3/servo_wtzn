#ifndef __MOTOR_TEST_H
#define __MOTOR_TEST_H

#include <stdint.h>

typedef enum
{
    motor_cmd_torque,
    motor_cmd_speed,
    motor_cmd_position,
} motor_cmd_mode_t;

typedef enum
{
    motor_wave_direct,
    motor_wave_triangle,
    motor_wave_sine,
} motor_wave_type_t;

typedef struct
{
    motor_cmd_mode_t mode;
    motor_wave_type_t wave;
    float val_max;
    float val_min;
    float step;
    float step_ms;
    float setpoint;
    float target;
    float motion_speed;
    float motion_accel;
} motor_cmd_t;

extern motor_cmd_t motor_cmd;

void motor_cmd_init(void);
void motor_test_poll(void);

#endif
