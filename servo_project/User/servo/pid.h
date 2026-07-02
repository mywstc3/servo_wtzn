#ifndef __PID_H__
#define __PID_H__
#include "servo_config.h"


void pid_init(pid_t *pid, float kp, float ki, float kd);
void pid_set_limits(pid_t *pid, float input_max, float integral_max, float output_max);
void speed_pid(pid_t *motor_speed_pid, float *target_speed, float *current_speed_degree);
void location_pid(pid_t *motor_angle_pid, motor_control_context_t *control, sensor_data_t *sensor);

#endif