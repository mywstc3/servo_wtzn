#include "pid.h"

static float pid_clamp(float x, float min_val, float max_val)
{
    if (x > max_val) {
        return max_val;
    }
    if (x < min_val) {
        return min_val;
    }
    return x;
}

void pid_init(pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->input_max = 0.0f;
    pid->integral_max = 0.0f;
    pid->output_max = 0.0f;
}

void pid_set_limits(pid_t *pid, float input_max, float integral_max, float output_max)
{
    pid->input_max = input_max;
    pid->integral_max = integral_max;
    pid->output_max = output_max;
}

void speed_pid(pid_t *motor_speed_pid, float *target_speed, float *current_speed_degree)
{
    float target = *target_speed;
    float d_term;

    if (motor_speed_pid->input_max > 0.0f) {
        target = pid_clamp(target, -motor_speed_pid->input_max, motor_speed_pid->input_max);
    }

    motor_speed_pid->target = target;
    motor_speed_pid->current = *current_speed_degree;
    motor_speed_pid->error = motor_speed_pid->target - motor_speed_pid->current;

    motor_speed_pid->integral += motor_speed_pid->ki * motor_speed_pid->error;
    if (motor_speed_pid->integral_max > 0.0f) {
        motor_speed_pid->integral = pid_clamp(motor_speed_pid->integral,
                                              -motor_speed_pid->integral_max,
                                              motor_speed_pid->integral_max);
    }

    d_term = motor_speed_pid->kd * (motor_speed_pid->error - motor_speed_pid->last_error);
    motor_speed_pid->output = motor_speed_pid->kp * motor_speed_pid->error
                            + motor_speed_pid->integral
                            + d_term;

    if (motor_speed_pid->output_max > 0.0f) {
        motor_speed_pid->output = pid_clamp(motor_speed_pid->output,
                                            -motor_speed_pid->output_max,
                                            motor_speed_pid->output_max);
    }

    motor_speed_pid->last_error = motor_speed_pid->error;
}

void location_pid(pid_t *motor_angle_pid, motor_control_context_t *control, sensor_data_t *sensor)
{
    float target = control->target_angle;
    float d_term;

    motor_angle_pid->target = target;
    motor_angle_pid->current = sensor->motor_angle_multi_degree;
    motor_angle_pid->error = motor_angle_pid->target - motor_angle_pid->current;

    motor_angle_pid->integral += motor_angle_pid->ki * motor_angle_pid->error;
    if (motor_angle_pid->integral_max > 0.0f) {
        motor_angle_pid->integral = pid_clamp(motor_angle_pid->integral,
                                              -motor_angle_pid->integral_max,
                                              motor_angle_pid->integral_max);
    }

    d_term = motor_angle_pid->kd * (motor_angle_pid->error - motor_angle_pid->last_error);
    motor_angle_pid->output = motor_angle_pid->kp * motor_angle_pid->error
                            + motor_angle_pid->integral
                            + d_term;

    if (motor_angle_pid->output_max > 0.0f) {
        motor_angle_pid->output = pid_clamp(motor_angle_pid->output,
                                            -motor_angle_pid->output_max,
                                            motor_angle_pid->output_max);
    }

    motor_angle_pid->last_error = motor_angle_pid->error;
}
