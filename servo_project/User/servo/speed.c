#include "speed.h"

speed_observer_t g_speed_observer;

static void angle_degree_diff_limit(speed_observer_t *observer, sensor_data_t *sensor)
{
    float angle_degree_diff =
        sensor->motor_encoder_degree - observer->last_angle_degree;

    if (angle_degree_diff > 180.0f) {
        angle_degree_diff -= 360.0f;
        observer->turn_count--;
    } else if (angle_degree_diff < -180.0f) {
        angle_degree_diff += 360.0f;
        observer->turn_count++;
    }

    observer->last_angle_degree = sensor->motor_encoder_degree;
}

static void angle_multi_degree(speed_observer_t *observer, sensor_data_t *sensor)
{
    sensor->motor_angle_multi_degree =
        (float)observer->turn_count * 360.0f + sensor->motor_encoder_degree;
}

void speed_observer_reset(speed_observer_t *observer, sensor_data_t *sensor)
{
    observer->turn_count = 0;
    observer->last_angle_degree = sensor->motor_encoder_degree;
    observer->last_multi_degree = sensor->motor_angle_multi_degree;
    observer->inited = 0U;
}

void speed_update(speed_observer_t *observer, sensor_data_t *sensor)
{
    float diff_degree;
    float speed_raw;

    angle_degree_diff_limit(observer, sensor);
    angle_multi_degree(observer, sensor);

    if (observer->inited == 0U) {
        observer->last_multi_degree = sensor->motor_angle_multi_degree;
        observer->inited = 1U;
        return;
    }

    diff_degree = sensor->motor_angle_multi_degree - observer->last_multi_degree;
    observer->last_multi_degree = sensor->motor_angle_multi_degree;

    speed_raw = diff_degree * (float)MOTOR_SPEED_CALC_HZ;
    sensor->motor_speed_degree =
        (1.0f - SPEED_LPF_ALPHA) * sensor->motor_speed_degree
        + SPEED_LPF_ALPHA * speed_raw;
    sensor->motor_speed_radian =
        sensor->motor_speed_degree * 3.14159265f / 180.0f;

    sensor->motor_speed_raw = sensor->motor_speed_degree/360.0f*4096.0f;
}
