#ifndef __SPEED_H__
#define __SPEED_H__

#include <stdint.h>
#include "servo_config.h"
#include "encoder.h"

#define SPEED_LPF_ALPHA         0.3f
#define MOTOR_SPEED_CALC_HZ     ENCODER_UPDATE_HZ

typedef struct
{
    int32_t turn_count;
    float last_angle_degree;
    float last_multi_degree;
    uint8_t inited;
} speed_observer_t;

extern speed_observer_t g_speed_observer;

void speed_observer_reset(speed_observer_t *observer, sensor_data_t *sensor);
void speed_update(speed_observer_t *observer, sensor_data_t *sensor);

#endif
