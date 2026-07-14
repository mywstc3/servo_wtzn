#ifndef __SPEED_H__
#define __SPEED_H__

#include <stdint.h>
#include "servo_config.h"
#include "encoder.h"

/* 先对多圈角做滑动平均，再对滤波角做固定跨距差分 */
#define SPEED_MA_WIN            4U
#define SPEED_DIFF_SPAN         16U   /* 滤波角差分跨距（约 32 ms @ 1 kHz） */
#define SPEED_FILT_WIN          (SPEED_DIFF_SPAN + 1U)
#define SPEED_LPF_ALPHA         0.3f
#define MOTOR_SPEED_CALC_HZ     ENCODER_UPDATE_HZ

typedef struct
{
    int32_t accum_raw;
    uint16_t last_raw;
    float ma_buf[SPEED_MA_WIN];
    uint8_t ma_idx;
    uint8_t ma_count;
    float filt_angle[SPEED_FILT_WIN];
    uint32_t filt_tick[SPEED_FILT_WIN];
    uint8_t filt_idx;
    uint8_t filt_count;
    uint8_t inited;
} speed_observer_t;

extern speed_observer_t g_speed_observer;

void speed_observer_reset(speed_observer_t *observer, sensor_data_t *sensor);
void speed_update(speed_observer_t *observer, sensor_data_t *sensor);

#endif
