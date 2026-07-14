#include "speed.h"
#include "time.h"

speed_observer_t g_speed_observer;

extern volatile uint32_t g_control_tick;

static int32_t raw_diff_wrap(int32_t diff)
{
    if (diff > 2048) {
        return diff - 4096;
    }
    if (diff < -2048) {
        return diff + 4096;
    }
    return diff;
}

static float raw_to_degree(int32_t raw)
{
    return (float)raw * 360.0f / 4096.0f;
}

static float speed_ma_push(speed_observer_t *observer, float angle)
{
    float sum = 0.0f;
    uint8_t i;
    uint8_t n;

    observer->ma_buf[observer->ma_idx] = angle;
    observer->ma_idx = (uint8_t)((observer->ma_idx + 1U) % SPEED_MA_WIN);
    if (observer->ma_count < SPEED_MA_WIN) {
        observer->ma_count++;
    }

    n = observer->ma_count;
    for (i = 0U; i < n; i++) {
        sum += observer->ma_buf[i];
    }
    return sum / (float)n;
}

static void speed_filt_push(speed_observer_t *observer, float angle, uint32_t tick)
{
    observer->filt_angle[observer->filt_idx] = angle;
    observer->filt_tick[observer->filt_idx] = tick;
    observer->filt_idx = (uint8_t)((observer->filt_idx + 1U) % SPEED_FILT_WIN);
    if (observer->filt_count < SPEED_FILT_WIN) {
        observer->filt_count++;
    }
}

/* 滑动滤波后再差分：用滤波角窗口首尾 / 真实 Δt */
static float speed_filt_diff(speed_observer_t *observer)
{
    uint8_t newest;
    uint8_t oldest;
    uint32_t dt_tick;
    float dt_sec;

    if (observer->filt_count <= SPEED_DIFF_SPAN) {
        return 0.0f;
    }

    newest = (uint8_t)((observer->filt_idx + SPEED_FILT_WIN - 1U) % SPEED_FILT_WIN);
    oldest = (uint8_t)((observer->filt_idx + SPEED_FILT_WIN - 1U - SPEED_DIFF_SPAN)
        % SPEED_FILT_WIN);
    dt_tick = observer->filt_tick[newest] - observer->filt_tick[oldest];
    if (dt_tick == 0U) {
        return 0.0f;
    }

    dt_sec = (float)dt_tick / (float)CONTROL_LOOP_HZ;
    return (observer->filt_angle[newest] - observer->filt_angle[oldest]) / dt_sec;
}

void speed_observer_reset(speed_observer_t *observer, sensor_data_t *sensor)
{
    uint16_t raw = sensor->motor_encoder_raw_u16;
    uint8_t i;

    observer->last_raw = raw;
    observer->accum_raw = (int32_t)raw;
    sensor->motor_angle_multi_degree = raw_to_degree(observer->accum_raw);

    observer->ma_idx = 0U;
    observer->ma_count = 0U;
    for (i = 0U; i < SPEED_MA_WIN; i++) {
        observer->ma_buf[i] = 0.0f;
    }

    observer->filt_idx = 0U;
    observer->filt_count = 0U;
    for (i = 0U; i < SPEED_FILT_WIN; i++) {
        observer->filt_angle[i] = 0.0f;
        observer->filt_tick[i] = 0U;
    }

    (void)speed_ma_push(observer, sensor->motor_angle_multi_degree);
    speed_filt_push(observer, sensor->motor_angle_multi_degree, g_control_tick);
    observer->inited = 0U;
}

void speed_update(speed_observer_t *observer, sensor_data_t *sensor)
{
    uint16_t raw = sensor->motor_encoder_raw_u16;
    int32_t diff_raw;
    float angle_filt;
    float speed_raw;

    diff_raw = raw_diff_wrap((int32_t)raw - (int32_t)observer->last_raw);
    observer->last_raw = raw;
    observer->accum_raw += diff_raw;
    sensor->motor_angle_multi_degree = raw_to_degree(observer->accum_raw);

    angle_filt = speed_ma_push(observer, sensor->motor_angle_multi_degree);
    speed_filt_push(observer, angle_filt, g_control_tick);

    if (observer->inited == 0U) {
        observer->inited = 1U;
        sensor->motor_speed_degree = 0.0f;
        sensor->motor_speed_radian = 0.0f;
        sensor->motor_speed_raw = 0.0f;
        return;
    }

    speed_raw = speed_filt_diff(observer);
    sensor->motor_speed_degree =
        (1.0f - SPEED_LPF_ALPHA) * sensor->motor_speed_degree
        + SPEED_LPF_ALPHA * speed_raw;
    sensor->motor_speed_radian =
        sensor->motor_speed_degree * 3.14159265f / 180.0f;
    sensor->motor_speed_raw = sensor->motor_speed_degree / 360.0f * 4096.0f;
}
