#include "motor_follow.h"
#include "time.h"

extern volatile uint32_t g_control_tick;

static uint8_t s_follow_active;
static uint8_t s_stream_hit_cnt;
static uint8_t s_stream_miss_cnt;
static uint32_t s_last_goal_tick;
static float s_last_goal_angle;
static uint8_t s_goal_seen;
static float s_v_est;

static float follow_absf(float v)
{
    return (v >= 0.0f) ? v : -v;
}

static float follow_clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

void motor_follow_init(void)
{
    motor_follow_reset();
}

void motor_follow_reset(void)
{
    s_follow_active = 0U;
    s_stream_hit_cnt = 0U;
    s_stream_miss_cnt = 0U;
    s_last_goal_tick = 0U;
    s_last_goal_angle = 0.0f;
    s_goal_seen = 0U;
    s_v_est = 0.0f;
}

uint8_t motor_follow_is_active(void)
{
    return s_follow_active;
}

float motor_follow_get_est_speed(void)
{
    return s_v_est;
}

void motor_follow_poll(void)
{
    uint32_t now;
    uint32_t idle_ms;

    if (s_follow_active == 0U || s_goal_seen == 0U) {
        return;
    }

    now = g_control_tick;
    idle_ms = now - s_last_goal_tick;
    if (idle_ms > (uint32_t)MOTOR_FOLLOW_IDLE_DT_MS) {
        s_follow_active = 0U;
        s_stream_hit_cnt = 0U;
        s_stream_miss_cnt = 0U;
    }
}

static void follow_update_speed(motor_control_context_t *ctrl, float new_target,
    uint32_t dt_ms, float actual_speed)
{
    float v_raw;
    float v_cmd;

    if (s_goal_seen != 0U && dt_ms > 0U && dt_ms < 2000U) {
        v_raw = (new_target - s_last_goal_angle) / ((float)dt_ms * 0.001f);
        v_raw = follow_clampf(v_raw,
            -MOTOR_FOLLOW_V_CAP_DEG, MOTOR_FOLLOW_V_CAP_DEG);
        s_v_est += MOTOR_FOLLOW_V_LPF_ALPHA * (v_raw - s_v_est);
    } else if (follow_absf(actual_speed) > 1.0f) {
        s_v_est = actual_speed;
    }

    v_cmd = follow_absf(s_v_est) * MOTOR_FOLLOW_V_MARGIN;
    if (v_cmd < MOTOR_FOLLOW_V_MIN_DEG) {
        v_cmd = MOTOR_FOLLOW_V_MIN_DEG;
    }
    if (v_cmd > MOTOR_FOLLOW_V_CAP_DEG) {
        v_cmd = MOTOR_FOLLOW_V_CAP_DEG;
    }
    ctrl->target_speed = v_cmd;
}

uint8_t motor_follow_on_goal_pos(motor_control_context_t *ctrl,
    float new_target,
    float actual_speed)
{
    uint32_t now = g_control_tick;
    uint32_t dt_ms = 0U;
    float delta_deg;
    uint8_t stream_sample = 0U;

    if (s_goal_seen != 0U) {
        dt_ms = now - s_last_goal_tick;
    }

    delta_deg = follow_absf(new_target - s_last_goal_angle);

    if (delta_deg > MOTOR_FOLLOW_JUMP_DEG) {
        s_follow_active = 0U;
        s_stream_hit_cnt = 0U;
        s_stream_miss_cnt = 0U;
    } else if (s_goal_seen != 0U
        && dt_ms > 0U
        && dt_ms <= (uint32_t)MOTOR_FOLLOW_STREAM_DT_MAX_MS
        && delta_deg >= MOTOR_FOLLOW_STEP_MIN_DEG
        && delta_deg <= MOTOR_FOLLOW_STEP_MAX_DEG) {
        stream_sample = 1U;
        s_stream_hit_cnt++;
        s_stream_miss_cnt = 0U;
    } else if (s_goal_seen != 0U) {
        s_stream_hit_cnt = 0U;
        if (s_follow_active != 0U) {
            s_stream_miss_cnt++;
        }
    }

    if (s_follow_active == 0U) {
        if (s_stream_hit_cnt >= MOTOR_FOLLOW_ENTER_COUNT) {
            s_follow_active = 1U;
        }
    } else if (stream_sample == 0U
        && s_stream_miss_cnt >= MOTOR_FOLLOW_EXIT_COUNT) {
        s_follow_active = 0U;
        s_stream_hit_cnt = 0U;
        s_stream_miss_cnt = 0U;
    }

    ctrl->target_angle = new_target;

    if (s_follow_active != 0U) {
        follow_update_speed(ctrl, new_target, dt_ms, actual_speed);
    }

    s_last_goal_angle = new_target;
    s_last_goal_tick = now;
    s_goal_seen = 1U;

    return s_follow_active;
}
