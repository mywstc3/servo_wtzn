#include "motor.h"
#include "bsp/bsp_gpio.h"
#include "bsp/bsp_pwm.h"
#include "pid.h"
#include "servo_config.h"
#include "speed.h"
#include "encoder.h"
#include "systick.h"
#include "time.h"
#include "sts_mem.h"
#include "motor_follow.h"

static int16_t s_motor_duty;
static uint8_t s_pos_stop_hold = 0U;
static uint8_t s_pos_was_stop_hold = 0U;
static float s_plan_last_target = 0.0f;
static uint8_t s_plan_target_valid = 0U;
static float s_stribeck_applied = 0.0f;
static uint8_t s_stribeck_boost_on = 0U;
static uint32_t s_stribeck_boost_enter_tick = 0U;
static float s_stribeck_last_plan = 0.0f;
static int8_t s_stribeck_last_plan_sign = 0;
static uint8_t s_plan_in_decel = 0U;
static uint8_t s_angle_plan_finished = 0U;

static float motor_fabsf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float motor_clampf(float x, float min_val, float max_val)
{
    if (x > max_val) {
        return max_val;
    }
    if (x < min_val) {
        return min_val;
    }
    return x;
}

static int8_t motor_plan_sign(float plan_speed)
{
    if (plan_speed >= MOTOR_STRIBECK_PLAN_THRESH) {
        return 1;
    }
    if (plan_speed <= -MOTOR_STRIBECK_PLAN_THRESH) {
        return -1;
    }
    return 0;
}

/*
 * 静摩擦助推（oneshot）：
 * - 位置精调区（stop hold）强制关闭，避免到位微振
 * - 仅边沿进入：零速启动（|plan| 从低于阈升到阈以上且 |v| 低）
 *             或换向（plan 符号翻转，且 |v|≤EXIT）
 * - 退出后同一段运动不重入，直到下一次边沿
 */
static int16_t motor_apply_stribeck_ff(
    pid_t *speed_pid, int16_t duty, float plan_speed, float actual_speed,
    uint8_t pos_hold)
{
    const float dt = 1.0f / (float)SPEED_PID_UPDATE_HZ;
    uint16_t start_duty = sts_mem_get_min_start_force();
    float abs_plan = motor_fabsf(plan_speed);
    float abs_actual = motor_fabsf(actual_speed);
    float abs_last_plan = motor_fabsf(s_stribeck_last_plan);
    float abs_want;
    float handed_off = 0.0f;
    float want;
    float max_step;
    float out;
    float opp_i;
    float peel;
    uint8_t was_boost_on = s_stribeck_boost_on;
    uint8_t same_dir;
    uint8_t rising_edge;
    uint8_t reverse_edge;
    uint8_t zero_start;
    int8_t plan_sign = motor_plan_sign(plan_speed);
    uint32_t hold_ms = g_control_tick - s_stribeck_boost_enter_tick;

    want = 0.0f;

    /* 精调区：硬关，并刷新边沿记忆，离开精调后可再次检测启动沿 */
    if (pos_hold != 0U || start_duty == 0U) {
        s_stribeck_boost_on = 0U;
        if (was_boost_on != 0U && s_stribeck_applied != 0.0f) {
            handed_off = s_stribeck_applied;
            speed_pid->integral += handed_off;
            if (speed_pid->integral_max > 0.0f) {
                speed_pid->integral = motor_clampf(speed_pid->integral,
                    -speed_pid->integral_max,
                    speed_pid->integral_max);
            }
            s_stribeck_applied = 0.0f;
            duty = (int16_t)((float)duty + handed_off);
            was_boost_on = 0U;
        }
        s_stribeck_last_plan = plan_speed;
        if (plan_sign != 0) {
            s_stribeck_last_plan_sign = plan_sign;
        } else {
            s_stribeck_last_plan_sign = 0;
        }
        out = (float)duty;
        if (out > 3500.0f) {
            out = 3500.0f;
        } else if (out < -3500.0f) {
            out = -3500.0f;
        }
        return (int16_t)out;
    }

    rising_edge = (abs_last_plan < MOTOR_STRIBECK_PLAN_THRESH
        && abs_plan >= MOTOR_STRIBECK_PLAN_THRESH) ? 1U : 0U;
    reverse_edge = (s_stribeck_last_plan_sign != 0
        && plan_sign != 0
        && s_stribeck_last_plan_sign != plan_sign) ? 1U : 0U;
    zero_start = (rising_edge != 0U
        && abs_actual <= MOTOR_STRIBECK_ENTRY_SPEED_DEG) ? 1U : 0U;

    if (s_stribeck_boost_on == 0U) {
        if (zero_start != 0U
            || (reverse_edge != 0U
                && abs_actual <= MOTOR_STRIBECK_EXIT_SPEED_DEG)) {
            s_stribeck_boost_on = 1U;
            s_stribeck_boost_enter_tick = g_control_tick;
        }
    } else {
        same_dir = ((actual_speed * plan_speed) > 0.0f) ? 1U : 0U;
        if ((same_dir != 0U
                && abs_actual >= MOTOR_STRIBECK_EXIT_SPEED_DEG
                && hold_ms >= MOTOR_STRIBECK_MIN_HOLD_MS)
            || abs_plan < MOTOR_STRIBECK_PLAN_THRESH) {
            s_stribeck_boost_on = 0U;
        }
    }

    if (s_stribeck_boost_on != 0U) {
        want = (plan_speed >= 0.0f)
            ? (float)start_duty
            : -(float)start_duty;
    }

    if (was_boost_on != 0U && s_stribeck_boost_on == 0U && s_stribeck_applied != 0.0f) {
        handed_off = s_stribeck_applied;
        speed_pid->integral += handed_off;
        if (speed_pid->integral_max > 0.0f) {
            speed_pid->integral = motor_clampf(speed_pid->integral,
                -speed_pid->integral_max,
                speed_pid->integral_max);
        }
        s_stribeck_applied = 0.0f;
        duty = (int16_t)((float)duty + handed_off);
    }

    if (was_boost_on == 0U && s_stribeck_boost_on != 0U) {
        abs_want = motor_fabsf(want);
        opp_i = speed_pid->integral;
        if ((want > 0.0f && opp_i < 0.0f) || (want < 0.0f && opp_i > 0.0f)) {
            peel = opp_i;
            if (peel > abs_want) {
                peel = abs_want;
            } else if (peel < -abs_want) {
                peel = -abs_want;
            }
            speed_pid->integral -= peel;
            duty = (int16_t)((float)duty - peel);
        }
        s_stribeck_applied = want;
    }

    max_step = MOTOR_STRIBECK_RAMP_DUTY_S * dt;
    if (want > s_stribeck_applied + max_step) {
        s_stribeck_applied += max_step;
    } else if (want < s_stribeck_applied - max_step) {
        s_stribeck_applied -= max_step;
    } else {
        s_stribeck_applied = want;
    }

    s_stribeck_last_plan = plan_speed;
    if (plan_sign != 0) {
        s_stribeck_last_plan_sign = plan_sign;
    }

    out = (float)duty + s_stribeck_applied;
    if (out > 3500.0f) {
        out = 3500.0f;
    } else if (out < -3500.0f) {
        out = -3500.0f;
    }
    return (int16_t)out;
}

static void motor_set_duty_from_speed_loop(motor_context_t *context)
{
    int16_t duty = (int16_t)context->motor_pid.motor_speed_pid.output;
    uint8_t pos_hold = 0U;

    if (context->control.mode == motor_control_mode_position_speed_torque
        && s_pos_stop_hold != 0U) {
        pos_hold = 1U;
    }

    duty = motor_apply_stribeck_ff(&context->motor_pid.motor_speed_pid,
        duty,
        context->control.plan_speed,
        context->sensor.motor_speed_degree,
        pos_hold);
    motor_set_duty(duty);
}

static void motor_position_on_hold_exit(motor_context_t *ctx)
{
    /* 退出精调进入粗调：继承当前速度，禁止 plan_speed 清零造成假启停 */
    angle_control_plan_reset(&ctx->control,
        ctx->sensor.motor_angle_multi_degree,
        ctx->control.plan_speed);
    if (motor_fabsf(ctx->control.plan_speed) < 1.0f) {
        ctx->control.plan_speed = ctx->sensor.motor_speed_degree;
    }
    ctx->motor_pid.motor_angle_pid.integral = 0.0f;
    ctx->motor_pid.motor_angle_pid.last_error = 0.0f;
    ctx->motor_pid.motor_angle_pid.output = 0.0f;
}

static void motor_position_stop_zone_update(float abs_err, float plan_rem)
{
    /* 规划已结束，或实际误差进 1°，或规划角已进 1° → 立刻精调 */
    if (s_angle_plan_finished != 0U
        || abs_err <= POS_PLAN_DEADZONE_DEG
        || (s_plan_target_valid != 0U && plan_rem <= POS_PLAN_DEADZONE_DEG)) {
        s_pos_stop_hold = 1U;
        return;
    }

    if (s_pos_stop_hold != 0U
        && abs_err > (POS_PLAN_DEADZONE_DEG + POS_STOP_HYSTERESIS_DEG)) {
        s_pos_stop_hold = 0U;
    }
}

static uint8_t motor_position_in_stop_zone(void)
{
    return s_pos_stop_hold;
}

void motor_init(void)
{
    s_motor_duty = 0;
    s_stribeck_applied = 0.0f;
    s_stribeck_boost_on = 0U;
    s_stribeck_boost_enter_tick = 0U;
    s_stribeck_last_plan = 0.0f;
    s_stribeck_last_plan_sign = 0;
    s_pos_stop_hold = 0U;
    s_pos_was_stop_hold = 0U;
    s_plan_in_decel = 0U;
    s_angle_plan_finished = 0U;
    motor_follow_init();
    pwm_init();
    motor_enable();
    pid_init(&motor_context.motor_pid.motor_speed_pid, 3.0f, 0.1f, 0.0f);
    pid_set_limits(&motor_context.motor_pid.motor_speed_pid, 720.0f, 3500.0f, 3500.0f);
    pid_init(&motor_context.motor_pid.motor_angle_pid, 5.0f, 0.0f, 0.05f);
    pid_set_limits(&motor_context.motor_pid.motor_angle_pid,
        0.0f, 0.0f, POS_TRIM_SPEED_MAX);
}

void motor_enable(void)
{
    gpio_bit_set(GPIOA, GPIO_PIN_6);
}

void motor_disable(void)
{
    s_motor_duty = 0;
    s_stribeck_applied = 0.0f;
    s_stribeck_boost_on = 0U;
    s_stribeck_boost_enter_tick = 0U;
    s_stribeck_last_plan = 0.0f;
    s_stribeck_last_plan_sign = 0;
    pwm_set_duty(0);
    gpio_bit_reset(GPIOA, GPIO_PIN_6);
}

void motor_set_duty(int16_t duty)
{
    s_motor_duty = duty;
    pwm_set_duty(duty);
}

int16_t motor_get_duty(void)
{
    return s_motor_duty;
}

void speed_control_plan_reset(motor_control_context_t *ctx, float current_speed)
{
    ctx->plan_speed = current_speed;
}

void angle_control_plan_reset(motor_control_context_t *ctx,
    float current_angle, float current_speed)
{
    ctx->plan_angle = current_angle;
    ctx->plan_start_angle = current_angle;
    ctx->plan_speed = current_speed;
    s_plan_target_valid = 0U;
    s_plan_in_decel = 0U;
    s_angle_plan_finished = 0U;
    s_pos_stop_hold = 0U; /* 新目标先进粗调 */
}

void motor_sync_target_to_actual(motor_context_t *ctx)
{
    ctx->control.target_angle = ctx->sensor.motor_angle_multi_degree;
    ctx->control.target_speed = 0.0f;
    angle_control_plan_reset(&ctx->control,
        ctx->sensor.motor_angle_multi_degree,
        ctx->sensor.motor_speed_degree);
    speed_control_plan_reset(&ctx->control, ctx->sensor.motor_speed_degree);
    ctx->motor_pid.motor_angle_pid.integral = 0.0f;
    ctx->motor_pid.motor_angle_pid.last_error = 0.0f;
    ctx->motor_pid.motor_angle_pid.error = 0.0f;
    ctx->motor_pid.motor_speed_pid.integral = 0.0f;
    ctx->motor_pid.motor_speed_pid.last_error = 0.0f;
    s_stribeck_applied = 0.0f;
    s_stribeck_boost_on = 0U;
    s_stribeck_boost_enter_tick = g_control_tick;
    s_stribeck_last_plan = 0.0f;
    s_stribeck_last_plan_sign = 0;
    s_pos_stop_hold = 1U; /* 已对齐目标，视为精调保持 */
    s_pos_was_stop_hold = 1U;
    motor_follow_reset();
    ctx->flag.angle_start_flag = TRUE;
}

void motor_position_home(void)
{
    uint32_t retry;

    motor_context.flag.angle_start_flag = FALSE;
    for (retry = 0U; retry < 500U; retry++) {
        if (encoder_update()) {
            speed_observer_reset(&g_speed_observer, &motor_context.sensor);
            speed_update(&g_speed_observer, &motor_context.sensor);
            motor_sync_target_to_actual(&motor_context);
            return;
        }
        delay_1ms(1U);
    }
}

void speed_control_plan(motor_control_context_t *ctx)
{
    const float dt = 1.0f / (float)SPEED_UPDATE_HZ;
    float v = ctx->plan_speed;
    float v_des = ctx->target_speed;
    float a = ctx->target_a_speed;
    float dv;
    float max_delta;

    if (a <= 0.0f) {
        ctx->plan_speed = v_des;
        return;
    }
    if ((v > 0.0f && v_des < 0.0f) || (v < 0.0f && v_des > 0.0f)) {
        if (v > 0.0f) {
            v -= a * dt;
            if (v < 0.0f) {
                v = 0.0f;
            }
        } else if (v < 0.0f) {
            v += a * dt;
            if (v > 0.0f) {
                v = 0.0f;
            }
        }
        ctx->plan_speed = v;
        return;
    }
    dv = v_des - v;
    max_delta = a * dt;
    if (dv > max_delta) {
        v += max_delta;
    } else if (dv < -max_delta) {
        v -= max_delta;
    } else {
        v = v_des;
    }
    ctx->plan_speed = v;
}

/*
 * 粗调区时间域梯形：积分 plan_angle，输出 plan_speed。
 * 带初速重规划；减速区带滞回，减轻边界抖振。
 */
void angle_control_plan(motor_control_context_t *ctx,
    float *motor_angle_multi_degree, float *current_speed_degree)
{
    const float dt = 1.0f / (float)POSITION_PID_UPDATE_HZ;
    float actual = *motor_angle_multi_degree;
    float v_max = ctx->target_speed;
    float a = ctx->target_a_speed;
    float da;
    float dist;
    float dir;
    float s_rem;
    float v;
    float v_along;
    float s_stop;
    float s_enter;
    float s_exit;

    if (a <= 0.0f || v_max <= 0.0f) {
        ctx->plan_speed = 0.0f;
        ctx->plan_angle = actual;
        return;
    }

    da = a * dt;

    if (s_plan_target_valid == 0U) {
        ctx->plan_angle = actual;
        if (motor_fabsf(ctx->plan_speed) < 1.0f) {
            ctx->plan_speed = *current_speed_degree;
        }
        ctx->plan_start_angle = ctx->plan_angle;
        s_plan_last_target = ctx->target_angle;
        s_plan_target_valid = 1U;
        s_plan_in_decel = 0U;
        s_angle_plan_finished = 0U;
    } else if (ctx->target_angle != s_plan_last_target) {
        ctx->plan_start_angle = ctx->plan_angle;
        s_plan_last_target = ctx->target_angle;
        s_plan_in_decel = 0U;
        s_angle_plan_finished = 0U;
    }

    dist = ctx->target_angle - ctx->plan_angle;
    dir = (dist >= 0.0f) ? 1.0f : -1.0f;
    s_rem = motor_fabsf(dist);

    if (s_rem < POS_PLAN_ARRIVE_DEG) {
        ctx->plan_angle = ctx->target_angle;
        ctx->plan_speed = 0.0f;
        s_plan_in_decel = 0U;
        s_angle_plan_finished = 1U; /* 规划到点 → 强制进精调 */
        return;
    }

    v = ctx->plan_speed;
    v_along = v * dir;

    if (v_along < 0.0f) {
        if (v > 0.0f) {
            v -= da;
            if (v < 0.0f) {
                v = 0.0f;
            }
        } else if (v < 0.0f) {
            v += da;
            if (v > 0.0f) {
                v = 0.0f;
            }
        }
        ctx->plan_speed = v;
        ctx->plan_angle += v * dt;
        return;
    }

    s_stop = (v_along * v_along) / (2.0f * a);
    s_enter = s_stop;
    s_exit = s_stop * POS_DECEL_HYSTERESIS;

    if (s_plan_in_decel != 0U) {
        if (s_rem > s_exit) {
            s_plan_in_decel = 0U;
        }
    } else if (s_rem <= s_enter) {
        s_plan_in_decel = 1U;
    }

    if (s_plan_in_decel != 0U) {
        if (v_along > da) {
            v_along -= da;
        } else {
            v_along = 0.0f;
        }
    } else if (v_along < v_max) {
        v_along += da;
        if (v_along > v_max) {
            v_along = v_max;
        }
    } else {
        v_along = v_max;
    }

    v = dir * v_along;
    ctx->plan_speed = v;
    ctx->plan_angle += v * dt;

    if (dir > 0.0f) {
        if (ctx->plan_angle > ctx->target_angle) {
            ctx->plan_angle = ctx->target_angle;
            /* 不瞬间清零：交给精调或下一拍减速 */
            if (motor_fabsf(ctx->plan_speed) < da) {
                ctx->plan_speed = 0.0f;
            }
        }
    } else if (ctx->plan_angle < ctx->target_angle) {
        ctx->plan_angle = ctx->target_angle;
        if (motor_fabsf(ctx->plan_speed) < da) {
            ctx->plan_speed = 0.0f;
        }
    }
}

void motor_control(motor_context_t *context)
{
    switch (context->control.mode) {
    case motor_control_mode_null:
        break;

    case motor_control_mode_torque:
        motor_set_duty((int16_t)context->control.target_duty);
        break;

    case motor_control_mode_speed_torque:
        if (context->flag.motor_speed_pid_flag) {
            speed_control_plan(&context->control);
            speed_pid(&context->motor_pid.motor_speed_pid,
                &context->control.plan_speed,
                &context->sensor.motor_speed_degree);
            motor_set_duty_from_speed_loop(context);
            context->flag.motor_speed_pid_flag = FALSE;
        }
        break;

    case motor_control_mode_position_speed_torque:
        /*
         * 粗调：未进 1° → 轨迹规划 → plan_speed
         * 精调：实际误差≤1° / 规划剩余≤1° / 规划到点 → 位置 PD
         * 跟随：高频 GOAL 流 → 仅粗调轨迹，target_speed 由差分推测
         * 内环：速度 PI 跟踪 plan_speed
         */
        motor_follow_poll();
        if (context->flag.motor_location_pid_flag) {
            float err = context->control.target_angle
                - context->sensor.motor_angle_multi_degree;
            float abs_err = motor_fabsf(err);
            uint8_t in_hold;

            if (motor_follow_is_active() != 0U) {
                in_hold = 0U;
                s_pos_stop_hold = 0U;
                s_pos_was_stop_hold = 0U;
                angle_control_plan(&context->control,
                    &context->sensor.motor_angle_multi_degree,
                    &context->sensor.motor_speed_degree);
                context->motor_pid.motor_angle_pid.target =
                    context->control.target_angle;
                context->motor_pid.motor_angle_pid.current =
                    context->sensor.motor_angle_multi_degree;
                context->motor_pid.motor_angle_pid.error = err;
                context->motor_pid.motor_angle_pid.output = 0.0f;
            } else {
                float plan_rem = motor_fabsf(context->control.target_angle
                    - context->control.plan_angle);

                motor_position_stop_zone_update(abs_err, plan_rem);
                in_hold = motor_position_in_stop_zone();

                if (s_pos_was_stop_hold && !in_hold) {
                    motor_position_on_hold_exit(context);
                }
                s_pos_was_stop_hold = in_hold;

                if (in_hold) {
                    location_pid(&context->motor_pid.motor_angle_pid,
                        &context->control,
                        &context->sensor);
                    context->control.plan_speed =
                        context->motor_pid.motor_angle_pid.output;
                } else {
                    angle_control_plan(&context->control,
                        &context->sensor.motor_angle_multi_degree,
                        &context->sensor.motor_speed_degree);

                    /* 本拍粗调后立刻再判：实际/规划进 1° 或规划到点 → 同拍精调 */
                    abs_err = motor_fabsf(context->control.target_angle
                        - context->sensor.motor_angle_multi_degree);
                    plan_rem = motor_fabsf(context->control.target_angle
                        - context->control.plan_angle);
                    motor_position_stop_zone_update(abs_err, plan_rem);

                    if (motor_position_in_stop_zone() != 0U
                        || s_angle_plan_finished != 0U) {
                        s_pos_stop_hold = 1U;
                        s_pos_was_stop_hold = 1U;
                        location_pid(&context->motor_pid.motor_angle_pid,
                            &context->control,
                            &context->sensor);
                        context->control.plan_speed =
                            context->motor_pid.motor_angle_pid.output;
                    } else {
                        context->motor_pid.motor_angle_pid.target =
                            context->control.target_angle;
                        context->motor_pid.motor_angle_pid.current =
                            context->sensor.motor_angle_multi_degree;
                        context->motor_pid.motor_angle_pid.error = err;
                        context->motor_pid.motor_angle_pid.output = 0.0f;
                    }
                }
            }

            context->flag.motor_location_pid_flag = FALSE;
        }
        if (context->flag.motor_speed_pid_flag) {
            speed_pid(&context->motor_pid.motor_speed_pid,
                &context->control.plan_speed,
                &context->sensor.motor_speed_degree);
            motor_set_duty_from_speed_loop(context);
            context->flag.motor_speed_pid_flag = FALSE;
        }
        break;
    }
}
