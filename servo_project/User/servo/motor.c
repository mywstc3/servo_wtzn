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
#include <math.h>

static int16_t s_motor_duty;
static uint8_t s_pos_stop_hold = 0U;
static uint8_t s_pos_was_stop_hold = 0U;
static float s_plan_last_target = 0.0f;
static uint8_t s_plan_target_valid = 0U;

static void motor_position_on_hold_exit(motor_context_t *ctx)
{
    angle_control_plan_reset(&ctx->control, ctx->sensor.motor_angle_multi_degree);
    speed_control_plan_reset(&ctx->control, ctx->sensor.motor_speed_degree);
    ctx->motor_pid.motor_angle_pid.integral = 0.0f;
    ctx->motor_pid.motor_angle_pid.last_error = 0.0f;
    ctx->motor_pid.motor_angle_pid.output = 0.0f;
}

static void motor_position_stop_zone_update(float abs_err)
{
    if (s_pos_stop_hold == 0U) {
        if (abs_err <= POS_PLAN_DEADZONE_DEG) {
            s_pos_stop_hold = 1U;
        }
    } else if (abs_err > (POS_PLAN_DEADZONE_DEG + POS_STOP_HYSTERESIS_DEG)) {
        s_pos_stop_hold = 0U;
    }
}

static uint8_t motor_position_in_stop_zone(void)
{
    return s_pos_stop_hold;
}

static float motor_fabsf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float motor_sqrtf(float x)
{
    float guess;

    if (x <= 0.0f) {
        return 0.0f;
    }

    guess = x;
    guess = 0.5f * (guess + x / guess);
    guess = 0.5f * (guess + x / guess);
    return guess;
}

/* 参考 FOC speed_forword_stribe：按实际速度平滑衰减的摩擦前馈幅值 */
static float motor_stribeck_feedforward_mag(float actual_speed)
{
    uint16_t start_duty = sts_mem_get_min_start_force();
    float move_duty = (float)MOTOR_STRIBECK_MOVE_DUTY;
    float abs_v = motor_fabsf(actual_speed);
    float start_f = (float)start_duty;

    if (start_duty == 0U) {
        return 0.0f;
    }
    if (abs_v >= MOTOR_STRIBECK_SPEED_DEG) {
        return move_duty;
    }

    return move_duty + (start_f - move_duty)
        * expf(-abs_v / MOTOR_STRIBECK_SPEED_DEG);
}

static int16_t motor_apply_stribeck_ff(int16_t duty, float plan_speed, float actual_speed)
{
    float abs_plan = motor_fabsf(plan_speed);
    float abs_actual = motor_fabsf(actual_speed);
    float ff_mag;
    float plan_scale;
    int16_t ff_duty;

    if (abs_plan < MOTOR_STRIBECK_PLAN_THRESH) {
        return duty;
    }
    if (abs_actual > abs_plan * MOTOR_STRIBECK_TRACK_RATIO) {
        return duty;
    }

    ff_mag = motor_stribeck_feedforward_mag(actual_speed);
    if (ff_mag <= 0.0f) {
        return duty;
    }

    plan_scale = abs_plan / MOTOR_STRIBECK_PLAN_FULL_DEG;
    if (plan_scale > 1.0f) {
        plan_scale = 1.0f;
    }
    ff_mag *= plan_scale;

    ff_duty = (int16_t)((plan_speed >= 0.0f) ? ff_mag : -ff_mag);

    /* 仅填补 PID 不足，避免 duty_pid + ff 叠加过冲 */
    if (duty == 0) {
        return ff_duty;
    }
    if (duty > 0 && ff_duty > 0 && duty < ff_duty) {
        return ff_duty;
    }
    if (duty < 0 && ff_duty < 0 && duty > ff_duty) {
        return ff_duty;
    }
    return duty;
}

static void motor_set_duty_from_speed_loop(motor_context_t *context)
{
    int16_t duty = (int16_t)context->motor_pid.motor_speed_pid.output;

    duty = motor_apply_stribeck_ff(duty,
        context->control.plan_speed,
        context->sensor.motor_speed_degree);
    motor_set_duty(duty);
}

void motor_init(void)
{
    s_motor_duty = 0;
    pwm_init();
    motor_enable();
    pid_init(&motor_context.motor_pid.motor_speed_pid, 10.0f, 0.5f, 0.0f);
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

void angle_control_plan_reset(motor_control_context_t *ctx, float current_angle)
{
    ctx->plan_angle = current_angle;
    ctx->plan_start_angle = current_angle;
    ctx->plan_speed = 0.0f;
    s_pos_stop_hold = 1U;
    s_plan_target_valid = 0U;
}

void motor_sync_target_to_actual(motor_context_t *ctx)
{
    ctx->control.target_angle = ctx->sensor.motor_angle_multi_degree;
    ctx->control.target_speed = 0.0f;
    angle_control_plan_reset(&ctx->control, ctx->sensor.motor_angle_multi_degree);
    speed_control_plan_reset(&ctx->control, ctx->sensor.motor_speed_degree);
    ctx->motor_pid.motor_angle_pid.integral = 0.0f;
    ctx->motor_pid.motor_angle_pid.last_error = 0.0f;
    ctx->motor_pid.motor_angle_pid.error = 0.0f;
    ctx->motor_pid.motor_speed_pid.integral = 0.0f;
    ctx->motor_pid.motor_speed_pid.last_error = 0.0f;
    s_pos_stop_hold = 1U;
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
    if (a <= 0.0f) {
        ctx->plan_speed = v_des;
        return;
    }
    /* 目标反向：先减到 0，再反向加（与位置规划一致，更安全） */
    if ((v > 0.0f && v_des < 0.0f) || (v < 0.0f && v_des > 0.0f)) {
        if (v > 0.0f) {
            v -= a * dt;
            if (v < 0.0f) v = 0.0f;
        } else if (v < 0.0f) {
            v += a * dt;
            if (v > 0.0f) v = 0.0f;
        }
        ctx->plan_speed = v;
        return;
    }
    float dv = v_des - v;
    float max_delta = a * dt;
    if (dv > max_delta)
        v += max_delta;
    else if (dv < -max_delta)
        v -= max_delta;
    else
        v = v_des;
    ctx->plan_speed = v;
}


void angle_control_plan(motor_control_context_t *ctx, float *motor_angle_multi_degree, float *current_speed_degree)
{
    const float dt = 1.0f / (float)POSITION_PID_UPDATE_HZ;
    float v_max = ctx->target_speed;
    float a = ctx->target_a_speed;
    float dist;
    float dir;
    float s_rem;
    float s_total;
    float d_traveled;
    float s_dec;
    float s_min_trap;
    float v_peak;
    float s_half;
    float v;
    float v_along;
    float da;
    uint8_t do_accel;
    uint8_t do_decel;

    (void)motor_angle_multi_degree;
    (void)current_speed_degree;

    if (a <= 0.0f || v_max <= 0.0f) {
        ctx->plan_speed = 0.0f;
        return;
    }

    if (s_plan_target_valid == 0U
        || ctx->target_angle != s_plan_last_target) {
        ctx->plan_start_angle = ctx->plan_angle;
        s_plan_last_target = ctx->target_angle;
        s_plan_target_valid = 1U;
    }

    dist = ctx->target_angle - ctx->plan_angle;
    dir = (dist >= 0.0f) ? 1.0f : -1.0f;
    s_rem = motor_fabsf(dist);
    s_total = motor_fabsf(ctx->target_angle - ctx->plan_start_angle);
    if (s_total < s_rem) {
        s_total = s_rem;
    }

    if (s_rem < POS_PLAN_DEADZONE_DEG) {
        ctx->plan_speed = 0.0f;
        ctx->plan_angle = ctx->target_angle;
        return;
    }

    d_traveled = s_total - s_rem;
    s_dec = (v_max * v_max) / (2.0f * a);
    s_min_trap = (v_max * v_max) / a;
    da = a * dt;

    if (s_total >= s_min_trap) {
        v_peak = v_max;
        if (s_rem <= s_dec) {
            do_decel = 1U;
            do_accel = 0U;
        } else {
            do_decel = 0U;
            v_along = ctx->plan_speed * dir;
            if (v_along < 0.0f) {
                v_along = 0.0f;
            }
            do_accel = (v_along < (v_peak - da)) ? 1U : 0U;
        }
    } else {
        v_peak = motor_sqrtf(a * s_total);
        if (v_peak > v_max) {
            v_peak = v_max;
        }
        s_half = s_total * 0.5f;
        if (d_traveled < s_half) {
            do_accel = 1U;
            do_decel = 0U;
        } else {
            do_accel = 0U;
            do_decel = 1U;
        }
    }

    v = ctx->plan_speed;
    v_along = v * dir;
    if (v_along < 0.0f) {
        v_along = 0.0f;
    }

    if (do_decel != 0U) {
        if (v_along > da) {
            v_along -= da;
        } else {
            v_along = 0.0f;
        }
    } else if (do_accel != 0U) {
        v_along += da;
        if (v_along > v_peak) {
            v_along = v_peak;
        }
    } else {
        v_along = v_peak;
    }

    v = dir * v_along;
    ctx->plan_speed = v;
    ctx->plan_angle += v * dt;

    if (dir > 0.0f) {
        if (ctx->plan_angle > ctx->target_angle) {
            ctx->plan_angle = ctx->target_angle;
        }
    } else if (ctx->plan_angle < ctx->target_angle) {
        ctx->plan_angle = ctx->target_angle;
    }
}

void motor_control(motor_context_t *context)
{
    switch(context->control.mode)
    {
        case motor_control_mode_null:
            break;
        case motor_control_mode_torque:
            motor_set_duty((int16_t)context->control.target_duty);
            break;

        case motor_control_mode_speed_torque:
            if(context->flag.motor_speed_pid_flag){
                speed_control_plan(&context->control);
                speed_pid(&context->motor_pid.motor_speed_pid, &context->control.plan_speed
                    , &context->sensor.motor_speed_degree);
                    motor_set_duty_from_speed_loop(context);
                    context->flag.motor_speed_pid_flag = FALSE;
                }
            break;

        case motor_control_mode_position_speed_torque:
            if (context->flag.motor_location_pid_flag) {
                float err = context->control.target_angle
                    - context->sensor.motor_angle_multi_degree;
                float abs_err = motor_fabsf(err);
                uint8_t in_hold;

                motor_position_stop_zone_update(abs_err);
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
                    context->motor_pid.motor_angle_pid.target =
                        context->control.target_angle;
                    context->motor_pid.motor_angle_pid.current =
                        context->sensor.motor_angle_multi_degree;
                    context->motor_pid.motor_angle_pid.error = err;
                    context->motor_pid.motor_angle_pid.output = 0.0f;
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