#include "motor.h"
#include "time.h"
#include "motor_test.h"

motor_cmd_t motor_cmd = {
    .mode         = motor_cmd_position,
    .wave         = motor_wave_direct,
    .val_max      = 90.0f,
    .val_min      = -90.0f,
    .step         = 2.0f,
    .step_ms      = 50.0f,
    .setpoint     = 0.0f,
    .target       = 0.0f,
    .motion_speed = 100.0f,
    .motion_accel = 2000.0f,
};

static float s_tri_val = 0.0f;
static int8_t s_tri_dir = 1;
static float s_sine_phase = 0.0f;
static float s_pos_home = 0.0f;
static uint8_t s_pos_home_set = 0U;
static uint32_t s_last_poll_tick = 0U;

static motor_cmd_mode_t s_last_mode = (motor_cmd_mode_t)-1;
static motor_wave_type_t s_last_wave = (motor_wave_type_t)-1;

void motor_cmd_reset(void)
{
    s_tri_val = motor_cmd.val_min;
    s_tri_dir = 1;
    s_sine_phase = 0.0f;
    s_pos_home_set = 0U;
}

static float motor_test_sinf(float rad)
{
    const float pi = 3.14159265f;
    const float two_pi = 6.28318531f;
    float x2;

    while (rad > pi) {
        rad -= two_pi;
    }
    while (rad < -pi) {
        rad += two_pi;
    }

    x2 = rad * rad;
    return rad * (1.0f - x2 * (1.0f / 6.0f - x2 * (1.0f / 120.0f)));
}

static float motor_cmd_calc(motor_cmd_t *cmd)
{
    switch (cmd->wave)
    {
    case motor_wave_direct:
        return cmd->setpoint;

    case motor_wave_triangle:
        s_tri_val += (float)s_tri_dir * cmd->step;
        if (s_tri_val >= cmd->val_max)
        {
            s_tri_val = cmd->val_max;
            s_tri_dir = -1;
        }
        else if (s_tri_val <= cmd->val_min)
        {
            s_tri_val = cmd->val_min;
            s_tri_dir = 1;
        }
        return s_tri_val;

    case motor_wave_sine:
        s_sine_phase += cmd->step;
        {
            float norm = 0.5f + 0.5f * motor_test_sinf(s_sine_phase);
            return cmd->val_min + (cmd->val_max - cmd->val_min) * norm;
        }

    default:
        return cmd->setpoint;
    }
}

static void motor_cmd_apply(motor_cmd_t *cmd)
{
    motor_control_context_t *ctrl = &motor_context.control;

    switch (cmd->mode)
    {
    case motor_cmd_torque:
        ctrl->mode = motor_control_mode_torque;
        ctrl->target_duty = cmd->target;
        break;

    case motor_cmd_speed:
        ctrl->mode = motor_control_mode_speed_torque;
        ctrl->target_speed = cmd->target;
        ctrl->target_a_speed = cmd->motion_accel;
        break;

    case motor_cmd_position:
        ctrl->mode = motor_control_mode_position_speed_torque;
        ctrl->target_speed = cmd->motion_speed;
        ctrl->target_a_speed = cmd->motion_accel;
        if (!s_pos_home_set && motor_context.flag.angle_start_flag)
        {
            s_pos_home = motor_context.sensor.motor_angle_multi_degree;
            s_pos_home_set = 1U;
            angle_control_plan_reset(ctrl, s_pos_home);
        }
        ctrl->target_angle = cmd->target;
        break;

    default:
        break;
    }
}

void motor_test_poll(void)
{
    float period_ms = motor_cmd.step_ms;
    uint32_t now = g_control_tick;

    if (period_ms <= 0.0f)
    {
        period_ms = 1.0f;
    }

    if ((uint32_t)period_ms > (now - s_last_poll_tick))
    {
        return;
    }
    s_last_poll_tick = now;

    if (motor_cmd.mode != s_last_mode || motor_cmd.wave != s_last_wave)
    {
        motor_cmd_reset();
        if (motor_context.flag.angle_start_flag)
        {
            if (motor_cmd.mode == motor_cmd_speed)
            {
                speed_control_plan_reset(&motor_context.control,
                    motor_context.sensor.motor_speed_degree);
            }
            else if (motor_cmd.mode == motor_cmd_position)
            {
                angle_control_plan_reset(&motor_context.control,
                    motor_context.sensor.motor_angle_multi_degree);
            }
        }
        s_last_mode = motor_cmd.mode;
        s_last_wave = motor_cmd.wave;
    }

    motor_cmd.target = motor_cmd_calc(&motor_cmd);
    motor_cmd_apply(&motor_cmd);
}

void motor_cmd_init(void)
{
    motor_cmd_reset();
    s_last_mode = (motor_cmd_mode_t)-1;
    s_last_wave = (motor_wave_type_t)-1;
    s_last_poll_tick = g_control_tick;

    if (motor_cmd.mode == motor_cmd_position
        && motor_context.flag.angle_start_flag) {
        motor_cmd.setpoint = motor_context.sensor.motor_angle_multi_degree;
    }

    motor_cmd.target = motor_cmd_calc(&motor_cmd);
    motor_cmd_apply(&motor_cmd);
}
