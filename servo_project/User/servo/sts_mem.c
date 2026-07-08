#include "sts_mem.h"
#include "sts_mem_map.h"
#include "servo_config.h"

#include <string.h>

extern volatile uint32_t g_control_tick;

#define STS_FEEDBACK_REFRESH_MS      50U

void motor_enable(void);
void motor_disable(void);
int16_t motor_get_duty(void);
void speed_control_plan_reset(motor_control_context_t *ctx, float current_speed);
void angle_control_plan_reset(motor_control_context_t *ctx, float current_angle);

#define STS_POS_UNIT_DEG             0.087f
#define STS_ACC_UNIT_DEGS2           8.7f
#define STS_VOLT_UNIT_V              0.1f
#define STS_CURR_UNIT_MA             6.5f

/* EPROM PID 寄存器为 uint8，经缩放对齐 motor_init 浮点参数 */
#define STS_PID_POS_D_SCALE          0.01f   /* POS_D: 5 -> Kd=0.05 */
#define STS_PID_SPEED_I_SCALE        0.1f    /* SPEED_I: 5 -> Ki=0.5 */

#define STS_ERR_INPUT_VOLTAGE_LOW    0x01U
#define STS_ERR_OVERCURRENT          0x02U
#define STS_ERR_OVER_TEMPERATURE     0x04U
#define STS_ERR_MAGNET_LOST          0x08U

#define STS_LOAD_DUTY_MAX            3599
#define STS_LOAD_DIR_BIT             0x0400U

static uint8_t s_mem[STS_MEM_SIZE];
static uint8_t s_last_error;
static uint8_t s_control_active;
static uint8_t s_magnet_ok = 1U;
static uint32_t s_feedback_last_tick;

static void put_u16_le(uint8_t addr, uint16_t value)
{
    s_mem[addr] = (uint8_t)(value & 0xFFU);
    s_mem[(uint8_t)(addr + 1U)] = (uint8_t)((value >> 8) & 0xFFU);
}

static uint16_t get_u16_le(uint8_t addr)
{
    return (uint16_t)s_mem[addr] | ((uint16_t)s_mem[(uint8_t)(addr + 1U)] << 8);
}

static float sts_absf(float v)
{
    return (v >= 0.0f) ? v : -v;
}

static int16_t sts_encode_load(int16_t duty)
{
    int32_t mag;
    int32_t ad;

    if (duty < 0) {
        ad = -(int32_t)duty;
    } else {
        ad = (int32_t)duty;
    }
    mag = ad * 1000 / STS_LOAD_DUTY_MAX;
    if (mag > 1023) {
        mag = 1023;
    }
    if (duty < 0) {
        mag |= (int32_t)STS_LOAD_DIR_BIT;
    }
    return (int16_t)mag;
}

static float pos_to_deg(int16_t pos)
{
    return (float)pos * STS_POS_UNIT_DEG;
}

static int16_t deg_to_pos(float deg)
{
    float q = deg / STS_POS_UNIT_DEG;
    if (q > 32767.0f) {
        q = 32767.0f;
    } else if (q < -32768.0f) {
        q = -32768.0f;
    }
    return (int16_t)q;
}

static float speed_raw_to_degs(int16_t raw)
{
    return (float)raw * STS_POS_UNIT_DEG;
}

static int16_t degs_to_speed_raw(float deg_per_sec)
{
    float q = deg_per_sec / STS_POS_UNIT_DEG;
    if (q > 32767.0f) {
        q = 32767.0f;
    } else if (q < -32768.0f) {
        q = -32768.0f;
    }
    return (int16_t)q;
}

static void sts_apply_pid_from_mem(void)
{
    motor_context.motor_pid.motor_angle_pid.kp = (float)s_mem[STS_ADDR_POS_P];
    motor_context.motor_pid.motor_angle_pid.kd =
        (float)s_mem[STS_ADDR_POS_D] * STS_PID_POS_D_SCALE;
    motor_context.motor_pid.motor_angle_pid.ki = (float)s_mem[STS_ADDR_POS_I];
    motor_context.motor_pid.motor_speed_pid.kp = (float)s_mem[STS_ADDR_SPEED_P];
    motor_context.motor_pid.motor_speed_pid.ki =
        (float)s_mem[STS_ADDR_SPEED_I] * STS_PID_SPEED_I_SCALE;
}

static uint8_t sts_addr_writable(uint8_t addr)
{
    if (addr < STS_ADDR_SERVO_ID) {
        return 0U;
    }
    if (addr >= STS_ADDR_PRESENT_POS_L && addr < STS_ADDR_FACTORY_BASE) {
        return 0U;
    }
    if (addr >= STS_ADDR_FACTORY_BASE && addr < STS_ADDR_FACTORY_END) {
        return 0U;
    }
    return 1U;
}

static uint8_t sts_range_hit(uint8_t start, uint8_t len, uint8_t reg, uint8_t reg_len)
{
    uint16_t a0 = start;
    uint16_t a1 = (uint16_t)start + (uint16_t)len;
    uint16_t b0 = reg;
    uint16_t b1 = (uint16_t)reg + (uint16_t)reg_len;
    return (uint8_t)((a0 < b1 && b0 < a1) ? 1U : 0U);
}

static void sts_on_write(uint8_t addr, uint8_t len)
{
    if (sts_range_hit(addr, len, STS_ADDR_TORQUE_SWITCH, 1U)) {
        uint8_t sw = s_mem[STS_ADDR_TORQUE_SWITCH];
        if (sw == 0U) {
            motor_disable();
        } else {
            motor_enable();
        }
        s_control_active = 1U;
    }

    if (sts_range_hit(addr, len, STS_ADDR_RUN_SPEED_L, 2U)) {
        int16_t raw = (int16_t)get_u16_le(STS_ADDR_RUN_SPEED_L);
        motor_context.control.mode = motor_control_mode_speed_torque;
        motor_context.control.target_speed = speed_raw_to_degs(raw);
        motor_context.control.target_a_speed = (float)s_mem[STS_ADDR_GOAL_ACC] * STS_ACC_UNIT_DEGS2;
        speed_control_plan_reset(&motor_context.control, motor_context.sensor.motor_speed_degree);
        s_control_active = 1U;
    }

    if (sts_range_hit(addr, len, STS_ADDR_GOAL_POS_L, 2U)) {
        int16_t raw = (int16_t)get_u16_le(STS_ADDR_GOAL_POS_L);
        motor_context.control.mode = motor_control_mode_position_speed_torque;
        motor_context.control.target_angle = pos_to_deg(raw);
        motor_context.control.target_speed =
            sts_absf(speed_raw_to_degs((int16_t)get_u16_le(STS_ADDR_RUN_SPEED_L)));
        if (motor_context.control.target_speed < 1.0f) {
            motor_context.control.target_speed = 180.0f;
        }
        motor_context.control.target_a_speed = (float)s_mem[STS_ADDR_GOAL_ACC] * STS_ACC_UNIT_DEGS2;
        if (motor_context.control.target_a_speed <= 0.0f) {
            motor_context.control.target_a_speed = 1000.0f;
        }
        angle_control_plan_reset(&motor_context.control, motor_context.sensor.motor_angle_multi_degree);
        s_control_active = 1U;
    }

    if (sts_range_hit(addr, len, STS_ADDR_PWM_OPEN_SPEED_L, 2U)) {
        int16_t raw = (int16_t)get_u16_le(STS_ADDR_PWM_OPEN_SPEED_L);
        /* 同步写位置时 0x2C 常为 0 占位，不应覆盖 GOAL_POS 触发的位置模式 */
        if (raw != 0) {
            float pct = ((float)raw) * 0.1f;
            if (pct > 100.0f) {
                pct = 100.0f;
            } else if (pct < -100.0f) {
                pct = -100.0f;
            }
            motor_context.control.mode = motor_control_mode_torque;
            motor_context.control.target_duty = pct * 10.0f;
            s_control_active = 1U;
        }
    }

    if (sts_range_hit(addr, len, STS_ADDR_POS_P, 3U)
        || sts_range_hit(addr, len, STS_ADDR_SPEED_P, 1U)
        || sts_range_hit(addr, len, STS_ADDR_SPEED_I, 1U)) {
        sts_apply_pid_from_mem();
    }
}

void sts_mem_init(void)
{
    memset(s_mem, 0, sizeof(s_mem));

    s_mem[STS_ADDR_FW_VER_MAJOR] = 3U;
    s_mem[STS_ADDR_FW_VER_MINOR] = 10U;
    s_mem[STS_ADDR_ENDIAN] = 0U;
    s_mem[STS_ADDR_MODEL_VER_MAJOR] = 9U;
    s_mem[STS_ADDR_MODEL_VER_MINOR] = 3U;

    s_mem[STS_ADDR_SERVO_ID] = 1U;
    s_mem[STS_ADDR_BAUD] = 0U;
    s_mem[STS_ADDR_RETURN_LEVEL] = 1U;
    put_u16_le(STS_ADDR_ANGLE_MIN_L, 0U);
    put_u16_le(STS_ADDR_ANGLE_MAX_L, 4095U);
    s_mem[STS_ADDR_TEMP_MAX] = 70U;
    s_mem[STS_ADDR_VOLT_MAX] = 140U;
    s_mem[STS_ADDR_VOLT_MIN] = 40U;
    put_u16_le(STS_ADDR_TORQUE_MAX_L, 1000U);
    s_mem[STS_ADDR_POS_P] = 5U;
    s_mem[STS_ADDR_POS_D] = 5U;
    s_mem[STS_ADDR_POS_I] = 0U;
    s_mem[STS_ADDR_SPEED_P] = 10U;
    s_mem[STS_ADDR_SPEED_I] = 5U;

    s_mem[STS_ADDR_TORQUE_SWITCH] = 1U;
    s_mem[STS_ADDR_GOAL_ACC] = 80U;
    put_u16_le(STS_ADDR_GOAL_POS_L, 2048U);
    put_u16_le(STS_ADDR_RUN_SPEED_L, 0U);
    put_u16_le(STS_ADDR_TORQUE_LIMIT_L, 1000U);
    s_mem[STS_ADDR_EPROM_LOCK] = 1U;

    s_last_error = 0U;
    s_control_active = 0U;
    s_magnet_ok = 1U;
    s_feedback_last_tick = 0U;
    sts_apply_pid_from_mem();
    sts_mem_refresh_feedback();
}

void sts_mem_poll(void)
{
    uint32_t now = g_control_tick;
    uint32_t elapsed = now - s_feedback_last_tick;

    if (elapsed >= STS_FEEDBACK_REFRESH_MS) {
        s_feedback_last_tick = now;
        sts_mem_refresh_feedback();
    }
}

void sts_mem_refresh_feedback(void)
{
    int16_t pos_raw;
    int16_t speed_raw;
    int16_t goal_raw;
    int16_t load_raw;
    uint8_t status = 0U;

    pos_raw = deg_to_pos(motor_context.sensor.motor_angle_multi_degree);
    /* PRESENT_SPEED 反馈为绝对值，方向由 RUN_SPEED 命令符号表示 */
    speed_raw = degs_to_speed_raw(sts_absf(motor_context.sensor.motor_speed_degree));
    goal_raw = deg_to_pos(motor_context.control.target_angle);
    load_raw = sts_encode_load(motor_get_duty());

    put_u16_le(STS_ADDR_PRESENT_POS_L, (uint16_t)pos_raw);
    put_u16_le(STS_ADDR_PRESENT_SPEED_L, (uint16_t)speed_raw);
    put_u16_le(STS_ADDR_GOAL_POS_FB_L, (uint16_t)goal_raw);
    put_u16_le(STS_ADDR_PRESENT_LOAD_L, (uint16_t)load_raw);

    s_mem[STS_ADDR_PRESENT_VOLT] =
        (uint8_t)(motor_context.sensor.motor_adc_v_bus / STS_VOLT_UNIT_V);
    put_u16_le(STS_ADDR_PRESENT_CUR_L,
        (uint16_t)(sts_absf(motor_context.sensor.motor_adc_i_bus * 1000.0f) / STS_CURR_UNIT_MA));

    s_mem[STS_ADDR_MOVING] =
        (uint8_t)(sts_absf(motor_context.control.target_angle
            - motor_context.sensor.motor_angle_multi_degree) > 1.0f);

    if (s_mem[STS_ADDR_PRESENT_VOLT] < s_mem[STS_ADDR_VOLT_MIN]) {
        status |= STS_ERR_INPUT_VOLTAGE_LOW;
    }
    if (s_mem[STS_ADDR_PRESENT_CUR_L] > s_mem[STS_ADDR_TORQUE_LIMIT_L]) {
        status |= STS_ERR_OVERCURRENT;
    }
    if (s_mem[STS_ADDR_PRESENT_TEMP] > s_mem[STS_ADDR_TEMP_MAX]) {
        status |= STS_ERR_OVER_TEMPERATURE;
    }
    if (s_magnet_ok == 0U) {
        status |= STS_ERR_MAGNET_LOST;
    }
    s_mem[STS_ADDR_STATUS] = status;
}

uint8_t sts_mem_read(uint8_t addr, uint8_t *out, uint8_t len)
{
    uint8_t i;

    s_last_error = 0U;
    if (out == NULL || len == 0U) {
        return 0U;
    }
    if ((uint16_t)addr + (uint16_t)len > STS_MEM_SIZE) {
        s_last_error = 1U;
        return 0U;
    }

    for (i = 0U; i < len; i++) {
        out[i] = s_mem[(uint8_t)(addr + i)];
    }
    return len;
}

uint8_t sts_mem_write(uint8_t addr, const uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint8_t wrote = 0U;

    s_last_error = 0U;
    if (data == NULL || len == 0U) {
        return 0U;
    }
    if ((uint16_t)addr + (uint16_t)len > STS_MEM_SIZE) {
        s_last_error = 1U;
        return 0U;
    }

    for (i = 0U; i < len; i++) {
        uint8_t a = (uint8_t)(addr + i);

        if (sts_addr_writable(a) == 0U) {
            continue;
        }
        if (a >= STS_ADDR_SERVO_ID && a <= STS_ADDR_SPEED_I
            && s_mem[STS_ADDR_EPROM_LOCK] != 0U
            && a != STS_ADDR_EPROM_LOCK) {
            continue;
        }
        s_mem[a] = data[i];
        wrote++;
    }

    if (wrote > 0U) {
        sts_on_write(addr, len);
    }
    return wrote;
}

uint8_t sts_mem_get_error(void)
{
    return s_last_error;
}

uint8_t sts_mem_get_servo_id(void)
{
    return s_mem[STS_ADDR_SERVO_ID];
}

uint8_t sts_mem_control_active(void)
{
    return s_control_active;
}

void sts_mem_set_magnet_ok(uint8_t ok)
{
    s_magnet_ok = (ok != 0U) ? 1U : 0U;
}
