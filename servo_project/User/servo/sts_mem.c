#include "sts_mem.h"
#include "sts_mem_map.h"
#include "sts_eeprom.h"
#include "servo_config.h"
#include "encoder.h"
#include "motor_follow.h"

#include <string.h>

extern volatile uint32_t g_control_tick;

void motor_sync_target_to_actual(motor_context_t *ctx);
void sts_mem_calibrate_midpoint(uint16_t target_raw);
void sts_mem_reset_offset(void);

#define STS_FEEDBACK_REFRESH_MS      50U

void motor_enable(void);
void motor_disable(void);
int16_t motor_get_duty(void);
void speed_control_plan_reset(motor_control_context_t *ctx, float current_speed);
void angle_control_plan_reset(motor_control_context_t *ctx,
    float current_angle, float current_speed);

#define STS_POS_UNIT_DEG             (360.0f / 4096.0f)  /* 与 encoder.c 一致，非协议近似 0.087 */
#define STS_ACC_UNIT_DEGS2           8.7f
#define STS_VOLT_UNIT_V              0.1f
#define STS_CURR_UNIT_MA             6.5f

/* EPROM PID 寄存器为 uint8，经缩放对齐 motor_init 浮点参数 */
#define STS_PID_POS_D_SCALE          0.01f   /* POS_D: 5 -> Kd=0.05 */
#define STS_PID_SPEED_I_SCALE        0.01f    /* SPEED_I: 10 -> Ki=0.1 */

#define STS_ERR_INPUT_VOLTAGE_LOW    0x01U
#define STS_ERR_OVERCURRENT          0x02U
#define STS_ERR_OVER_TEMPERATURE     0x04U
#define STS_ERR_MAGNET_LOST          0x08U

#define STS_LOAD_DUTY_MAX            3599
#define STS_LOAD_DIR_BIT             0x0400U
#define STS_SPEED_DIR_BIT            0x8000U /* STS 速度：bit15 方向，低 15 位幅值 */

static uint8_t s_mem[STS_MEM_SIZE];
static uint8_t s_last_error;
static uint8_t s_control_active;
static uint8_t s_magnet_ok = 1U;
static uint32_t s_feedback_last_tick;
static int16_t s_last_goal_sts = -1;
static float s_goal_multi_angle;
static uint8_t s_goal_track_valid;

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

static int16_t sts_get_pos_offset_raw(void)
{
    return (int16_t)get_u16_le(STS_ADDR_POS_OFFSET_L);
}

static uint16_t sts_raw_to_present_pos(uint16_t encoder_raw)
{
    int32_t pos = (int32_t)encoder_raw + (int32_t)sts_get_pos_offset_raw();

    while (pos < 0) {
        pos += 4096;
    }
    while (pos >= 4096) {
        pos -= 4096;
    }
    return (uint16_t)pos;
}

static int32_t sts_shortest_raw_delta(int32_t from_raw, int32_t to_raw)
{
    int32_t delta = to_raw - from_raw;

    while (delta > 2048) {
        delta -= 4096;
    }
    while (delta < -2048) {
        delta += 4096;
    }
    return delta;
}

static void sts_goal_track_reset(void)
{
    s_last_goal_sts = -1;
    s_goal_multi_angle = 0.0f;
    s_goal_track_valid = 0U;
}

static float sts_goal_to_target_angle(int16_t goal_raw)
{
    float cur = motor_context.sensor.motor_angle_multi_degree;
    int32_t delta;

    if (s_goal_track_valid == 0U) {
        uint16_t enc_raw = motor_context.sensor.motor_encoder_raw_u16;
        int32_t present = (int32_t)sts_raw_to_present_pos(enc_raw);

        delta = sts_shortest_raw_delta(present, (int32_t)goal_raw);
        s_goal_multi_angle = cur + (float)delta * STS_POS_UNIT_DEG;
        s_goal_track_valid = 1U;
    } else {
        /* 连续 GOAL：按 STS 寄存器差值累计多圈，避免 0→4090 被折成反向 6 格 */
        delta = (int32_t)goal_raw - (int32_t)s_last_goal_sts;
        s_goal_multi_angle += (float)delta * STS_POS_UNIT_DEG;
    }

    s_last_goal_sts = goal_raw;
    return s_goal_multi_angle;
}

static int16_t sts_deg_to_goal_raw(float deg)
{
    int32_t phys_raw = (int32_t)(deg / STS_POS_UNIT_DEG + 0.5f);
    int32_t goal_raw;

    while (phys_raw < 0) {
        phys_raw += 4096;
    }
    while (phys_raw >= 4096) {
        phys_raw -= 4096;
    }
    goal_raw = phys_raw + (int32_t)sts_get_pos_offset_raw();
    while (goal_raw < 0) {
        goal_raw += 4096;
    }
    while (goal_raw >= 4096) {
        goal_raw -= 4096;
    }
    return (int16_t)goal_raw;
}

static void sts_eprom_save_if_unlocked(void)
{
    if (s_mem[STS_ADDR_EPROM_LOCK] == 0U) {
        sts_eeprom_save(s_mem);
    }
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

/* STS 速度寄存器：sign-magnitude，非 int16 补码（位15=方向） */
static float speed_raw_to_degs(uint16_t raw)
{
    float mag = (float)(raw & 0x7FFFU) * STS_POS_UNIT_DEG;

    if ((raw & STS_SPEED_DIR_BIT) != 0U) {
        return -mag;
    }
    return mag;
}

static uint16_t degs_to_speed_raw(float deg_per_sec)
{
    float abs_deg = (deg_per_sec >= 0.0f) ? deg_per_sec : -deg_per_sec;
    float q = abs_deg / STS_POS_UNIT_DEG;
    uint16_t mag;

    if (q > 32767.0f) {
        q = 32767.0f;
    }
    mag = (uint16_t)q;
    if (deg_per_sec < 0.0f) {
        mag |= STS_SPEED_DIR_BIT;
    }
    return mag;
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

static void sts_mem_set_eprom_defaults(void)
{
    s_mem[STS_ADDR_SERVO_ID] = 1U;
    s_mem[STS_ADDR_BAUD] = 0U;
    s_mem[STS_ADDR_RETURN_DELAY] = STS_RETURN_DELAY_DEFAULT;
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
    put_u16_le(STS_ADDR_MIN_START_FORCE_L, 150U);
    s_mem[STS_ADDR_SPEED_P] = 3U;
    s_mem[STS_ADDR_SPEED_I] = 10U;
}

static uint8_t sts_mem_eprom_all_zero(void)
{
    uint8_t addr;

    for (addr = STS_ADDR_EPROM_BEGIN; addr <= STS_ADDR_EPROM_END; addr++) {
        if (s_mem[addr] != 0U) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t sts_write_touches_eprom(uint8_t addr, uint8_t len)
{
    uint8_t i;

    for (i = 0U; i < len; i++) {
        if (sts_eeprom_is_eprom_addr((uint8_t)(addr + i)) != 0U) {
            return 1U;
        }
    }
    return 0U;
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
        } else if (sw == STS_TORQUE_SW_CALIB) {
            sts_mem_calibrate_midpoint(STS_POS_MIDPOINT_RAW);
            s_mem[STS_ADDR_TORQUE_SWITCH] = 1U;
            motor_enable();
        } else {
            motor_enable();
        }
        s_control_active = 1U;
    }

    if (sts_range_hit(addr, len, STS_ADDR_RUN_SPEED_L, 2U)) {
        uint16_t raw = get_u16_le(STS_ADDR_RUN_SPEED_L);
        motor_context.control.mode = motor_control_mode_speed_torque;
        motor_context.control.target_speed = speed_raw_to_degs(raw);
        motor_context.control.target_a_speed = (float)s_mem[STS_ADDR_GOAL_ACC] * STS_ACC_UNIT_DEGS2;
        speed_control_plan_reset(&motor_context.control, motor_context.sensor.motor_speed_degree);
        s_control_active = 1U;
    }

    if (sts_range_hit(addr, len, STS_ADDR_GOAL_POS_L, 2U)) {
        int16_t raw = (int16_t)get_u16_le(STS_ADDR_GOAL_POS_L);
        float new_target;
        float inherit_speed;

        new_target = sts_goal_to_target_angle(raw);
        motor_context.control.mode = motor_control_mode_position_speed_torque;
        motor_context.control.target_a_speed = (float)s_mem[STS_ADDR_GOAL_ACC] * STS_ACC_UNIT_DEGS2;
        if (motor_context.control.target_a_speed <= 0.0f) {
            motor_context.control.target_a_speed = 1000.0f;
        }

        if (motor_follow_on_goal_pos(&motor_context.control,
                new_target,
                motor_context.sensor.motor_speed_degree) != 0U) {
            s_control_active = 1U;
        } else {
            motor_context.control.target_angle = new_target;
            motor_context.control.target_speed =
                sts_absf(speed_raw_to_degs(get_u16_le(STS_ADDR_RUN_SPEED_L)));
            if (motor_context.control.target_speed < 1.0f) {
                motor_context.control.target_speed = 90.0f;
            }
            inherit_speed = motor_context.control.plan_speed;
            if (sts_absf(inherit_speed) < 1.0f) {
                inherit_speed = motor_context.sensor.motor_speed_degree;
            }
            angle_control_plan_reset(&motor_context.control,
                motor_context.sensor.motor_angle_multi_degree,
                inherit_speed);
            s_control_active = 1U;
        }
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

    sts_eeprom_load(s_mem);
    if (sts_mem_eprom_all_zero() != 0U) {
        sts_mem_set_eprom_defaults();
        sts_eeprom_save(s_mem);
    } else if (s_mem[STS_ADDR_RETURN_DELAY] == 0U) {
        /* 旧固件未写过 0x07：运行期补默认（不强制落盘，便于上位机改写含写 0） */
        s_mem[STS_ADDR_RETURN_DELAY] = STS_RETURN_DELAY_DEFAULT;
    }

    s_mem[STS_ADDR_TORQUE_SWITCH] = 1U;
    s_mem[STS_ADDR_GOAL_ACC] = 100U;
    put_u16_le(STS_ADDR_GOAL_POS_L, 2048U);
    put_u16_le(STS_ADDR_RUN_SPEED_L, 0U);
    put_u16_le(STS_ADDR_TORQUE_LIMIT_L, 1000U);
    s_mem[STS_ADDR_EPROM_LOCK] = 1U;

    s_last_error = 0U;
    s_control_active = 0U;
    s_magnet_ok = 1U;
    s_feedback_last_tick = 0U;
    sts_goal_track_reset();
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
    uint16_t speed_raw;
    int16_t load_raw;
    uint8_t status = 0U;
    uint16_t enc_raw;

    enc_raw = motor_context.sensor.motor_encoder_raw_u16;
    pos_raw = (int16_t)sts_raw_to_present_pos(enc_raw);
    /* PRESENT_SPEED：幅值 + bit15 方向（飞特 sign-magnitude） */
    speed_raw = degs_to_speed_raw(motor_context.sensor.motor_speed_degree);
    /* PRESENT_LOAD：上报方向与内部 duty 取反，便于与上位机/速度同号约定对齐 */
    load_raw = sts_encode_load((int16_t)(-motor_get_duty()));

    put_u16_le(STS_ADDR_PRESENT_POS_L, (uint16_t)pos_raw);
    put_u16_le(STS_ADDR_PRESENT_SPEED_L, speed_raw);
    put_u16_le(STS_ADDR_GOAL_POS_FB_L,
        (uint16_t)sts_deg_to_goal_raw(motor_context.control.target_angle));
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
        if (s_mem[STS_ADDR_EPROM_LOCK] == 0U
            && sts_write_touches_eprom(addr, len) != 0U) {
            sts_eeprom_save(s_mem);
        }
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

uint8_t sts_mem_get_return_delay(void)
{
    return s_mem[STS_ADDR_RETURN_DELAY];
}

uint8_t sts_mem_control_active(void)
{
    return s_control_active;
}

uint16_t sts_mem_get_min_start_force(void)
{
    return get_u16_le(STS_ADDR_MIN_START_FORCE_L);
}

void sts_mem_set_magnet_ok(uint8_t ok)
{
    s_magnet_ok = (ok != 0U) ? 1U : 0U;
}

void sts_mem_calibrate_midpoint(uint16_t target_raw)
{
    uint16_t raw;
    int16_t offset;

    s_last_error = 0U;
    if (encoder_update() == 0U) {
        s_last_error = 1U;
        return;
    }
    raw = motor_context.sensor.motor_encoder_raw_u16;
    offset = (int16_t)((int16_t)target_raw - (int16_t)raw);
    put_u16_le(STS_ADDR_POS_OFFSET_L, (uint16_t)offset);
    sts_eprom_save_if_unlocked();
    sts_goal_track_reset();
    motor_sync_target_to_actual(&motor_context);
    sts_mem_refresh_feedback();
}

void sts_mem_reset_offset(void)
{
    put_u16_le(STS_ADDR_POS_OFFSET_L, 0U);
    sts_eprom_save_if_unlocked();
    sts_goal_track_reset();
    sts_mem_refresh_feedback();
}
