#include "encoder.h"
#include "bsp/bsp_i2c.h"
#include "servo_config.h"
#define AS5600_REG_STATUS_BASE    0x0BU   /* 连读 STATUS + RAW + ANGLE，一次 I2C 事务 */

void encoder_init(void)
{
    bsp_i2c_init();
}

uint8_t encoder_update(void)
{
    uint8_t buf[5];
    uint16_t raw;

    /* 0x0B STATUS, 0x0C-0x0D RAW, 0x0E-0x0F ANGLE */
    if (!bsp_i2c_read_reg(AS5600_I2C_ADDR_7BIT, AS5600_REG_STATUS_BASE, buf, 5U)) {
        return 0U;
    }
    if ((buf[0] & AS5600_STATUS_MD) == 0U) {
        return 0U;
    }

    raw = (uint16_t)(((uint16_t)buf[3] << 8) | buf[4]) & 0x0FFFU;
    motor_context.sensor.motor_encoder_raw = (float)raw;
    motor_context.sensor.motor_encoder_degree = (float)raw * 360.0f / 4096.0f;
    motor_context.sensor.motor_encoder_radian =
        motor_context.sensor.motor_encoder_degree * 3.14159265f / 180.0f;
    return 1U;
}
