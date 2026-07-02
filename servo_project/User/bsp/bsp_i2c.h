#ifndef __BSP_I2C_H__

#define __BSP_I2C_H__



#include "gd32f1x0.h"
#include <stdint.h>



#define I2C_PERIPH              I2C0

#define I2C_PERIPH_RCU          RCU_I2C0

#define I2C_GPIO_PORT           GPIOA

#define I2C_GPIO_AF             GPIO_AF_4

#define I2C_SCL_PIN             GPIO_PIN_9

#define I2C_SDA_PIN             GPIO_PIN_10

#define I2C_SPEED_HZ            400000U



#define AS5600_I2C_ADDR_7BIT    0x36U

#define AS5600_REG_STATUS       0x0BU

#define AS5600_REG_ANGLE        0x0EU

#define AS5600_STATUS_MD        0x20U



/* 调试：bsp_i2c_read_reg 失败阶段，1=总线忙 2=START 3=写地址 4=写寄存器 5=重START 6=读地址 7=读数据 8=STOP */

extern volatile uint8_t g_i2c_last_fail_stage;



void bsp_i2c_init(void);

bool bsp_i2c_read_reg(uint8_t dev_addr_7bit, uint8_t reg, uint8_t *data, uint8_t len);



#endif


