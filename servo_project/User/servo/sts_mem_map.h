#ifndef __STS_MEM_MAP_H__
#define __STS_MEM_MAP_H__

#define STS_MEM_SIZE                 0x80U

/* 版本只读区 */
#define STS_ADDR_FW_VER_MAJOR        0x00U
#define STS_ADDR_FW_VER_MINOR        0x01U
#define STS_ADDR_ENDIAN              0x02U
#define STS_ADDR_MODEL_VER_MAJOR     0x03U
#define STS_ADDR_MODEL_VER_MINOR     0x04U

/* EPROM 配置区 */
#define STS_ADDR_SERVO_ID            0x05U
#define STS_ADDR_BAUD                0x06U
#define STS_ADDR_RETURN_DELAY        0x07U
#define STS_ADDR_RETURN_LEVEL        0x08U
#define STS_ADDR_ANGLE_MIN_L         0x09U
#define STS_ADDR_ANGLE_MAX_L         0x0BU
#define STS_ADDR_TEMP_MAX            0x0DU
#define STS_ADDR_VOLT_MAX            0x0EU
#define STS_ADDR_VOLT_MIN            0x0FU
#define STS_ADDR_TORQUE_MAX_L        0x10U
#define STS_ADDR_PHASE               0x12U
#define STS_ADDR_UNLOAD_COND         0x13U
#define STS_ADDR_LED_COND            0x14U
#define STS_ADDR_POS_P               0x15U
#define STS_ADDR_POS_D               0x16U
#define STS_ADDR_POS_I               0x17U
#define STS_ADDR_SPEED_P             0x25U
#define STS_ADDR_SPEED_I             0x27U

/* SRAM 控制区 */
#define STS_ADDR_TORQUE_SWITCH       0x28U
#define STS_ADDR_GOAL_ACC            0x29U
#define STS_ADDR_GOAL_POS_L          0x2AU
#define STS_ADDR_PWM_OPEN_SPEED_L    0x2CU
#define STS_ADDR_RUN_SPEED_L         0x2EU
#define STS_ADDR_TORQUE_LIMIT_L      0x30U
#define STS_ADDR_EPROM_LOCK          0x37U

/* SRAM 反馈区 */
#define STS_ADDR_PRESENT_POS_L       0x38U
#define STS_ADDR_PRESENT_SPEED_L     0x3AU
#define STS_ADDR_PRESENT_LOAD_L      0x3CU
#define STS_ADDR_PRESENT_VOLT        0x3EU
#define STS_ADDR_PRESENT_TEMP        0x3FU
#define STS_ADDR_STATUS              0x41U
#define STS_ADDR_MOVING              0x42U
#define STS_ADDR_GOAL_POS_FB_L       0x43U
#define STS_ADDR_PRESENT_CUR_L       0x45U

#define STS_ADDR_FACTORY_BASE        0x50U
#define STS_ADDR_FACTORY_END         0x57U

#define STS_ID_BROADCAST             0xFEU

#endif
