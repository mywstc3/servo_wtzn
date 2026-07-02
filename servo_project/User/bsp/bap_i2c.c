#include "bsp/bsp_i2c.h"

#include <stddef.h>

#include <stdint.h>



#define I2C_TIMEOUT_LOOPS    100000U



volatile uint8_t g_i2c_last_fail_stage;



static bool i2c_wait_flag_set(i2c_flag_enum flag)

{

    uint32_t timeout = I2C_TIMEOUT_LOOPS;



    while (RESET == i2c_flag_get(I2C_PERIPH, flag)) {

        if (timeout-- == 0U) {

            return FALSE;

        }

    }

    return TRUE;

}



static bool i2c_wait_bus_idle(void)

{

    uint32_t timeout = I2C_TIMEOUT_LOOPS;



    while (SET == i2c_flag_get(I2C_PERIPH, I2C_FLAG_I2CBSY)) {

        if (timeout-- == 0U) {

            return FALSE;

        }

    }

    return TRUE;

}



static bool i2c_wait_stop_sent(void)

{

    uint32_t timeout = I2C_TIMEOUT_LOOPS;



    while (0U != (I2C_CTL0(I2C_PERIPH) & I2C_CTL0_STOP)) {

        if (timeout-- == 0U) {

            return FALSE;

        }

    }

    return TRUE;

}



static void i2c_clear_error_flags(void)

{

    i2c_flag_clear(I2C_PERIPH, I2C_FLAG_AERR);

    i2c_flag_clear(I2C_PERIPH, I2C_FLAG_BERR);

    i2c_flag_clear(I2C_PERIPH, I2C_FLAG_LOSTARB);

    i2c_flag_clear(I2C_PERIPH, I2C_FLAG_OUERR);

}



static void i2c_gpio_bus_recover(void)

{

    uint8_t i;



    i2c_disable(I2C_PERIPH);

    gpio_mode_set(I2C_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, I2C_SCL_PIN | I2C_SDA_PIN);

    gpio_output_options_set(I2C_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SCL_PIN | I2C_SDA_PIN);

    gpio_bit_set(I2C_GPIO_PORT, I2C_SCL_PIN | I2C_SDA_PIN);



    for (i = 0U; i < 9U; i++) {

        gpio_bit_reset(I2C_GPIO_PORT, I2C_SCL_PIN);

        for (volatile uint32_t d = 0U; d < 200U; d++) { }

        gpio_bit_set(I2C_GPIO_PORT, I2C_SCL_PIN);

        for (volatile uint32_t d = 0U; d < 200U; d++) { }

    }



    gpio_af_set(I2C_GPIO_PORT, I2C_GPIO_AF, I2C_SCL_PIN | I2C_SDA_PIN);

    gpio_mode_set(I2C_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_SCL_PIN | I2C_SDA_PIN);

    gpio_output_options_set(I2C_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SCL_PIN | I2C_SDA_PIN);

    i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);

    i2c_enable(I2C_PERIPH);

    i2c_clear_error_flags();

}



static void i2c_bus_recover(void)

{

    i2c_stop_on_bus(I2C_PERIPH);

    (void)i2c_wait_stop_sent();

    i2c_clear_error_flags();

    if (!i2c_wait_bus_idle()) {

        i2c_gpio_bus_recover();

    }

}



static bool i2c_master_addr_ok(void)

{

    if (!i2c_wait_flag_set(I2C_FLAG_ADDSEND)) {

        return FALSE;

    }

    if (SET == i2c_flag_get(I2C_PERIPH, I2C_FLAG_AERR)) {

        i2c_flag_clear(I2C_PERIPH, I2C_FLAG_ADDSEND);

        i2c_clear_error_flags();

        return FALSE;

    }

    i2c_flag_clear(I2C_PERIPH, I2C_FLAG_ADDSEND);

    return TRUE;

}



void bsp_i2c_init(void)

{

    g_i2c_last_fail_stage = 0U;



    rcu_periph_clock_enable(RCU_GPIOA);

    rcu_periph_clock_enable(I2C_PERIPH_RCU);



    gpio_af_set(I2C_GPIO_PORT, I2C_GPIO_AF, I2C_SCL_PIN | I2C_SDA_PIN);

    gpio_mode_set(I2C_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_SCL_PIN | I2C_SDA_PIN);

    gpio_output_options_set(I2C_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SCL_PIN | I2C_SDA_PIN);



    i2c_deinit(I2C_PERIPH);

    i2c_clock_config(I2C_PERIPH, I2C_SPEED_HZ, I2C_DTCY_2);

    i2c_mode_addr_config(I2C_PERIPH, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0x00U);

    i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);

    i2c_enable(I2C_PERIPH);

    i2c_clear_error_flags();

}



bool bsp_i2c_read_reg(uint8_t dev_addr_7bit, uint8_t reg, uint8_t *data, uint8_t len)

{

    uint8_t addr_w = (uint8_t)(dev_addr_7bit << 1);

    uint8_t addr_r = addr_w;

    uint8_t i;

    bool ok = FALSE;



    if ((data == NULL) || (len == 0U)) {

        g_i2c_last_fail_stage = 0U;

        return FALSE;

    }



    /* I2C 不可重入：禁止 TIMER13 等中断在传输中途抢占 */

    __disable_irq();

    i2c_clear_error_flags();



    if (!i2c_wait_bus_idle()) {

        g_i2c_last_fail_stage = 1U;

        goto exit;

    }



    i2c_start_on_bus(I2C_PERIPH);

    if (!i2c_wait_flag_set(I2C_FLAG_SBSEND)) {

        g_i2c_last_fail_stage = 2U;

        goto exit;

    }



    i2c_master_addressing(I2C_PERIPH, addr_w, I2C_TRANSMITTER);

    if (!i2c_master_addr_ok()) {

        g_i2c_last_fail_stage = 3U;

        goto exit;

    }



    i2c_data_transmit(I2C_PERIPH, reg);

    if (!i2c_wait_flag_set(I2C_FLAG_BTC)) {

        g_i2c_last_fail_stage = 4U;

        goto exit;

    }



    i2c_start_on_bus(I2C_PERIPH);

    if (!i2c_wait_flag_set(I2C_FLAG_SBSEND)) {

        g_i2c_last_fail_stage = 5U;

        goto exit;

    }



    i2c_master_addressing(I2C_PERIPH, addr_r, I2C_RECEIVER);

    if (!i2c_master_addr_ok()) {

        g_i2c_last_fail_stage = 6U;

        goto exit;

    }



    for (i = 0U; i < len; i++) {

        if (i == (len - 1U)) {

            i2c_ack_config(I2C_PERIPH, I2C_ACK_DISABLE);

        }

        if (!i2c_wait_flag_set(I2C_FLAG_RBNE)) {

            i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);

            g_i2c_last_fail_stage = 7U;

            goto exit;

        }

        data[i] = i2c_data_receive(I2C_PERIPH);

    }



    i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);

    i2c_stop_on_bus(I2C_PERIPH);

    if (!i2c_wait_stop_sent()) {

        g_i2c_last_fail_stage = 8U;

        goto exit;

    }

    if (!i2c_wait_bus_idle()) {

        g_i2c_last_fail_stage = 1U;

        goto exit;

    }



    g_i2c_last_fail_stage = 0U;

    ok = TRUE;



exit:

    if (!ok) {

        i2c_bus_recover();

    }

    __enable_irq();

    return ok;

}


