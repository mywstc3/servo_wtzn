#include "electricity.h"
#include "bsp/bsp_dma.h"
#include "servo_config.h"
#include "motor.h"

void electricity_update(void)
{
    uint16_t i_raw;
    uint16_t v_raw;
    float v_pa3;
    float i_mag;

    __disable_irq();
    i_raw = adc_dma_buf[0];
    v_raw = adc_dma_buf[1];
    __enable_irq();

    v_pa3 = (float)i_raw * ADC_VREF_V / ADC_FULL_SCALE;
    motor_context.sensor.motor_adc_i_raw = v_pa3;
    motor_context.sensor.motor_adc_v_raw =
        (float)v_raw * ADC_VREF_V / ADC_FULL_SCALE;

    i_mag = (v_pa3 - motor_context.sensor.motor_adc_i_bus_offset)
            / (ADC_CURRENT_GAIN * ADC_CURRENT_R_SHUNT_OHM);
    if (i_mag < 0.0f) {
        i_mag = 0.0f;
    }

    {
        int16_t duty = motor_get_duty();
        if (duty > 0) {
            motor_context.sensor.motor_adc_i_bus = i_mag;
        } else if (duty < 0) {
            motor_context.sensor.motor_adc_i_bus = -i_mag;
        } else {
            motor_context.sensor.motor_adc_i_bus = 0.0f;
        }
    }

    motor_context.sensor.motor_adc_v_bus =
        motor_context.sensor.motor_adc_v_raw * 11.5f / 1.5f;
}
