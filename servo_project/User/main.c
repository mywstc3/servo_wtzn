#include "gd32f1x0.h"
#include "servo/init.h"
#include "servo/time.h"
#include "servo/encoder.h"
#include "servo/electricity.h"
#include "servo/speed.h"
#include "servo/servo_config.h"
#include "servo/uart.h"
#include "servo/data_send.h"
#include "servo/pid.h"
#include "servo/motor_test.h"
#include "bsp/bsp_i2c.h"

int main(void)
{
    init_all();

    while (1) {
        if (motor_context.flag.motor_adc_flag) {
            motor_context.flag.motor_adc_flag = FALSE;
            electricity_update();
        }
        if (motor_context.flag.motor_encoder_flag) {
            motor_context.flag.motor_encoder_flag = FALSE;
            encoder_update();
            if (motor_context.flag.motor_speed_flag) {
                motor_context.flag.motor_speed_flag = FALSE;
                speed_update(&g_speed_observer, &motor_context.sensor);
            }
        }
        uart_comm_poll();
        motor_test_poll();
        motor_control(&motor_context);
        if (!justfloat_dma_busy()) {
            JF_SEND(motor_context.sensor.motor_encoder_degree,
                    motor_context.sensor.motor_speed_degree,
                    motor_context.control.plan_speed,
                    motor_context.control.target_angle,
                    motor_context.sensor.motor_angle_multi_degree);
        }
    }
}
