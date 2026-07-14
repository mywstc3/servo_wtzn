#include "gd32f1x0.h"
#include "servo/init.h"
#include "servo/time.h"
#include "servo/encoder.h"
#include "servo/electricity.h"
#include "servo/speed.h"
#include "servo/servo_config.h"
#include "servo/uart.h"
#include "servo/motor_test.h"
#include "servo/sts_mem.h"
#include "bsp/bsp_i2c.h"

/* 1=VOFA+ JustFloat 波形；与 FD/STS 同占 USART1，联调 STS 时请改 0 */
#define DEBUG_JUSTFLOAT  0

#if DEBUG_JUSTFLOAT
#include "servo/data_send.h"
#include "servo/motor.h"
#endif

int main(void)
{
    init_all();

    while (1) {
        uart_comm_poll();
        sts_mem_poll();

        if (motor_context.flag.motor_adc_flag) {
            motor_context.flag.motor_adc_flag = FALSE;
            electricity_update();
            uart_comm_poll();
        }
        if (motor_context.flag.motor_encoder_flag) {
            motor_context.flag.motor_encoder_flag = FALSE;
            encoder_update();
            uart_comm_poll();
            if (motor_context.flag.motor_speed_flag) {
                motor_context.flag.motor_speed_flag = FALSE;
                speed_update(&g_speed_observer, &motor_context.sensor);
            }
#if DEBUG_JUSTFLOAT
            if (sts_mem_control_active() == 0U && justfloat_dma_busy() == 0U) {
                JF_SEND(
                    motor_context.sensor.motor_angle_multi_degree,
                    motor_context.sensor.motor_speed_degree,
                    motor_context.control.plan_speed,
                    motor_context.control.target_angle,
                    motor_context.control.plan_angle,
                    (float)motor_get_duty()
                );
            }
#endif
        }
        if (sts_mem_control_active() == 0U) {
            motor_test_poll();
        }
        motor_control(&motor_context);
        uart_comm_poll();
    }
}
