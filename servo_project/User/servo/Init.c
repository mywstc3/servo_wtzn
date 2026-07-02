#include "init.h"
#include "systick.h"
#include "encoder.h"
#include "time.h"
#include "uart.h"
#include "motor_test.h"

void init_all(void)
{
    systick_config();
    gpio_init();
    adc_init();
    motor_init();
    encoder_init();
    motor_position_home();
    time_init();
    uart_comm_init();
    motor_cmd_init();
}
