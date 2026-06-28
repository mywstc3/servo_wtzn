#include "init.h"

#include "systick.h"



void init_all(void)

{
    systick_config();
    gpio_init();
    adc_init();
    DMA_init();
    motor_init();

}

