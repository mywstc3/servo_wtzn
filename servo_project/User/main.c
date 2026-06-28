#include "gd32f1x0.h"
#include "systick.h"
#include "servo/init.h"
#include "servo/servo_config.h"
#include "servo/motor.h"


int main(void)

{

    init_all();
    motor_set_duty(500);
	
    while (1)
    {

    }

}

