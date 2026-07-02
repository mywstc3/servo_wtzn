#ifndef __TIME_H__
#define __TIME_H__

#include "gd32f1x0.h"
#include <stdint.h>

#define CONTROL_TIMER           TIMER13
#define CONTROL_TIMER_RCU       RCU_TIMER13
#define CONTROL_TIMER_IRQn      TIMER13_IRQn
#define CONTROL_LOOP_HZ         1000U
#define ELECTRICITY_UPDATE_HZ   1000U
#define SPEED_UPDATE_HZ         1000U
#define SPEED_PID_UPDATE_HZ     500U
#define POSITION_PID_UPDATE_HZ   250U

void time_init(void);

extern volatile uint32_t g_control_tick;


#endif
