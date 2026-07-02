#ifndef ELECTRICITY_H
#define ELECTRICITY_H

#include "gd32f1x0.h"

#define ADC_VREF_V              3.3f
#define ADC_FULL_SCALE          4096.0f
#define ADC_CURRENT_R_SHUNT_OHM 0.01f
#define ADC_CURRENT_GAIN        50.0f

void electricity_update(void);

#endif
