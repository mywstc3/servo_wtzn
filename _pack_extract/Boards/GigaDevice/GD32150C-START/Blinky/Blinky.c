/*----------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2014 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <stdio.h>
//#include "core_cm3.h"
#include "gd32f1x0.h"                   // Device header
#include "Board_LED.h"                  // ::Board Support:LED
#include "Board_Buttons.h"              // ::Board Support:Buttons



volatile uint32_t msTicks;                      /* counts 1ms timeTicks       */
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;
}


/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 10 ms)
 *----------------------------------------------------------------------------*/
void Delay (uint32_t dlyTicks) {
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) { __NOP(); }
}


/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
  int32_t num  = -1;
  int32_t dir  =  1;
 uint32_t btns =  0;
 uint32_t ledNum = LED_GetCount();

  SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */

  LED_Initialize();
  Buttons_Initialize();

  SysTick_Config(SystemCoreClock / 100);        /* SysTick 10 msec interrupts */

  while(1) {                                    /* Loop forever               */
    btns = Buttons_GetState();                  /* Read button state          */

    if (btns != (1UL << 0)) {
      /* Calculate 'num': 0,1,...,LED_NUM-1,LED_NUM-1,...,1,0,0,...  */
      num += dir;
      if (num == ledNum) { dir = -1; num =  ledNum-1; }
      else if  (num < 0) { dir =  1; num =  0;         }

      LED_On (num);
      Delay(10);                                /* Delay 100ms                */
      LED_Off(num);
    }
    else {
      LED_SetOut ((1ul << ledNum) - 1);
    }

    Delay(40);                                  /* Delay 400ms                */

  }

}
