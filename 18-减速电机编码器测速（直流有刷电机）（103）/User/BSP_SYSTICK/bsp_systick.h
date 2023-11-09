#ifndef __BSP_SYSTICK__
#define __BSP_SYSTICK__

#include "stm32f1xx_hal.h"

#define     ITORNO      1

void Systick_Delay_us(uint32_t count);
void Systick_Delay_ms(uint32_t count);

#endif /*__BSP_SYSTICK__*/
