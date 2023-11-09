#ifndef __BSP_CLKCONFIG__
#define __BSP_CLKCONFIG__

#include "stm32f1xx_hal.h"

void HSE_SetSystemClock(void);
void HSI_SetSystemClock(void);
void Delay(__IO uint32_t nCount);

#endif /*__BSP_CLKCONFIG__*/
