#ifndef BSP_BASETIM
#define BSP_BASETIM

#include "stm32f1xx_hal.h"

#define         BASIC_TIM                                   TIM6
#define         BASIC_TIM_CLK_ENABLE()                      __HAL_RCC_TIM6_CLK_ENABLE()
#define         BASIC_TIM_IRQn                              TIM6_DAC_IRQn
#define         BASIC_TIM_IRQHandler                        TIM6_DAC_IRQHandler

extern TIM_HandleTypeDef TIM_Handle;

void TIM_Init(void);

#endif /* BSP_BASETIM */

