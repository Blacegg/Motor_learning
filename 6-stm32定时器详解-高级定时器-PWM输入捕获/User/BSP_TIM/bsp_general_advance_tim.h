#ifndef BSP_GENERAL_ADVANCE_TIM
#define BSP_GENERAL_ADVANCE_TIM

#include "stm32f1xx_hal.h"

#define         General_TIM                                     TIM3
#define         General_TIM_CLK_ENABLE                          __HAL_RCC_TIM3_CLK_ENABLE
#define         General_TIM_GPIOx_CLK_ENABLE                    __HAL_RCC_GPIOA_CLK_ENABLE
#define         General_TIM_PORT                                GPIOA
#define         General_TIM_GPIO_PIN                            GPIO_PIN_6
#define         General_TIM_Period                              (1000-1)
#define         General_TIM_PSC                                 (72-1)
#define         General_TIM_Pulse                               500

#define         Advance_TIM                                     TIM1
#define         Advance_TIM_RCC_CLK_ENABLE()                    __HAL_RCC_TIM1_CLK_ENABLE()
#define         Advance_TIM_GPIO_RCC_CLK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()
#define         Advance_TIM_GPIO_PIN                            GPIO_PIN_8
#define         Advance_TIM_PORT                                GPIOA
#define         Advance_TIM_Period                              0xFFFF
#define         Advance_TIM_PSC                                 (72-1)

void General_Advance_TIM_Init(void);

extern TIM_HandleTypeDef General_TIM_Handle;
extern TIM_HandleTypeDef Advance_TIM_Handle;

#endif /* BSP_GENERAL_ADVANCE_TIM */

