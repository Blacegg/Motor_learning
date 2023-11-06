#ifndef BSP_ADVANCE_TIM
#define BSP_ADVANCE_TIM

#include "stm32f1xx_hal.h"

/* 定时器*/
#define         ADVANCED_TIMx                               TIM1
#define         ADVANCED_TIM_RCC_CLK_ENABLE()               __HAL_RCC_TIM1_CLK_ENABLE()
#define         ADVANCED_TIM_RCC_CLK_DISABLE()              __HAL_RCC_TIM1_CLK_DISABLE()
#define         ADVANCED_TIM_GPIO_RCC_CLK_ENABLE() \
                                                            {\
                                                                __HAL_RCC_GPIOA_CLK_ENABLE();\
                                                                __HAL_RCC_GPIOB_CLK_ENABLE();\
                                                            }
/* TIM8 通道1 输出引脚*/
#define         ADVANCE_OCPWM_PIN                           GPIO_PIN_8
#define         ADVANCE_OCPWM_GPIO_PORT                     GPIOA

/* TIM8 通道1 互补输出引脚*/
#define         ADVANCE_OCNPWM_PIN                          GPIO_PIN_13
#define         ADVANCE_OCNPWM_GPIO_PORT                    GPIOB
                                                            
/* TIM8 刹车引脚*/
#define         ADVANCE_BRAKE_PIN                           GPIO_PIN_12
#define         ADVANCE_BRAKE_GPIO_PORT                     GPIOB 

#define         ADVANCE_TIM_PSC                             (7200-1)                                                            
#define         ADVANCE_TIM_Period                          (10000-1)
                                                           
void Advance_TIM_Init(void);                                                            
extern uint16_t  ADVANCE_TIM_Pulse; 
extern TIM_HandleTypeDef ADVANCE_TIM_Handle;                                                            
#endif /* BSP_ADVANCE_TIM */

