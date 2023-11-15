/***********************************************************************
 * @file bsp_basic_tim.h
 * @brief
 * @author blacegg
 * @version 1.0
 * @date 2023-08-21
 ***********************************************************************/

#ifndef __BSP_BASIC_TIM__
#define __BSP_BASIC_TIM__

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "pid.h"

#define BASIC_TIM TIM6
#define BASIC_TIM_CLK_ENABLE() __HAL_RCC_TIM6_CLK_ENABLE()

#define BASIC_TIM_IRQn TIM6_IRQn
#define BASIC_TIM_IRQHandler TIM6_IRQHandler

#define BASIC_PERIOD_COUNT (50 * 50)
#define BASIC_PRESCALER_COUNT (1440)

#define SET_BASIC_TIM_PERIOD(T) __HAL_TIM_SET_AUTORELOAD(&TIM_TimeBaseStructure, (T)*50 - 1)   // 设置定时器的周期（1~1000ms）
#define GET_BASIC_TIM_PERIOD() ((__HAL_TIM_GET_AUTORELOAD(&TIM_TimeBaseStructure) + 1) / 50.0) // 获取定时器的周期，单位ms

extern TIM_HandleTypeDef TIM_TimeBaseStructure;
void TIMx_Configuration(void);

#endif /*__BSP_BASIC_TIM__*/
