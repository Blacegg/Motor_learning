#ifndef __BSP_MOTOR_TIM__
#define __BSP_MOTOR_TIM__

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

/*宏定义*/
#define PWM_TIM TIM8
// #define PWM_TIM_GPIO_AF_ENALBE() __HAL_AFIO_REMAP_TIM1_PARTIAL();
#define PWM_TIM_CLK_ENABLE() __HAL_RCC_TIM8_CLK_ENABLE()

#define PWM_CHANNEL_1 TIM_CHANNEL_1
#define PWM_CHANNEL_2 TIM_CHANNEL_2

/* 累计 TIM_Period个后产生一个更新或者中断*/
/* 当定时器从0计数到PWM_PERIOD_COUNT，即为PWM_PERIOD_COUNT+1次，为一个定时周期 */
#define PWM_PERIOD_COUNT (3600)

/* 通用控制定时器时钟源TIMxCLK = HCLK=72MHz */
/* 设定定时器频率为=TIMxCLK/(PWM_PRESCALER_COUNT)/PWM_PERIOD_COUNT */
#define PWM_PRESCALER_COUNT (2)

/* 最大比较值 */
#define PWM_MAX_PERIOD_COUNT (PWM_PERIOD_COUNT - 100)

/*PWM引脚*/
#define PWM_TIM_CH1_GPIO_CLK() __HAL_RCC_GPIOC_CLK_ENABLE();
#define PWM_TIM_CH1_GPIO_PORT GPIOC
#define PWM_TIM_CH1_PIN GPIO_PIN_6

#define PWM_TIM_CH2_GPIO_CLK() __HAL_RCC_GPIOC_CLK_ENABLE();
#define PWM_TIM_CH2_GPIO_PORT GPIOC
#define PWM_TIM_CH2_PIN GPIO_PIN_7

#define PWM_TIM_CH3_GPIO_CLK() __HAL_RCC_GPIOE_CLK_ENABLE();
#define PWM_TIM_CH3_GPIO_PORT GPIOE
#define PWM_TIM_CH3_PIN GPIO_PIN_13

extern TIM_HandleTypeDef TIM_TimeBaseStructure;

void TIM1_SetPWM_pulse(uint32_t channel, int compare);
void TIMx_Configuration(void);
#endif /*__BSP_MOTOR_TIM__*/
