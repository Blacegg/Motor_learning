#ifndef __BSP_GENERAL_TIM_H
#define	__BSP_GENERAL_TIM_H

#include "stm32f1xx.h"

/*宏定义*/
#define GENERAL_TIM                        	TIM3
#define GENERAL_TIM_CLK_ENABLE()  					__HAL_RCC_TIM3_CLK_ENABLE()

#define PWM_CHANNEL                       TIM_CHANNEL_1
//#define PWM_CHANNEL                       TIM_CHANNEL_2
//#define PWM_CHANNEL                       TIM_CHANNEL_3
//#define PWM_CHANNEL                       TIM_CHANNEL_4

#define STEERING_ENGINE_CHANNEL             PWM_CHANNEL

/* 累计 TIM_Period个后产生一个更新或者中断*/		
/* 当定时器从0计数到PWM_PERIOD_COUNT，即为PWM_PERIOD_COUNT+1次，为一个定时周期 */
#define PWM_PERIOD_COUNT     999

/* 通用控制定时器时钟源TIMxCLK = HCLK=72MHz */
/* 设定定时器频率为=TIMxCLK/(PWM_PRESCALER_COUNT+1) */
#define PWM_PRESCALER_COUNT     1440

/*PWM引脚*/
#define GENERAL_TIM3_CH1_GPIO_PORT           GPIOB
#define GENERAL_TIM3_CH1_PIN                 GPIO_PIN_4

#define GENERAL_TIM3_CH2_GPIO_PORT           GPIOB
#define GENERAL_TIM3_CH2_PIN                 GPIO_PIN_5

#define GENERAL_TIM3_CH3_GPIO_PORT           GPIOB
#define GENERAL_TIM3_CH3_PIN                 GPIO_PIN_0

#define STEERING_ENGINE_PORT_CLK()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define STEERING_ENGINE_PORT          GENERAL_TIM3_CH1_GPIO_PORT
#define STEERING_ENGINE_PIN           GENERAL_TIM3_CH1_PIN

/* 将TIM3引脚重映射到PB4\5\0\1 <---->CH1\2\3\4 PB4为JTRST,需要禁用*/
#define TIM3_AF_ENABLE()							__HAL_AFIO_REMAP_TIM3_PARTIAL();\
																			__HAL_AFIO_REMAP_SWJ_NONJTRST()


extern TIM_HandleTypeDef  TIM_TimeBaseStructure;

extern void TIMx_Configuration(void);
extern void TIM_SetPWM_pulse(uint32_t channel,int compare);
void set_steering_gear_dutyfactor(uint16_t dutyfactor);
void set_steering_gear_angle(uint16_t angle);
void show_help(void);
void deal_serial_data(void);

#endif

