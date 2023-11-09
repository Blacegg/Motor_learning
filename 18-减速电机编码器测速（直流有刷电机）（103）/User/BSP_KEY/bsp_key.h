#ifndef __BSP_KEY__
#define __BSP_KEY__

#include "stm32f1xx_hal.h"

#define             KEY_ON                              1
#define             KEY_OFF                             0

//按键1
#define             KEY1_GPIO_PORT                      GPIOA
#define             KEY1_GPIO_Pin                       GPIO_PIN_0
#define             KEY1_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOA_CLK_ENABLE()

//按键2
#define             KEY2_GPIO_PORT                      GPIOC
#define             KEY2_GPIO_Pin                       GPIO_PIN_13
#define             KEY2_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOC_CLK_ENABLE()

//按键3
#define             KEY3_GPIO_PORT                      GPIOA
#define             KEY3_GPIO_Pin                       GPIO_PIN_2
#define             KEY3_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOA_CLK_ENABLE()

//按键4
#define             KEY4_GPIO_PORT                      GPIOA
#define             KEY4_GPIO_Pin                       GPIO_PIN_3
#define             KEY4_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOA_CLK_ENABLE()

void KEY_GPIO_Config(void);
uint8_t KEY_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);

#endif /*__BSP_KEY__*/
