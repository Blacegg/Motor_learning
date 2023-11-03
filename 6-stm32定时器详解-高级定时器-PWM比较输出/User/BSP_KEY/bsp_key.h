#ifndef __BSP_KEY__
#define __BSP_KEY__

#include "stm32f1xx_hal.h"

#define             KEY_ON                              1
#define             KEY_OFF                             0

//°´¼ü1
#define             KEY1_GPIO_PORT                      GPIOA
#define             KEY1_GPIO_Pin                       GPIO_PIN_0
#define             KEY1_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOA_CLK_ENABLE()

//°´¼ü2
#define             KEY2_GPIO_PORT                      GPIOC
#define             KEY2_GPIO_Pin                       GPIO_PIN_13
#define             KEY2_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOC_CLK_ENABLE()

void KEY_GPIO_Config(void);
uint8_t KEY_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);

#endif /*__BSP_KEY__*/
