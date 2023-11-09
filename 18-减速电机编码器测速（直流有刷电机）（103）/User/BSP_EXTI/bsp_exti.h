#ifndef __BSP_EXTI__
#define __BSP_EXTI__

#include "stm32f1xx_hal.h"

#define             KEY_ON                              1
#define             KEY_OFF                             0

//°´¼ü1
#define             KEY1_GPIO_PORT                      GPIOA
#define             KEY1_GPIO_Pin                       GPIO_PIN_0
#define             KEY1_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOA_CLK_ENABLE()
#define             KEY1_EXTI_IRQ                       EXTI0_IRQn
#define             KEY1_EXTI_Handler                   EXTI0_IRQHandler

//°´¼ü2
#define             KEY2_GPIO_PORT                      GPIOC
#define             KEY2_GPIO_Pin                       GPIO_PIN_13
#define             KEY2_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOC_CLK_ENABLE()
#define             KEY2_EXTI_IRQ                       EXTI15_10_IRQn
#define             KEY2_EXTI_Handler                   EXTI15_10_IRQHandler

void KEY_EXTI_Config(void);

#endif /*__BSP_EXTI__*/
