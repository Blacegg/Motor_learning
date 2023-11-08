#ifndef __BSP_BEEP__
#define __BSP_BEEP__

#include "stm32f1xx_hal.h"

//·äÃùÆ÷
#define             BEEP_GPIO_PORT                      GPIOA
#define             BEEP_GPIO_Pin                       GPIO_PIN_8
#define             BEEP_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOA_CLK_ENABLE()

#define             BEEP_ON                             HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_GPIO_Pin, GPIO_PIN_SET);
#define             BEEP_OFF                            HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_GPIO_Pin, GPIO_PIN_RESET);

void BEEP_GPIO_Config(void);

#endif /*__BSP_BEEP__*/
