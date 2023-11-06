#ifndef __BSP_LED__
#define __BSP_LED__

#include "stm32f1xx_hal.h"

//ºìµÆ
#define             LED1_GPIO_PORT                      GPIOB
#define             LED1_GPIO_Pin                       GPIO_PIN_5
#define             LED1_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOB_CLK_ENABLE()

//ÂÌµÆ
#define             LED2_GPIO_PORT                      GPIOB
#define             LED2_GPIO_Pin                       GPIO_PIN_0
#define             LED2_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOB_CLK_ENABLE()

//À¶µÆ
#define             LED3_GPIO_PORT                      GPIOB
#define             LED3_GPIO_Pin                       GPIO_PIN_1
#define             LED3_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOB_CLK_ENABLE()

#define             NO                                  GPIO_PIN_RESET
#define             OFF                                 GPIO_PIN_SET

//¼Ä´æÆ÷¿ØÖÆ
#define             SetRegister_H(p,i)                  {p->BSRR|=i;}
#define             SetRegister_L(p,i)                  {p->BSRR|=(uint32_t)i<<16;}
#define             SetRegister_Toggle(p,i)             {p->ODR^=i;}

//ÁÁµÆ
//ºìµÆ
#define             LED_Red_ON                           SetRegister_L(LED1_GPIO_PORT,LED1_GPIO_Pin)
#define             LED_Red_OFF                          SetRegister_H(LED1_GPIO_PORT,LED1_GPIO_Pin)
#define             LED_Red_Toggle                       SetRegister_Toggle(LED1_GPIO_PORT,LED1_GPIO_Pin)

//ÂÌµÆ
#define             LED_Green_ON                         SetRegister_L(LED2_GPIO_PORT,LED2_GPIO_Pin)
#define             LED_Green_OFF                        SetRegister_H(LED2_GPIO_PORT,LED2_GPIO_Pin)
#define             LED_Green_Toggle                     SetRegister_Toggle(LED2_GPIO_PORT,LED2_GPIO_Pin)

//À¶µÆ
#define             LED_Blue_ON                          SetRegister_L(LED3_GPIO_PORT,LED3_GPIO_Pin)
#define             LED_Blue_OFF                         SetRegister_H(LED3_GPIO_PORT,LED3_GPIO_Pin)
#define             LED_Blue_Toggle                      SetRegister_Toggle(LED3_GPIO_PORT,LED3_GPIO_Pin)

//µ¥µÆ
#define             LED_Red                              {LED_Red_ON;LED_Green_OFF;LED_Blue_OFF;}
#define             LED_Blue                             {LED_Red_OFF;LED_Green_OFF;LED_Blue_ON;}
#define             LED_Green                            {LED_Red_OFF;LED_Green_ON;LED_Blue_OFF;}
#define             LED_AllOFF                           {LED_Red_OFF;LED_Green_OFF;LED_Blue_OFF;}

void LED_GPIO_Config(void);

#endif /*__BSP_LED__*/
