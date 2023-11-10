#ifndef __LED_H
#define __LED_H

#include "stm32f1xx.h"

// ���Ŷ���
/*******************************************************/
// LED1
#define LED1_PIN GPIO_PIN_0
#define LED1_GPIO_PORT GPIOB
#define LED1_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

// LED2
#define LED2_PIN GPIO_PIN_1
#define LED2_GPIO_PORT GPIOB
#define LED2_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

// LED3
#define LED3_PIN GPIO_PIN_5
#define LED3_GPIO_PORT GPIOB
#define LED3_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

/************************************************************/

#define ON GPIO_PIN_RESET
#define OFF GPIO_PIN_SET

#define LED1(a) HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, a)

#define LED2(a) HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, a)

#define LED3(a) HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, a)

#define digitalHi(p, i) \
	{                   \
		p->BSRR = i;    \
	} // ����Ϊ�ߵ�ƽ
#define digitalLo(p, i)              \
	{                                \
		p->BSRR = (uint32_t)i << 16; \
	} // ����͵��?
#define digitalToggle(p, i) \
	{                       \
		p->ODR ^= i;        \
	} // ������?״̬

/* �������IO�ĺ� */
#define LED1_TOGGLE digitalToggle(LED1_GPIO_PORT, LED1_PIN)
#define LED1_OFF digitalHi(LED1_GPIO_PORT, LED1_PIN)
#define LED1_ON digitalLo(LED1_GPIO_PORT, LED1_PIN)

#define LED2_TOGGLE digitalToggle(LED2_GPIO_PORT, LED2_PIN)
#define LED2_OFF digitalHi(LED2_GPIO_PORT, LED2_PIN)
#define LED2_ON digitalLo(LED2_GPIO_PORT, LED2_PIN)

#define LED3_TOGGLE digitalToggle(LED3_GPIO_PORT, LED3_PIN)
#define LED3_OFF digitalHi(LED3_GPIO_PORT, LED3_PIN)
#define LED3_ON digitalLo(LED3_GPIO_PORT, LED3_PIN)

// ��(ȫ���ر�)
#define LED_ALLOFF \
	LED1_OFF;      \
	LED2_OFF;      \
	LED3_OFF;

void LED_GPIO_Config(void);

#endif /* __LED_H */
