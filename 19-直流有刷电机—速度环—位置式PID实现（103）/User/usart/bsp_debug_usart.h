#ifndef __DEBUG_USART_H
#define __DEBUG_USART_H

#include "stm32f1xx.h"
#include <stdio.h>
#include "./protocol/protocol.h"

// ���ڲ�����
#define DEBUG_USART_BAUDRATE 115200

// ���Ŷ���
/*******************************************************/
#define DEBUG_USART USART1
#define DEBUG_USART_CLK_ENABLE() __HAL_RCC_USART1_CLK_ENABLE();

#define RCC_PERIPHCLK_UARTx RCC_PERIPHCLK_USART1
#define RCC_UARTxCLKSOURCE_SYSCLK RCC_USART1CLKSOURCE_SYSCLK

#define DEBUG_USART_RX_GPIO_PORT GPIOA
#define DEBUG_USART_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define DEBUG_USART_RX_PIN GPIO_PIN_10

#define DEBUG_USART_TX_GPIO_PORT GPIOA
#define DEBUG_USART_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define DEBUG_USART_TX_PIN GPIO_PIN_9

#define DEBUG_USART_IRQHandler USART1_IRQHandler
#define DEBUG_USART_IRQ USART1_IRQn

// ������1���õ�PB6\PB7��
#define DEBUG_USART_AF_ENABLE() __HAL_AFIO_REMAP_USART1_ENABLE()

/************************************************************/

void uart_FlushRxBuffer(void);
void Usart_SendByte(uint8_t str);
void Usart_SendString(uint8_t *str);
void DEBUG_USART_Config(void);
int fputc(int ch, FILE *f);
int fgetc(FILE *f);
extern UART_HandleTypeDef UartHandle;
#endif /* __USART1_H */
