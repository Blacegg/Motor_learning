#ifndef __BSP_USART__
#define __BSP_USART__

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include <stdio.h>
#include "./protocol/protocol.h"

// 串口波特率
#define DEBUG_USART_BAUDRATE 115200

#define DEBUG_USART USART1
#define DEBUG_USART_CLK_ENABLE() __HAL_RCC_USART1_CLK_ENABLE()

// 发送
#define DEBUG_USART_Tx_PORT GPIOA
#define DEBUG_USART_Tx_Pin GPIO_PIN_9
#define DEBUG_USART_Tx_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

// 接收
#define DEBUG_USART_Rx_PORT GPIOA
#define DEBUG_USART_Rx_Pin GPIO_PIN_10
#define DEBUG_USART_Rx_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define DEBUG_USART_IRQHandler USART1_IRQHandler
#define DEBUG_USART_IRQ USART1_IRQn
#define DEBUG_USART_AF_ENABLE()					      	__HAL_AFIO_REMAP_USART1_ENABLE()

void Usart_SendString(uint8_t *str);
void USART_Config(void);
int fputc(int ch, FILE *f);
int fgetc(FILE *f);

extern UART_HandleTypeDef UartHandle;

#endif /*__BSP_USART__*/
