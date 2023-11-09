/**
 ******************************************************************************
 * @file    bsp_debug_usart.c
 * @author  fire
 * @version V1.0
 * @date    2016-xx-xx
 * @brief   ʹ�ô���1���ض���c��printf������usart�˿ڣ��жϽ���ģʽ
 ******************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:Ұ�� STM32 F103 ������
 * ��̳    :http://www.firebbs.cn
 * �Ա�    :http://firestm32.taobao.com
 *
 ******************************************************************************
 ***/

#include "./usart/bsp_debug_usart.h"

UART_HandleTypeDef UartHandle;

/**
 * @brief  DEBUG_USART GPIO ����,����ģʽ���á�115200 8-N-1
 * @param  ��
 * @retval ��
 */
void DEBUG_USART_Config(void)
{

  UartHandle.Instance = DEBUG_USART;

  UartHandle.Init.BaudRate = DEBUG_USART_BAUDRATE;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;

  HAL_UART_Init(&UartHandle);

  /*ʹ�ܴ��ڽ��ն� */
  __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);
}

/**
 * @brief UART MSP ��ʼ��
 * @param huart: UART handle
 * @retval ��
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  DEBUG_USART_CLK_ENABLE();
  // DEBUG_USART_AF_ENABLE();
  DEBUG_USART_RX_GPIO_CLK_ENABLE();
  DEBUG_USART_TX_GPIO_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  /* ����Tx����  */
  GPIO_InitStruct.Pin = DEBUG_USART_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStruct);

  /* ����Rx����Ϊ���ù��� */
  GPIO_InitStruct.Pin = DEBUG_USART_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT; // ģʽҪ����Ϊ��������ģʽ��
  HAL_GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(DEBUG_USART_IRQ, 0, 1); // ��ռ���ȼ�0�������ȼ�1
  HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ);         // ʹ��USART1�ж�ͨ��
}

/*****************  �����ַ� **********************/
void Usart_SendByte(uint8_t str)
{
  HAL_UART_Transmit(&UartHandle, &str, 1, 1000);
}

/*****************  �����ַ��� **********************/
void Usart_SendString(uint8_t *str)
{
  unsigned int k = 0;
  do
  {
    HAL_UART_Transmit(&UartHandle, (uint8_t *)(str + k), 1, 1000);
    k++;
  } while (*(str + k) != '\0');
}
// �ض���c�⺯��printf������DEBUG_USART���ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
  /* ����һ���ֽ����ݵ�����DEBUG_USART */
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 1000);

  return (ch);
}

int fgetc(FILE *f)
{
  int ch;
  HAL_UART_Receive(&UartHandle, (uint8_t *)&ch, 1, 1000);
  return (ch);
}

/*********************************************END OF FILE**********************/
