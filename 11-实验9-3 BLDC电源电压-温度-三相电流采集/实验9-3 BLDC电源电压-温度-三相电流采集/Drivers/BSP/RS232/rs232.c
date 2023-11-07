/**
 ****************************************************************************************************
 * @file        rs232.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 7* @date       2021-10-27
 * @brief       RS232 ��������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� F407���������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com/forum.php
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20211027
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#include "./BSP/RS232/rs232.h"
#include "./SYSTEM/delay/delay.h"
#include "./DEBUG/debug.h"

UART_HandleTypeDef g_rs232_handler;                         /* RS232���ƾ��(����) */

/**
 * @brief       RS232�ӿڳ�ʼ������
 * @param       baudrate: ������
 * @retval      ��
 */
void rs232_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio_initure;
    
    RS232_TX_GPIO_CLK_ENABLE();                             /* ʹ�� TX�� ʱ�� */
    RS232_RX_GPIO_CLK_ENABLE();                             /* ʹ�� RX�� ʱ�� */
    RS232_UX_CLK_ENABLE();                                  /* ʹ�� ����5 ʱ�� */

    gpio_initure.Pin = RS232_TX_GPIO_PIN;                   /* TX���ţ�PC12�� */
    gpio_initure.Mode = GPIO_MODE_AF_PP;                    /* ����������� */
    gpio_initure.Pull = GPIO_PULLUP;                        /* ���� */
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;              /* ���� */
    gpio_initure.Alternate = RS232_AF_USART5;               /* ����Ϊ����5 */
    HAL_GPIO_Init(RS232_TX_GPIO_PORT, &gpio_initure);       /* ��ʼ��TX���� */

    gpio_initure.Pin = RS232_RX_GPIO_PIN;                   /* RX���ţ�PD2�� */
    gpio_initure.Alternate = RS232_AF_USART5;               /* ����Ϊ����5 */
    HAL_GPIO_Init(RS232_RX_GPIO_PORT, &gpio_initure);       /* ��ʼ��RX���� */

    /* USART ��ʼ������ */
    g_rs232_handler.Instance = RS232_UX;                    /* ����5 */
    g_rs232_handler.Init.BaudRate = baudrate;               /* ������ */
    g_rs232_handler.Init.WordLength = UART_WORDLENGTH_8B;   /* �ֳ�Ϊ8λ���ݸ�ʽ */
    g_rs232_handler.Init.StopBits = UART_STOPBITS_1;        /* һ��ֹͣλ */
    g_rs232_handler.Init.Parity = UART_PARITY_NONE;         /* ����żУ��λ */
    g_rs232_handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;   /* ��Ӳ������ */
    g_rs232_handler.Init.Mode = UART_MODE_TX_RX;            /* �շ�ģʽ */
    HAL_UART_Init(&g_rs232_handler);                        /* ��ʼ������5 */
    
    __HAL_UART_DISABLE_IT(&g_rs232_handler, UART_IT_TC);    /* �����ж� */

#if RS232_EN_RX                                             /* ���ʹ���˽��� */

    __HAL_UART_ENABLE_IT(&g_rs232_handler, UART_IT_RXNE);   /* ���������ж� */
    HAL_NVIC_EnableIRQ(RS232_UX_IRQn);                      /* ʹ��RS232�ж� */
    HAL_NVIC_SetPriority(RS232_UX_IRQn, 3, 3);              /* ��ռ���ȼ�3�������ȼ�3 */

#endif

}

#ifdef RS232_EN_RX                                          /* ���ʹ���˽��� */

void RS232_UX_IRQHandler(void)
{
    uint8_t res;

    if ((__HAL_UART_GET_FLAG(&g_rs232_handler, UART_FLAG_RXNE) != RESET)) /* ���յ����� */
    {
        HAL_UART_Receive(&g_rs232_handler, &res, 1, 1000);
        debug_handle(&res);
    }
}

#endif


