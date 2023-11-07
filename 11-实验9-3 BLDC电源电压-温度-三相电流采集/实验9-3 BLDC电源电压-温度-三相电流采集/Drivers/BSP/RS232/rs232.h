/**
 ****************************************************************************************************
 * @file        rs232.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-27
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

#ifndef __RS232_H
#define __RS232_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* RS232 ���� �� ���� ���� 
 * Ĭ�������RS232�ӿڵ�.
 * ע��: ͨ���޸���10���궨��, ����֧��UART1~UART7����һ������.
 */

#define RS232_TX_GPIO_PORT                  GPIOC
#define RS232_TX_GPIO_PIN                   GPIO_PIN_12
#define RS232_TX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)     /* PC��ʱ��ʹ�� */

#define RS232_RX_GPIO_PORT                  GPIOD
#define RS232_RX_GPIO_PIN                   GPIO_PIN_2
#define RS232_RX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)     /* PD��ʱ��ʹ�� */

#define RS232_UX                            UART5
#define RS232_UX_IRQn                       UART5_IRQn
#define RS232_UX_IRQHandler                 UART5_IRQHandler
#define RS232_UX_CLK_ENABLE()               do{ __HAL_RCC_UART5_CLK_ENABLE(); }while(0)     /* USART5 ʱ��ʹ�� */

#define RS232_AF_USART5                     GPIO_AF8_UART5                                  /* RS232����ΪUSART5 */

#define RS232_EN_RX                         1                                               /* ʹ�ܣ�1��/��ֹ��0��RS232���� */

extern UART_HandleTypeDef g_rs232_handler;

/******************************************************************************************/
void rs232_init( uint32_t baudrate);  /* RS232��ʼ�� */

#endif
















