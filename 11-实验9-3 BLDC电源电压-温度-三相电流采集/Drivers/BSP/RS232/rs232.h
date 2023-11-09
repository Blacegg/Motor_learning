/**
 ****************************************************************************************************
 * @file        rs232.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-27
 * @brief       RS232 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com/forum.php
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211027
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __RS232_H
#define __RS232_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* RS232 引脚 和 串口 定义 
 * 默认是针对RS232接口的.
 * 注意: 通过修改这10个宏定义, 可以支持UART1~UART7任意一个串口.
 */

#define RS232_TX_GPIO_PORT                  GPIOC
#define RS232_TX_GPIO_PIN                   GPIO_PIN_12
#define RS232_TX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)     /* PC口时钟使能 */

#define RS232_RX_GPIO_PORT                  GPIOD
#define RS232_RX_GPIO_PIN                   GPIO_PIN_2
#define RS232_RX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)     /* PD口时钟使能 */

#define RS232_UX                            UART5
#define RS232_UX_IRQn                       UART5_IRQn
#define RS232_UX_IRQHandler                 UART5_IRQHandler
#define RS232_UX_CLK_ENABLE()               do{ __HAL_RCC_UART5_CLK_ENABLE(); }while(0)     /* USART5 时钟使能 */

#define RS232_AF_USART5                     GPIO_AF8_UART5                                  /* RS232复用为USART5 */

#define RS232_EN_RX                         1                                               /* 使能（1）/禁止（0）RS232接收 */

extern UART_HandleTypeDef g_rs232_handler;

/******************************************************************************************/
void rs232_init( uint32_t baudrate);  /* RS232初始化 */

#endif
















