/**
 ****************************************************************************************************
 * @file        rs232.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 7* @date       2021-10-27
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

#include "./BSP/RS232/rs232.h"
#include "./SYSTEM/delay/delay.h"
#include "./DEBUG/debug.h"

UART_HandleTypeDef g_rs232_handler;                         /* RS232控制句柄(串口) */

/**
 * @brief       RS232接口初始化函数
 * @param       baudrate: 波特率
 * @retval      无
 */
void rs232_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio_initure;
    
    RS232_TX_GPIO_CLK_ENABLE();                             /* 使能 TX脚 时钟 */
    RS232_RX_GPIO_CLK_ENABLE();                             /* 使能 RX脚 时钟 */
    RS232_UX_CLK_ENABLE();                                  /* 使能 串口5 时钟 */

    gpio_initure.Pin = RS232_TX_GPIO_PIN;                   /* TX引脚（PC12） */
    gpio_initure.Mode = GPIO_MODE_AF_PP;                    /* 复用推挽输出 */
    gpio_initure.Pull = GPIO_PULLUP;                        /* 上拉 */
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
    gpio_initure.Alternate = RS232_AF_USART5;               /* 复用为串口5 */
    HAL_GPIO_Init(RS232_TX_GPIO_PORT, &gpio_initure);       /* 初始化TX引脚 */

    gpio_initure.Pin = RS232_RX_GPIO_PIN;                   /* RX引脚（PD2） */
    gpio_initure.Alternate = RS232_AF_USART5;               /* 复用为串口5 */
    HAL_GPIO_Init(RS232_RX_GPIO_PORT, &gpio_initure);       /* 初始化RX引脚 */

    /* USART 初始化设置 */
    g_rs232_handler.Instance = RS232_UX;                    /* 串口5 */
    g_rs232_handler.Init.BaudRate = baudrate;               /* 波特率 */
    g_rs232_handler.Init.WordLength = UART_WORDLENGTH_8B;   /* 字长为8位数据格式 */
    g_rs232_handler.Init.StopBits = UART_STOPBITS_1;        /* 一个停止位 */
    g_rs232_handler.Init.Parity = UART_PARITY_NONE;         /* 无奇偶校验位 */
    g_rs232_handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;   /* 无硬件流控 */
    g_rs232_handler.Init.Mode = UART_MODE_TX_RX;            /* 收发模式 */
    HAL_UART_Init(&g_rs232_handler);                        /* 初始化串口5 */
    
    __HAL_UART_DISABLE_IT(&g_rs232_handler, UART_IT_TC);    /* 开启中断 */

#if RS232_EN_RX                                             /* 如果使能了接收 */

    __HAL_UART_ENABLE_IT(&g_rs232_handler, UART_IT_RXNE);   /* 开启接收中断 */
    HAL_NVIC_EnableIRQ(RS232_UX_IRQn);                      /* 使能RS232中断 */
    HAL_NVIC_SetPriority(RS232_UX_IRQn, 3, 3);              /* 抢占优先级3，子优先级3 */

#endif

}

#ifdef RS232_EN_RX                                          /* 如果使能了接收 */

void RS232_UX_IRQHandler(void)
{
    uint8_t res;

    if ((__HAL_UART_GET_FLAG(&g_rs232_handler, UART_FLAG_RXNE) != RESET)) /* 接收到数据 */
    {
        HAL_UART_Receive(&g_rs232_handler, &res, 1, 1000);
        debug_handle(&res);
    }
}

#endif


