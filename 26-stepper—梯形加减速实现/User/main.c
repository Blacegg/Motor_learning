/**
 ******************************************************************************
 * @file    main.c
 * @author  fire
 * @version V1.0
 * @date    2020-xx-xx
 * @brief   步进电机--梯形加减速实现
 ******************************************************************************
 * @attention
 *
 * 实验平台:野火 F103-拂晓 STM32 开发板
 * 论坛    :http://www.firebbs.cn
 * 淘宝    :https://fire-stm32.taobao.com
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include "./usart/bsp_debug_usart.h"
#include "./stepper/bsp_stepper_init.h"
#include "./key/bsp_key.h"
#include "./led/bsp_led.h"
#include "./stepper/bsp_stepper_T_speed.h"

// 硬件决定速度的上限，软件算法决定是否可以达到上限
__IO uint32_t set_speed = 4000; // 速度 单位为0.05rad/sec
// 加速度和减速度选取一般根据实际需要，值越大速度变化越快，加减速阶段比较抖动
// 所以加速度和减速度值一般是在实际应用中多尝试出来的结果
__IO uint32_t accel_val = 500; // 加速度 单位为0.05rad/sec^2
__IO uint32_t decel_val = 500; // 减速度 单位为0.05rad/sec^2

/**
 * @brief  主函数
 * @param  无
 * @retval 无
 */
int main(void)
{
  /* 初始化系统时钟为72MHz */
  SystemClock_Config();
  /* 开启复用寄存器时钟 */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  /*初始化USART 配置模式为 115200 8-N-1，中断接收*/
  DEBUG_USART_Config();

  /*步进电机初始化*/
  stepper_Init();
  int i = 0, dir_val = 0;

  while (1)
  {
    /*通过取余实现正反转交替，即正转之后反转*/
    dir_val = (++i % 2) ? CW : CCW;
    printf("dir_val =%d\r\n", dir_val);
    /*判断方向，进行参数计算*/
    if (dir_val)
    {
      /*计算正转参数，参数step为正，表示正转*/
      stepper_move_T(6400 * 10, accel_val, decel_val, set_speed);
    }
    else
    {
      /*计算反转参数，参数step为负，表示反转*/
      stepper_move_T(-6400 * 10, accel_val, decel_val, set_speed);
    }

    HAL_Delay(2000); // 要等旋转再反向旋转
  }
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 72000000
 *            HCLK(Hz)                       = 72000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 2
 *            APB2 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            HSE PREDIV1                    = 2
 *            PLLMUL                         = 9
 *            Flash Latency(WS)              = 0
 * @param  None
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  oscinitstruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&oscinitstruct) != HAL_OK)
  {
    /* Initialization Error */
    while (1)
      ;
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2) != HAL_OK)
  {
    /* Initialization Error */
    while (1)
      ;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
