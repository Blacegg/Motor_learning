/**
 ******************************************************************************
 * @file    main.c
 * @author  fire
 * @version V1.0
 * @date    2020-xx-xx
 * @brief   �������--���μӼ���ʵ��
 ******************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:Ұ�� F103-���� STM32 ������
 * ��̳    :http://www.firebbs.cn
 * �Ա�    :https://fire-stm32.taobao.com
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

// Ӳ�������ٶȵ����ޣ�����㷨�����Ƿ���Դﵽ����
__IO uint32_t set_speed = 4000; // �ٶ� ��λΪ0.05rad/sec
// ���ٶȺͼ��ٶ�ѡȡһ�����ʵ����Ҫ��ֵԽ���ٶȱ仯Խ�죬�Ӽ��ٽ׶αȽ϶���
// ���Լ��ٶȺͼ��ٶ�ֵһ������ʵ��Ӧ���жೢ�Գ����Ľ��
__IO uint32_t accel_val = 500; // ���ٶ� ��λΪ0.05rad/sec^2
__IO uint32_t decel_val = 500; // ���ٶ� ��λΪ0.05rad/sec^2

/**
 * @brief  ������
 * @param  ��
 * @retval ��
 */
int main(void)
{
  /* ��ʼ��ϵͳʱ��Ϊ72MHz */
  SystemClock_Config();
  /* �������üĴ���ʱ�� */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  /*��ʼ��USART ����ģʽΪ 115200 8-N-1���жϽ���*/
  DEBUG_USART_Config();

  /*���������ʼ��*/
  stepper_Init();
  int i = 0, dir_val = 0;

  while (1)
  {
    /*ͨ��ȡ��ʵ������ת���棬����ת֮��ת*/
    dir_val = (++i % 2) ? CW : CCW;
    printf("dir_val =%d\r\n", dir_val);
    /*�жϷ��򣬽��в�������*/
    if (dir_val)
    {
      /*������ת����������stepΪ������ʾ��ת*/
      stepper_move_T(6400 * 10, accel_val, decel_val, set_speed);
    }
    else
    {
      /*���㷴ת����������stepΪ������ʾ��ת*/
      stepper_move_T(-6400 * 10, accel_val, decel_val, set_speed);
    }

    HAL_Delay(2000); // Ҫ����ת�ٷ�����ת
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
