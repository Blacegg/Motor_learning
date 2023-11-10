/**
 ******************************************************************************
 * @file    main.c
 * @author  fire
 * @version V1.0
 * @date    2020-xx-xx
 * @brief   PID�㷨ʵ��
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
#include <stdlib.h>
#include "./tim/bsp_basic_tim.h"
#include "./led/bsp_led.h"
#include "./pid/bsp_pid.h"
#include "./usart/bsp_debug_usart.h"
#include "bsp_key.h"
#include "./motor_control/bsp_motor_control.h"
#include "./protocol/protocol.h"
#include "./pid/bsp_pid.h"
#include "./encoder/bsp_encoder.h"

/**
 * @brief  ������
 * @param  ��
 * @retval ��
 */

int main(void)
{
  int32_t target_location = PER_CYCLE_PULSES;
  HAL_Init();
  /* ��ʼ��ϵͳʱ��Ϊ72MHz */
  SystemClock_Config();
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  Key_GPIO_Config();
  /*��ʼ��USART ����ģʽΪ 115200 8-N-1���жϽ���*/

  protocol_init();
  DEBUG_USART_Config();

  motor_init();
  set_motor_disable();
  Encoder_Init();
  TIMx_Configuration();

  LED_GPIO_Config();
  PID_param_init();
#if defined(PID_ASSISTANT_EN)
  set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);               // ͬ����λ����������ť״̬
  set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &target_location, 1); // ��ͨ�� 1 ����Ŀ��ֵ
#endif

  while (1)
  {
    receiving_process();
    /* ɨ��KEY3 */
    if (Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
    {
#if defined(PID_ASSISTANT_EN)
      set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0); // ͬ����λ����������ť״̬
#endif
      set_pid_target(&pid_location, target_location);
      set_motor_enable();
    }

    /* ɨ��KEY4 */
    if (Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
    {
			static uint8_t i=0;
			//if(i%3==0)
			//{
			//	target_location -= PER_CYCLE_PULSES;
			//}
			//else
			{
			target_location += PER_CYCLE_PULSES;
			}
			//i++;
      
      set_pid_target(&pid_location, target_location);
#if defined(PID_ASSISTANT_EN)
      set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &target_location, 1); // ¸øÍ¨µÀ 1 ·¢ËÍÄ¿±êÖµ
#endif
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    /* Initialization Error */
    while (1)
      ;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
