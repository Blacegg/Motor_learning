/**
 ******************************************************************************
 * @file    main.c
 * @author  fire
 * @version V1.0
 * @date    2020-xx-xx
 * @brief   ֱ����ˢ���ٵ��-��������
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
#include ".\motor_control\bsp_motor_control.h"
#include "./led/bsp_led.h"
#include "./key/bsp_key.h"
#include "./usart/bsp_debug_usart.h"
#include "./adc/bsp_adc.h"

int first_start = 0;
/**
 * @brief  ������
 * @param  ��
 * @retval ��
 */
int main(void)
{
  __IO uint16_t ChannelPulse = PWM_MAX_PERIOD_COUNT / 10;
  uint8_t flag = 0;

  /* ��ʼ��ϵͳʱ��Ϊ72MHz */
  SystemClock_Config();

  /* �������üĴ���ʱ�� */
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* ��ʼ������GPIO */
  Key_GPIO_Config();

  /* LED �Ƴ�ʼ�� */
  LED_GPIO_Config();

  ADC_Init();

  /* ���Դ��ڳ�ʼ�� */
  DEBUG_USART_Config();

  printf("Ұ��ֱ����ˢ���������������\r\n");

  /* �����ʼ�� */
  bldcm_init();

  while (1)
  {
    /* ɨ��KEY1 */
    if (Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
    {
      if (first_start == 0)
      {
        /* ʹ�ܵ�� */
        set_bldcm_speed(ChannelPulse);
        set_bldcm_enable();
        first_start = 1;
      }

      static int is_run_flag;
      if (ChannelPulse == 0) // ռ�ձȴ������Ӻ� ����ʹ��һ��
      {
        is_run_flag = 1;
      }
      /* ����ռ�ձ� */
      ChannelPulse += PWM_MAX_PERIOD_COUNT / 10;

      if (ChannelPulse > PWM_MAX_PERIOD_COUNT)
        ChannelPulse = PWM_MAX_PERIOD_COUNT;

      set_bldcm_speed(ChannelPulse);
      if (is_run_flag == 1)
      {
        set_bldcm_enable();
        is_run_flag = 0;
      }
    }

    /* ɨ��KEY2 */
    if (Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
    {
      /* ����ռ�ձ� */
      if (ChannelPulse < PWM_MAX_PERIOD_COUNT / 10)
      {
        ChannelPulse = 0;
        set_bldcm_disable();
        first_start = 0;
      }
      else
        ChannelPulse -= PWM_MAX_PERIOD_COUNT / 10;

      set_bldcm_speed(ChannelPulse);
    }
    /* ÿ50�����ȡһ���¶ȡ���ѹ */
    if (HAL_GetTick() % 50 == 0 && flag == 0)
    {
      flag = 1;
      int32_t current_v = get_curr_val_v();
      int32_t current_u = get_curr_val_u();
      int32_t current_w = get_curr_val_w();
      float current_temp = get_ntc_t_val();
      float current_vbus = get_vbus_val();
      printf("��Դ��ѹ=%.2fV\t����¶�=%.2f���϶�\tռ�ձ�=%%%.2f\r\n", current_vbus, current_temp,(float)ChannelPulse/PWM_PERIOD_COUNT);
      printf("U�����=%dmA\tV�����=%dmA\tW�����=%dmA\r\n\n", current_v, current_u, current_w);
    }
    else if (HAL_GetTick() % 50 != 0 && flag == 1)
    {
      flag = 0;
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
