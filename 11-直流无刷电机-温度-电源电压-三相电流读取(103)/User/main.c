/**
 ******************************************************************************
 * @file    main.c
 * @author  fire
 * @version V1.0
 * @date    2020-xx-xx
 * @brief   直流有刷减速电机-按键控制
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
#include <stdlib.h>
#include ".\motor_control\bsp_motor_control.h"
#include "./led/bsp_led.h"
#include "./key/bsp_key.h"
#include "./usart/bsp_debug_usart.h"
#include "./adc/bsp_adc.h"

int first_start = 0;
/**
 * @brief  主函数
 * @param  无
 * @retval 无
 */
int main(void)
{
  __IO uint16_t ChannelPulse = PWM_MAX_PERIOD_COUNT / 10;
  uint8_t flag = 0;

  /* 初始化系统时钟为72MHz */
  SystemClock_Config();

  /* 开启复用寄存器时钟 */
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* 初始化按键GPIO */
  Key_GPIO_Config();

  /* LED 灯初始化 */
  LED_GPIO_Config();

  ADC_Init();

  /* 调试串口初始化 */
  DEBUG_USART_Config();

  printf("野火直流无刷电机按键控制例程\r\n");

  /* 电机初始化 */
  bldcm_init();

  while (1)
  {
    /* 扫描KEY1 */
    if (Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
    {
      if (first_start == 0)
      {
        /* 使能电机 */
        set_bldcm_speed(ChannelPulse);
        set_bldcm_enable();
        first_start = 1;
      }

      static int is_run_flag;
      if (ChannelPulse == 0) // 占空比从零增加后 重新使能一次
      {
        is_run_flag = 1;
      }
      /* 增大占空比 */
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

    /* 扫描KEY2 */
    if (Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
    {
      /* 减少占空比 */
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
    /* 每50毫秒读取一次温度、电压 */
    if (HAL_GetTick() % 50 == 0 && flag == 0)
    {
      flag = 1;
      int32_t current_v = get_curr_val_v();
      int32_t current_u = get_curr_val_u();
      int32_t current_w = get_curr_val_w();
      float current_temp = get_ntc_t_val();
      float current_vbus = get_vbus_val();
      printf("电源电压=%.2fV\t电机温度=%.2f摄氏度\t占空比=%%%.2f\r\n", current_vbus, current_temp,(float)ChannelPulse/PWM_PERIOD_COUNT);
      printf("U相电流=%dmA\tV相电流=%dmA\tW相电流=%dmA\r\n\n", current_v, current_u, current_w);
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
