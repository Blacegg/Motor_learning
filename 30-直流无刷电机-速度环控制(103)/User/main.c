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
// #include "./adc/bsp_adc.h"
#include "pid.h"
#include "./protocol/protocol.h"
#include "./tim/bsp_basic_tim.h"

int first_start = 0;
int scend_start = 0;
/**
 * @brief  主函数
 * @param  无
 * @retval 无
 */
int main(void)
{
  __IO uint16_t ChannelPulse = PWM_MAX_PERIOD_COUNT / 10;
  // uint8_t flag = 0;
  int16_t target_speed = 1800;

  /* 初始化系统时钟为72MHz */
  SystemClock_Config();

  /* 开启复用寄存器时钟 */
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* 初始化按键GPIO */
  Key_GPIO_Config();

  /* LED 灯初始化 */
  LED_GPIO_Config();

  // ADC_Init();

  /* 协议初始化 */
  protocol_init();

  /* 调试串口初始化 */
  DEBUG_USART_Config();

  /* PID参数初始化 */
  PID_param_init();

  /* 周期控制定时器 50ms */
  TIMx_Configuration();

  // printf("野火直流无刷电机按键控制例程\r\n");

  /* 电机初始化 */
  bldcm_init();

  /* 设置目标速度 */
  set_pid_target(target_speed);

#if defined(PID_ASSISTANT_EN)
  set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);            // 同步上位机的启动按钮状态
  set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &target_speed, 1); // 给通道 1 发送目标值
#endif

  while (1)
  {
    /* 接收数据处理 */
    receiving_process();

    /* 扫描KEY1 */
    if (Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
    {
      if (first_start == 0)
      {
        /* 使能电机 */
        set_bldcm_enable();
        first_start = 1;

#if defined(PID_ASSISTANT_EN)
        set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0); // 同步上位机的启动按钮状态
#endif
      }

      target_speed += 300;

      if (target_speed > 3000)
        target_speed = 3000;
      if (target_speed == 0)
      {
        /* 停止电机 */
        set_bldcm_disable();
        first_start = 0;
        PID_param_init();
#if defined(PID_ASSISTANT_EN)
        set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0); // 同步上位机的启动按钮状态
#endif
      }

      set_pid_target(target_speed);

#if defined(PID_ASSISTANT_EN)
      set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &target_speed, 1); // 给通道 1 发送目标值
#endif
    }

    /* 扫描KEY2 */
    if (Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
    {
      target_speed -= 300;

      if (target_speed < -3000)
      {
        target_speed = -3000;
      }
      if (target_speed == 0)
      {
        /* 停止电机 */
        set_bldcm_disable();
        first_start = 0;
        PID_param_init();
#if defined(PID_ASSISTANT_EN)
        set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0); // 同步上位机的启动按钮状态
#endif
      }
      if (target_speed == -300)
      {
        /* 使能电机 */
        set_bldcm_enable();
        first_start = -1;

#if defined(PID_ASSISTANT_EN)
        set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0); // 同步上位机的启动按钮状态
#endif
      }

      set_pid_target(target_speed);

#if defined(PID_ASSISTANT_EN)
      set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &target_speed, 1); // 给通道 1 发送目标值
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
