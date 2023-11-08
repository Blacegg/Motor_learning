#include "main.h"
#include <stdlib.h>
#include "bsp_basic_tim.h"
#include "bsp_led.h"
#include "bsp_key.h"
#include "bsp_pid.h"
#include "bsp_usart.h"
#include "protocol.h"

extern _pid pid;
extern float set_point;
extern int pid_status;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  USART_Config();
  LED_GPIO_Config();
  protocol_init();
 __HAL_RCC_SYSCFG_CLK_ENABLE();
  KEY_GPIO_Config();
  /* ��ʼ��������ʱ����ʱ��20ms����һ���ж� */
  TIMx_Configuration();
  /* PID�㷨������ʼ�� */
  PID_param_init();
  int run_i = 0;

#if defined(PID_ASSISTANT_EN)
  int temp = set_point;                                      // ��λ����Ҫ����������ת��һ��
  set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1); // ��ͨ�� 1 ����Ŀ��ֵ
#endif
  while (1)
  {
    /* �������ݴ��� */
    receiving_process();
    /*ģ���޸�PIDĿ��ֵ*/
    if (KEY_Scan(KEY1_GPIO_PORT, KEY1_GPIO_Pin) == KEY_ON)
    {
      if (run_i % 2 == 0)
      {
        set_point = 200;
        pid.target_val = set_point;
      }
      else
      {
        set_point = 0;
        pid.target_val = set_point;
      }
      run_i++;
#if defined(PID_ASSISTANT_EN)
      temp = set_point;                                          // ��λ����Ҫ����������ת��һ��
      set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1); // ��ͨ�� 1 ����Ŀ��ֵ
#endif
    }

    if (KEY_Scan(KEY2_GPIO_PORT, KEY2_GPIO_Pin) == KEY_ON)
    {
      pid_status = !pid_status; // ȡ��״̬

#if defined(PID_ASSISTANT_EN)
      if (!pid_status)
      {
        set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0); // ͬ����λ����������ť״̬
      }
      else
      {
        set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0); // ͬ����λ����������ť״̬
      }
#endif
    }
  }
}

void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};

  /* Enable HSE Oscillator and activate PLL with HSE as source��ʹ�� HSE ������Ϊ PLL ��ʱ��Դ�� */
  // ѡ��Ҫ���õ�������HSE��
  oscinitstruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  // ѡ��HSE��������״̬
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
