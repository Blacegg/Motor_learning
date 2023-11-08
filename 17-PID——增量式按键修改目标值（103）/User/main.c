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
  /* 初始化基本定时器定时，20ms产生一次中断 */
  TIMx_Configuration();
  /* PID算法参数初始化 */
  PID_param_init();
  int run_i = 0;

#if defined(PID_ASSISTANT_EN)
  int temp = set_point;                                      // 上位机需要整数参数，转换一下
  set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1); // 给通道 1 发送目标值
#endif
  while (1)
  {
    /* 接收数据处理 */
    receiving_process();
    /*模拟修改PID目标值*/
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
      temp = set_point;                                          // 上位机需要整数参数，转换一下
      set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1); // 给通道 1 发送目标值
#endif
    }

    if (KEY_Scan(KEY2_GPIO_PORT, KEY2_GPIO_Pin) == KEY_ON)
    {
      pid_status = !pid_status; // 取反状态

#if defined(PID_ASSISTANT_EN)
      if (!pid_status)
      {
        set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0); // 同步上位机的启动按钮状态
      }
      else
      {
        set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0); // 同步上位机的启动按钮状态
      }
#endif
    }
  }
}

void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};

  /* Enable HSE Oscillator and activate PLL with HSE as source（使能 HSE ，并作为 PLL 的时钟源） */
  // 选择要配置的震荡器（HSE）
  oscinitstruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  // 选择HSE振荡器的新状态
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
