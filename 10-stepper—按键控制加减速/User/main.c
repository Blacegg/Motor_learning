/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   stepper-PWM控制匀速旋转
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
#include "./stepper/bsp_stepper_init.h"
#include "./usart/bsp_debug_usart.h"
#include "./led/bsp_led.h"
#include "./key/bsp_key.h" 


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
  printf("欢迎使用野火 电机开发板 步进电机 按键控制加减速 例程\r\n");
  printf("按下按键2加速，按下按键3减速\r\n");
  printf("支持两个通道，切换宏定义即可\r\n");
  /*按键中断初始化*/
  Key_GPIO_Config();  
  /*led初始化*/
  LED_GPIO_Config();
  /*步进电机初始化*/
  stepper_Init();

  while(1)
  {
    if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON  )
    {
      LED2_TOGGLE;
      
      OC_Pulse_num -= 20;
      if(OC_Pulse_num <= 20)
        OC_Pulse_num = 20;
    }
    if( Key_Scan(KEY3_GPIO_PORT, KEY3_PIN) == KEY_ON  )
    {
      LED3_TOGGLE;
      
      OC_Pulse_num += 20;
      if(OC_Pulse_num >= 400)
        OC_Pulse_num = 400;
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
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
