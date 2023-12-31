/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   测试led
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 F103 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "./usart/bsp_debug_usart.h"
#include "./delay/core_delay.h"
#include "./stepper/bsp_stepper_init.h"
#include "./key/bsp_key.h" 

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
  int key_val=0;
  int i=0;
  int dir_val=0;
  int angle_val=90;
  
  HAL_Init();        
  /* 配置系统时钟为72 MHz */ 
  SystemClock_Config();
		/* 开启复用寄存器时钟 */
	__HAL_RCC_SYSCFG_CLK_ENABLE();
  /*初始化USART 配置模式为 115200 8-N-1，中断接收*/
  DEBUG_USART_Config();
  printf("欢迎使用野火 电机开发板 步进电机 IO口模拟控制 例程\r\n");
  printf("按下按键2可修改旋转方向、按下按键3可修改旋转角度\r\n");
  /*按键初始化*/
  Key_GPIO_Config();
  /*步进电机初始化*/
  stepper_Init();
  /*开启步进电机使能*/
  while(1)
  {     
    if( Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON  )
    {
      /*改变方向*/
      dir_val=(++i % 2) ? CW : CCW;
      MOTOR_DIR(dir_val);
      key_val = ON;
    }
    if( Key_Scan(KEY3_GPIO_PORT,KEY3_PIN) == KEY_ON  )
    {
      /*改变角度*/
      angle_val=angle_val+90;
      key_val = ON;
    }
    if( key_val == ON  )
    {
      /*打印状态*/
      if(dir_val)
        printf("顺时针旋转 %d 度,",angle_val);
      else
        printf("逆时针旋转 %d 度,",angle_val);
      
      printf("正在运行中......\r\n");
      stepper_turn(500,angle_val,32,dir_val);
      key_val = OFF;
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
  *            HSE PREDIV1                    = 1
  *            PLLMUL                         = 9
  *            Flash Latency(WS)              = 2
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
