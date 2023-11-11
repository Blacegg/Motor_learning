/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   步进电机-速度环
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
#include "./pid/bsp_pid.h"
#include "./tim/bsp_basic_tim.h"
#include "./stepper/bsp_stepper_ctrl.h"
#include "./Encoder/bsp_encoder.h"
#include "./protocol/protocol.h"

extern _pid pid;
extern int pid_status;

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
	
	/*补充：PID例程中 MOTOR_PUL_IRQn 优先级需要调为最高 */	
	/* Set Interrupt Group Priority */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);	

	/*初始化USART 配置模式为 115200 8-N-1，中断接收*/
	DEBUG_USART_Config();
	printf("欢迎使用野火 电机开发板 步进电机 速度闭环控制 位置式PID例程\r\n");
	printf("按下按键1增加目标值、按键2减少目标值\r\n");	
  printf("其他操作请使用PID调试助手\r\n");	  /* 初始化时间戳 */
  protocol_init();          /* 初始化串口通信协议 */
  HAL_InitTick(5);
	/*按键中断初始化*/
	Key_GPIO_Config();	
	/*led初始化*/
	LED_GPIO_Config();
  /* 初始化基本定时器定时，20ms产生一次中断 */
	TIMx_Configuration();
  /* 编码器接口初始化 */
	Encoder_Init();
	/*步进电机初始化*/
	stepper_Init();
  /* 上电默认停止电机 */
  Set_Stepper_Stop();
  /* PID算法参数初始化 */
  PID_param_init();
  
  /* 目标速度转换为编码器的脉冲数作为pid目标值 */
  pid.target_val = TARGET_SPEED * ENCODER_TOTAL_RESOLUTION / SAMPLING_PERIOD;
    
#if PID_ASSISTANT_EN
  int Temp = pid.target_val;    // 上位机需要整数参数，转换一下
  set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);    // 同步上位机的启动按钮状态
  set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &Temp, 1);// 给通道 1 发送目标值
#endif

	while(1)
	{
    /* 接收数据处理 */
    receiving_process();
    
    /* 扫描KEY1，启动电机 */
    if( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON  )
		{
    #if PID_ASSISTANT_EN
      Set_Stepper_Start();
      set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0);// 同步上位机的启动按钮状态
    #else
      Set_Stepper_Start();
    #endif
		}
    /* 扫描KEY2，停止电机 */
    if( Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON  )
		{
    #if PID_ASSISTANT_EN
      Set_Stepper_Stop();
      set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);// 同步上位机的启动按钮状态
    #else
      Set_Stepper_Stop();     
    #endif
		}
    /* 扫描KEY3，增大目标位置 */
    if( Key_Scan(KEY3_GPIO_PORT,KEY3_PIN) == KEY_ON  )
		{
			/* 位置增加2圈 */
      pid.target_val += 80;
      
    #if PID_ASSISTANT_EN
      int temp = pid.target_val;
      set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1);// 给通道 1 发送目标值
    #endif
		}
    /* 扫描KEY4，减小目标位置 */
    if( Key_Scan(KEY4_GPIO_PORT,KEY4_PIN) == KEY_ON  )
		{
			/* 位置减小2圈 */
      pid.target_val -= 80;
      
    #if PID_ASSISTANT_EN
      int temp = pid.target_val;
      set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1);// 给通道 1 发送目标值
    #endif
		}
	}
} 	


/**
  * @brief  定时器更新事件回调函数
  * @param  无
  * @retval 无
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* 判断触发中断的定时器 */
  if(htim->Instance == BASIC_TIM)
  {
    Stepper_Speed_Ctrl();
  }
  else if(htim->Instance == ENCODER_TIM)
  {  
    /* 判断当前计数方向 */
    if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
      /* 下溢 */
      encoder_overflow_count--;
    else
      /* 上溢 */
      encoder_overflow_count++;
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
