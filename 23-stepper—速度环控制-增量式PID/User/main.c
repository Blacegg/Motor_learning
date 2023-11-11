/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   �������-�ٶȻ�
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
#include "./pid/bsp_pid.h"
#include "./tim/bsp_basic_tim.h"
#include "./stepper/bsp_stepper_ctrl.h"
#include "./Encoder/bsp_encoder.h"
#include "./protocol/protocol.h"

extern _pid pid;
extern int pid_status;

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
	
	/*���䣺PID������ MOTOR_PUL_IRQn ���ȼ���Ҫ��Ϊ��� */	
	/* Set Interrupt Group Priority */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);	

	/*��ʼ��USART ����ģʽΪ 115200 8-N-1���жϽ���*/
	DEBUG_USART_Config();
	printf("��ӭʹ��Ұ�� ��������� ������� �ٶȱջ����� λ��ʽPID����\r\n");
	printf("���°���1����Ŀ��ֵ������2����Ŀ��ֵ\r\n");	
  printf("����������ʹ��PID��������\r\n");	  /* ��ʼ��ʱ��� */
  protocol_init();          /* ��ʼ������ͨ��Э�� */
  HAL_InitTick(5);
	/*�����жϳ�ʼ��*/
	Key_GPIO_Config();	
	/*led��ʼ��*/
	LED_GPIO_Config();
  /* ��ʼ��������ʱ����ʱ��20ms����һ���ж� */
	TIMx_Configuration();
  /* �������ӿڳ�ʼ�� */
	Encoder_Init();
	/*���������ʼ��*/
	stepper_Init();
  /* �ϵ�Ĭ��ֹͣ��� */
  Set_Stepper_Stop();
  /* PID�㷨������ʼ�� */
  PID_param_init();
  
  /* Ŀ���ٶ�ת��Ϊ����������������ΪpidĿ��ֵ */
  pid.target_val = TARGET_SPEED * ENCODER_TOTAL_RESOLUTION / SAMPLING_PERIOD;
    
#if PID_ASSISTANT_EN
  int Temp = pid.target_val;    // ��λ����Ҫ����������ת��һ��
  set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);    // ͬ����λ����������ť״̬
  set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &Temp, 1);// ��ͨ�� 1 ����Ŀ��ֵ
#endif

	while(1)
	{
    /* �������ݴ��� */
    receiving_process();
    
    /* ɨ��KEY1��������� */
    if( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON  )
		{
    #if PID_ASSISTANT_EN
      Set_Stepper_Start();
      set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0);// ͬ����λ����������ť״̬
    #else
      Set_Stepper_Start();
    #endif
		}
    /* ɨ��KEY2��ֹͣ��� */
    if( Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON  )
		{
    #if PID_ASSISTANT_EN
      Set_Stepper_Stop();
      set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);// ͬ����λ����������ť״̬
    #else
      Set_Stepper_Stop();     
    #endif
		}
    /* ɨ��KEY3������Ŀ��λ�� */
    if( Key_Scan(KEY3_GPIO_PORT,KEY3_PIN) == KEY_ON  )
		{
			/* λ������2Ȧ */
      pid.target_val += 80;
      
    #if PID_ASSISTANT_EN
      int temp = pid.target_val;
      set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1);// ��ͨ�� 1 ����Ŀ��ֵ
    #endif
		}
    /* ɨ��KEY4����СĿ��λ�� */
    if( Key_Scan(KEY4_GPIO_PORT,KEY4_PIN) == KEY_ON  )
		{
			/* λ�ü�С2Ȧ */
      pid.target_val -= 80;
      
    #if PID_ASSISTANT_EN
      int temp = pid.target_val;
      set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1);// ��ͨ�� 1 ����Ŀ��ֵ
    #endif
		}
	}
} 	


/**
  * @brief  ��ʱ�������¼��ص�����
  * @param  ��
  * @retval ��
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* �жϴ����жϵĶ�ʱ�� */
  if(htim->Instance == BASIC_TIM)
  {
    Stepper_Speed_Ctrl();
  }
  else if(htim->Instance == ENCODER_TIM)
  {  
    /* �жϵ�ǰ�������� */
    if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
      /* ���� */
      encoder_overflow_count--;
    else
      /* ���� */
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
