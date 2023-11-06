/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   stepper-PWM����������ת
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
#include "./stepper/bsp_stepper_init.h"
#include "./usart/bsp_debug_usart.h"
#include "./led/bsp_led.h"
#include "./key/bsp_key.h" 


/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void) 
{
  int i=0,j=0;
  int dir_val=0;
  int en_val=0;
  /* ��ʼ��ϵͳʱ��Ϊ72MHz */
  SystemClock_Config();
	/* �������üĴ���ʱ�� */
	__HAL_RCC_SYSCFG_CLK_ENABLE();
  /*��ʼ��USART ����ģʽΪ 115200 8-N-1���жϽ���*/
  DEBUG_USART_Config();
  printf("��ӭʹ��Ұ�� ��������� ������� PWM������ת ����\r\n");
  printf("���°���2���޸���ת���򣬰��°���3���޸�ʹ��\r\n");  
  printf("֧������ͨ�����л��궨�弴��\r\n");
  /*�����жϳ�ʼ��*/
  Key_GPIO_Config();  
  /*led��ʼ��*/
  LED_GPIO_Config();
  /*���������ʼ��*/
  stepper_Init();

  while(1)
  {     
    if( Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON  )
    {
      // LED2 ȡ��    
      LED2_TOGGLE;
      
      /*�ı䷽��*/
      dir_val=(++i % 2) ? CW : CCW;
      MOTOR_DIR(dir_val);
    }
    if( Key_Scan(KEY3_GPIO_PORT,KEY3_PIN) == KEY_ON  )
    {
      // LED1 ȡ��    
      LED1_TOGGLE;

      /*�ı�ʹ��*/
      en_val=(++j % 2) ? CW : CCW;
      MOTOR_EN(en_val);
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
