/**
 ******************************************************************************
 * @file    bsp_basic_tim.c
 * @author  STMicroelectronics
 * @version V1.0
 * @date    2015-xx-xx
 * @brief   ������ʱ����ʱ����
 ******************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:Ұ��  STM32 F103 ������
 * ��̳    :http://www.firebbs.cn
 * �Ա�    :http://firestm32.taobao.com
 *
 ******************************************************************************
 */

#include "./tim/bsp_basic_tim.h"
#include "./usart/bsp_debug_usart.h"
#include "./protocol/protocol.h"
#include "main.h"

TIM_HandleTypeDef TIM_TimeBaseStructure;
/**
 * @brief  ������ʱ�� TIMx,x[6,7]�ж����ȼ�����
 * @param  ��
 * @retval ��
 */
static void TIMx_NVIC_Configuration(void)
{
  // ������ռ���ȼ��������ȼ�
  HAL_NVIC_SetPriority(BASIC_TIM_IRQn, 2, 0);
  // �����ж���Դ
  HAL_NVIC_EnableIRQ(BASIC_TIM_IRQn);
}

/*
 * ע�⣺TIM_TimeBaseInitTypeDef�ṹ��������5����Ա��TIM6��TIM7�ļĴ�������ֻ��
 * TIM_Prescaler��TIM_Period������ʹ��TIM6��TIM7��ʱ��ֻ���ʼ����������Ա���ɣ�
 * ����������Ա��ͨ�ö�ʱ���͸߼���ʱ������.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         ����
 * TIM_CounterMode			 TIMx,x[6,7]û�У��������У�������ʱ����
 * TIM_Period            ����
 * TIM_ClockDivision     TIMx,x[6,7]û�У���������(������ʱ��)
 * TIM_RepetitionCounter TIMx,x[1,8]����(�߼���ʱ��)
 *-----------------------------------------------------------------------------
 */
static void TIM_Mode_Config(void)
{
  // ����TIMx_CLK,x[6,7]
  BASIC_TIM_CLK_ENABLE();

  TIM_TimeBaseStructure.Instance = BASIC_TIM;
  /* �ۼ� TIM_Period�������һ�����»����ж�*/
  // ����ʱ����0������BASIC_PERIOD_COUNT - 1����ΪBASIC_PERIOD_COUNT�Σ�Ϊһ����ʱ����
  TIM_TimeBaseStructure.Init.Period = BASIC_PERIOD_COUNT - 1;
  // ��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1
  //				PCLK1 = HCLK / 2
  //				=> TIMxCLK=HCLK/2=SystemCoreClock/2*2=72MHz
  //  �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)
  TIM_TimeBaseStructure.Init.Prescaler = BASIC_PRESCALER_COUNT - 1;
  TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;       // ���ϼ���
  TIM_TimeBaseStructure.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // ʱ�ӷ�Ƶ

  // ��ʼ����ʱ��TIMx, x[2,3,4,5]
  HAL_TIM_Base_Init(&TIM_TimeBaseStructure);

  // ������ʱ�������ж�
  HAL_TIM_Base_Start_IT(&TIM_TimeBaseStructure);
}

void TIMx_Configuration(void)
{
  TIMx_NVIC_Configuration();

  TIM_Mode_Config();

#if PID_ASSISTANT_EN
  uint32_t temp = GET_BASIC_TIM_PERIOD();                    // �������ڣ���λms
  set_computer_value(SEND_PERIOD_CMD, CURVES_CH1, &temp, 1); // ��ͨ�� 1 ����Ŀ��ֵ
#endif
}

/*********************************************END OF FILE**********************/
