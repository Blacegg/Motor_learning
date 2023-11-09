/**
 ******************************************************************************
 * @file    bsp_motor_tim.c
 * @author  STMicroelectronics
 * @version V1.0
 * @date    2015-xx-xx
 * @brief   PWM�������
 ******************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:Ұ�� STM32 F103 ������
 * ��̳    :http://www.firebbs.cn
 * �Ա�    :http://firestm32.taobao.com
 *
 ******************************************************************************
 */

#include "./tim/bsp_motor_tim.h"

void TIM_SetTIMxCompare(TIM_TypeDef *TIMx, uint32_t channel, uint32_t compare);
void TIM_SetPWM_period(TIM_TypeDef *TIMx, uint32_t TIM_period);

/**
 * @brief  ����TIM�������PWMʱ�õ���I/O
 * @param  ��
 * @retval ��
 */
static void TIMx_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* ��ʱ��ͨ���������Ŷ˿�ʱ��ʹ�� */

  PWM_TIM_CH1_GPIO_CLK();
  PWM_TIM_CH2_GPIO_CLK();

  /* ��ʱ��ͨ��1��������IO��ʼ�� */
  /*�����������*/
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  /*������������ */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  /*���ø���*/
  PWM_TIM_GPIO_AF_ENALBE();

  /*ѡ��Ҫ���Ƶ�GPIO����*/
  GPIO_InitStruct.Pin = PWM_TIM_CH1_PIN;
  /*���ÿ⺯����ʹ���������õ�GPIO_InitStructure��ʼ��GPIO*/
  HAL_GPIO_Init(PWM_TIM_CH1_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PWM_TIM_CH2_PIN;
  HAL_GPIO_Init(PWM_TIM_CH2_GPIO_PORT, &GPIO_InitStruct);
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
TIM_HandleTypeDef DCM_TimeBaseStructure;
static void TIM_PWMOUTPUT_Config(void)
{
  TIM_OC_InitTypeDef TIM_OCInitStructure;

  /*ʹ�ܶ�ʱ��*/
  PWM_TIM_CLK_ENABLE();

  DCM_TimeBaseStructure.Instance = PWM_TIM;
  /* �ۼ� TIM_Period�������һ�����»����ж�*/
  // ����ʱ����0������PWM_PERIOD_COUNT - 1����ΪPWM_PERIOD_COUNT�Σ�Ϊһ����ʱ����
  DCM_TimeBaseStructure.Init.Period = PWM_PERIOD_COUNT - 1;
  // ͨ�ÿ��ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK/2=84MHz
  // �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(PWM_PRESCALER_COUNT+1)
  DCM_TimeBaseStructure.Init.Prescaler = PWM_PRESCALER_COUNT - 1;

  /*������ʽ*/
  DCM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;
  /*����ʱ�ӷ�Ƶ*/
  DCM_TimeBaseStructure.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  /*��ʼ����ʱ��*/
  HAL_TIM_PWM_Init(&DCM_TimeBaseStructure);

  /*PWMģʽ����*/
  TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
  TIM_OCInitStructure.Pulse = 500;
  TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
  TIM_OCInitStructure.OCNPolarity = TIM_OCPOLARITY_HIGH;
  TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
  TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;

  /*����PWMͨ��*/
  HAL_TIM_PWM_ConfigChannel(&DCM_TimeBaseStructure, &TIM_OCInitStructure, PWM_CHANNEL_1);
  /*��ʼ���PWM*/
  HAL_TIM_PWM_Start(&DCM_TimeBaseStructure, PWM_CHANNEL_1);

  /*��������*/
  TIM_OCInitStructure.Pulse = 0; // Ĭ��ռ�ձ�Ϊ50%
                                 /*����PWMͨ��*/
  HAL_TIM_PWM_ConfigChannel(&DCM_TimeBaseStructure, &TIM_OCInitStructure, PWM_CHANNEL_2);
  /*��ʼ���PWM*/
  HAL_TIM_PWM_Start(&DCM_TimeBaseStructure, PWM_CHANNEL_2);
}

/**
 * @brief  ����TIMͨ����ռ�ձ�
 * @param  channel		ͨ��	��1,2,3,4��
 * @param  compare		ռ�ձ�
 *	@note 	��
 * @retval ��
 */
void TIM1_SetPWM_pulse(uint32_t channel, int compare)
{
  switch (channel)
  {
  case TIM_CHANNEL_1:
    __HAL_TIM_SET_COMPARE(&DCM_TimeBaseStructure, TIM_CHANNEL_1, compare);
    break;
  case TIM_CHANNEL_2:
    __HAL_TIM_SET_COMPARE(&DCM_TimeBaseStructure, TIM_CHANNEL_2, compare);
    break;
  case TIM_CHANNEL_3:
    __HAL_TIM_SET_COMPARE(&DCM_TimeBaseStructure, TIM_CHANNEL_3, compare);
    break;
  case TIM_CHANNEL_4:
    __HAL_TIM_SET_COMPARE(&DCM_TimeBaseStructure, TIM_CHANNEL_4, compare);
    break;
  }
}

/**
 * @brief  ��ʼ������ͨ�ö�ʱ��
 * @param  ��
 * @retval ��
 */
void Motor_TIMx_Configuration(void)
{
  TIMx_GPIO_Config();

  TIM_PWMOUTPUT_Config();
}

/*********************************************END OF FILE**********************/
