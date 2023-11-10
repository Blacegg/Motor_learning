/***********************************************************************
 * @file bsp_basic_tim.c
 * @brief
 * @author blacegg
 * @version 1.0
 * @date 2023-08-02
 ***********************************************************************/

#include "./tim/bsp_basic_tim.h"
#include "./usart/bsp_debug_usart.h"
#include "./protocol/protocol.h"
#include "main.h"

TIM_HandleTypeDef TIM_TimeBaseStructure;

/***********************************************************************
 * @brief
 ***********************************************************************/
static void TIMx_NVIC_Configuration(void)
{
  HAL_NVIC_SetPriority(BASIC_TIM_IRQn, 2, 0);

  HAL_NVIC_EnableIRQ(BASIC_TIM_IRQn);
}

static void TIM_Mode_Config(void)
{
  // ����TIMx_CLK,x[6,7]
  BASIC_TIM_CLK_ENABLE();

  TIM_TimeBaseStructure.Instance = BASIC_TIM;

  TIM_TimeBaseStructure.Init.Period = BASIC_PERIOD_COUNT - 1;

  TIM_TimeBaseStructure.Init.Prescaler = BASIC_PRESCALER_COUNT - 1;
  TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM_TimeBaseStructure.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  HAL_TIM_Base_Init(&TIM_TimeBaseStructure);

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
