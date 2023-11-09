#include "./tim/bsp_motor_tim.h"

void TIM_SetTIMxCompare(TIM_TypeDef *TIMx, uint32_t channel, uint32_t compare);
void TIM_SetPWM_period(TIM_TypeDef *TIMx, uint32_t TIM_period);

static void MOTOR_TIMx_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  PWM_TIM_CH1_GPIO_CLK();
  PWM_TIM_CH2_GPIO_CLK();

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  PWM_TIM_GPIO_AF_ENALBE();

  GPIO_InitStruct.Pin = PWM_TIM_CH1_PIN;
  HAL_GPIO_Init(PWM_TIM_CH1_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PWM_TIM_CH2_PIN;
  HAL_GPIO_Init(PWM_TIM_CH2_GPIO_PORT, &GPIO_InitStruct);
}

TIM_HandleTypeDef DCM_TimeBaseStructure;
static void MOTOR_TIM_PWMOUTPUT_Config(void)
{
  TIM_OC_InitTypeDef TIM_OCInitStructure;

  PWM_TIM_CLK_ENABLE();

  DCM_TimeBaseStructure.Instance = PWM_TIM;
  DCM_TimeBaseStructure.Init.Period = PWM_PERIOD_COUNT - 1;
  DCM_TimeBaseStructure.Init.Prescaler = PWM_PRESCALER_COUNT - 1;
  DCM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;
  DCM_TimeBaseStructure.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&DCM_TimeBaseStructure);

  TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
  TIM_OCInitStructure.Pulse = 500;
  TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
  TIM_OCInitStructure.OCNPolarity = TIM_OCPOLARITY_HIGH;
  TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
  TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;

  HAL_TIM_PWM_ConfigChannel(&DCM_TimeBaseStructure, &TIM_OCInitStructure, PWM_CHANNEL_1);
  HAL_TIM_PWM_Start(&DCM_TimeBaseStructure, PWM_CHANNEL_1);

  TIM_OCInitStructure.Pulse = 0;
  HAL_TIM_PWM_ConfigChannel(&DCM_TimeBaseStructure, &TIM_OCInitStructure, PWM_CHANNEL_2);
  HAL_TIM_PWM_Start(&DCM_TimeBaseStructure, PWM_CHANNEL_2);
}

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

void MOTOR_TIMx_Configuration(void)
{
  MOTOR_TIMx_GPIO_Config();

  MOTOR_TIM_PWMOUTPUT_Config();
}

/*********************************************END OF FILE**********************/
