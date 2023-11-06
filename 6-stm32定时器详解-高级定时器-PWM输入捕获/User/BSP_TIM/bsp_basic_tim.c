#include "bsp_basic_tim.h"

TIM_HandleTypeDef TIM_Handle;

static void TIMx_NVIC_Config(void)
{
    HAL_NVIC_SetPriority(BASIC_TIM_IRQn,0,3);
    HAL_NVIC_EnableIRQ(BASIC_TIM_IRQn);
}

static void TIMx_MODE_Config(void)
{
    BASIC_TIM_CLK_ENABLE();
    
    TIM_Handle.Instance = BASIC_TIM;
    TIM_Handle.Init.Period = (500-1);
    TIM_Handle.Init.Prescaler = (72000-1);
    
    HAL_TIM_Base_Init(&TIM_Handle);
    HAL_TIM_Base_Start_IT(&TIM_Handle);
}

void TIM_Init(void)
{
    TIMx_NVIC_Config();
    TIMx_MODE_Config();
}

