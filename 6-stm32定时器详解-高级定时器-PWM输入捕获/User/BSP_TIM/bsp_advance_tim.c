#include "bsp_advance_tim.h"

uint16_t  ADVANCE_TIM_Pulse = (5000-1);
TIM_HandleTypeDef ADVANCE_TIM_Handle;

static void Advance_TIM_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    ADVANCED_TIM_GPIO_RCC_CLK_ENABLE();
    ADVANCED_TIM_RCC_CLK_ENABLE();
    
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pin = ADVANCE_OCPWM_PIN;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ADVANCE_OCPWM_GPIO_PORT,&GPIO_InitStructure);
    
    GPIO_InitStructure.Pin = ADVANCE_OCNPWM_PIN;
    HAL_GPIO_Init(ADVANCE_OCNPWM_GPIO_PORT,&GPIO_InitStructure);
    
    GPIO_InitStructure.Pin = ADVANCE_BRAKE_PIN;
    HAL_GPIO_Init(ADVANCE_BRAKE_GPIO_PORT,&GPIO_InitStructure);
}

static void Advance_TIM_TimeBaseConfig(void)
{
    TIM_OC_InitTypeDef TIM_OC_InitStructure;
    TIM_BreakDeadTimeConfigTypeDef TIM_BreakInitStructure;
    
    ADVANCE_TIM_Handle.Instance = ADVANCED_TIMx;
    ADVANCE_TIM_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    ADVANCE_TIM_Handle.Init.CounterMode =TIM_COUNTERMODE_UP;
    ADVANCE_TIM_Handle.Init.Period = ADVANCE_TIM_Period;
    ADVANCE_TIM_Handle.Init.Prescaler = ADVANCE_TIM_PSC;
    HAL_TIM_Base_Init(&ADVANCE_TIM_Handle);
    
    TIM_OC_InitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OC_InitStructure.Pulse = ADVANCE_TIM_Pulse;
    TIM_OC_InitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStructure.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    TIM_OC_InitStructure.OCFastMode = TIM_OCFAST_DISABLE;
    TIM_OC_InitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;
    TIM_OC_InitStructure.OCNIdleState = TIM_OCNIDLESTATE_SET;
    HAL_TIM_PWM_ConfigChannel(&ADVANCE_TIM_Handle,&TIM_OC_InitStructure,TIM_CHANNEL_1);
    
    TIM_BreakInitStructure.OffStateRunMode = TIM_OSSR_ENABLE;
    TIM_BreakInitStructure.OffStateIDLEMode = TIM_OSSI_ENABLE;
    TIM_BreakInitStructure.LockLevel = TIM_LOCKLEVEL_1;
    TIM_BreakInitStructure.DeadTime = 11;
    TIM_BreakInitStructure.BreakState = TIM_BREAK_ENABLE;
    TIM_BreakInitStructure.BreakPolarity = TIM_BREAKPOLARITY_LOW;
    TIM_BreakInitStructure.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&ADVANCE_TIM_Handle,&TIM_BreakInitStructure);
    
    HAL_TIM_PWM_Start(&ADVANCE_TIM_Handle,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&ADVANCE_TIM_Handle,TIM_CHANNEL_1);
}

void Advance_TIM_Init(void)
{
    Advance_TIM_GPIO_Config();
    Advance_TIM_TimeBaseConfig();
}
