#include "bsp_general_advance_tim.h"

TIM_HandleTypeDef General_TIM_Handle;
TIM_HandleTypeDef Advance_TIM_Handle;

static void TIM_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    General_TIM_CLK_ENABLE();
    General_TIM_GPIOx_CLK_ENABLE();
    Advance_TIM_RCC_CLK_ENABLE();
    Advance_TIM_GPIO_RCC_CLK_ENABLE();
    
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pin = General_TIM_GPIO_PIN;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(General_TIM_PORT,&GPIO_InitStructure);
    
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pin = Advance_TIM_GPIO_PIN;
    HAL_GPIO_Init(Advance_TIM_PORT,&GPIO_InitStructure);
}

static void General_TIM_MODE_Config(void)
{
    General_TIM_Handle.Instance = General_TIM;
    General_TIM_Handle.Init.Period = General_TIM_Period;
    General_TIM_Handle.Init.Prescaler = General_TIM_PSC;
    General_TIM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    General_TIM_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&General_TIM_Handle);
    
    TIM_OC_InitTypeDef TIM_OC_InitStructure;
    TIM_OC_InitStructure.OCFastMode = TIM_OCFAST_DISABLE;
    TIM_OC_InitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;
    TIM_OC_InitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OC_InitStructure.Pulse = General_TIM_Pulse;
    TIM_OC_InitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&General_TIM_Handle,&TIM_OC_InitStructure,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&General_TIM_Handle,TIM_CHANNEL_1);
}

static void Advance_TIM_MODE_Config(void)
{
    Advance_TIM_Handle.Instance = Advance_TIM;
    Advance_TIM_Handle.Init.Period = Advance_TIM_Period;
    Advance_TIM_Handle.Init.Prescaler = Advance_TIM_PSC;
    Advance_TIM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    Advance_TIM_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&Advance_TIM_Handle);
    
    TIM_IC_InitTypeDef TIM_IC_InitStructure;
    TIM_IC_InitStructure.ICFilter = 0x0;
    TIM_IC_InitStructure.ICPolarity = TIM_ICPOLARITY_RISING;
    TIM_IC_InitStructure.ICPrescaler = TIM_ICPSC_DIV1;
    TIM_IC_InitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI;
    HAL_TIM_IC_ConfigChannel(&Advance_TIM_Handle,&TIM_IC_InitStructure,TIM_CHANNEL_1);
    
    TIM_IC_InitStructure.ICFilter = 0x0;
    TIM_IC_InitStructure.ICPolarity = TIM_ICPOLARITY_FALLING;
    TIM_IC_InitStructure.ICPrescaler = TIM_ICPSC_DIV1;
    TIM_IC_InitStructure.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    HAL_TIM_IC_ConfigChannel(&Advance_TIM_Handle,&TIM_IC_InitStructure,TIM_CHANNEL_2);
    
    TIM_SlaveConfigTypeDef TIM_SlaveConfigStructure;
    TIM_SlaveConfigStructure.SlaveMode = TIM_SLAVEMODE_RESET;
    TIM_SlaveConfigStructure.InputTrigger = TIM_TS_TI1FP1;
    HAL_TIM_SlaveConfigSynchronization(&Advance_TIM_Handle,&TIM_SlaveConfigStructure);
    
    HAL_TIM_IC_Start_IT(&Advance_TIM_Handle,TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&Advance_TIM_Handle,TIM_CHANNEL_2);
}

static void TIM_NVIC_Config(void)
{
    HAL_NVIC_SetPriority(TIM1_CC_IRQn,0,3);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void General_Advance_TIM_Init(void)
{
    TIM_GPIO_Config();
    General_TIM_MODE_Config();
    Advance_TIM_MODE_Config();
    TIM_NVIC_Config();
}

