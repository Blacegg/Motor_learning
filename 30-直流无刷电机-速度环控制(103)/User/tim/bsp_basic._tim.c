/***********************************************************************
 * @file bsp_basic._tim.c
 * @brief
 * @author blacegg
 * @version 1.0
 * @date 2023-08-21
 ***********************************************************************/

#include "bsp_basic_tim.h"
#include "./usart/bsp_debug_usart.h"

TIM_HandleTypeDef TIM_TimeBaseStructure;

/***********************************************************************
 * @brief Construct a new timx nvic configuration object
 ***********************************************************************/
static void TIMx_NVIC_Configuration(void)
{
    // 设置抢占优先级，子优先级
    HAL_NVIC_SetPriority(BASIC_TIM_IRQn, 1, 3);
    // 设置中断来源
    HAL_NVIC_EnableIRQ(BASIC_TIM_IRQn);
}

/***********************************************************************
 * @brief TIM_Mode_Config
 ***********************************************************************/
static void TIM_Mode_Config(void)
{
    BASIC_TIM_CLK_ENABLE();
    TIM_TimeBaseStructure.Instance = BASIC_TIM;
    TIM_TimeBaseStructure.Init.Period = BASIC_PERIOD_COUNT - 1;
    TIM_TimeBaseStructure.Init.Prescaler = BASIC_PRESCALER_COUNT - 1;
    TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;       // 向上计数
    TIM_TimeBaseStructure.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟分频

    HAL_TIM_Base_Init(&TIM_TimeBaseStructure);

    HAL_TIM_Base_Start_IT(&TIM_TimeBaseStructure);
}

/***********************************************************************
 * @brief TIMx_Configuration
 ***********************************************************************/
void TIMx_Configuration(void)
{
    TIMx_NVIC_Configuration();
    TIM_Mode_Config();

#if defined(PID_ASSISTANT_EN)

    uint32_t temp = GET_BASIC_TIM_PERIOD();
    set_computer_value(SEND_PERIOD_CMD, CURVES_CH1, &temp, 1);

#endif
}
