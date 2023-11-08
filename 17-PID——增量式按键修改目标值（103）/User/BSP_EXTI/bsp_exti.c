#include "bsp_exti.h"

void KEY_EXTI_Config(void)
{
    GPIO_InitTypeDef GPIO_InitDtructure;
    
    KEY1_GPIO_CLK_ENABLE();
    KEY2_GPIO_CLK_ENABLE();
    
    GPIO_InitDtructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitDtructure.Pin = KEY1_GPIO_Pin;
    GPIO_InitDtructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(KEY1_GPIO_PORT,&GPIO_InitDtructure);
    HAL_NVIC_SetPriority(KEY1_EXTI_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(KEY1_EXTI_IRQ);
    
    GPIO_InitDtructure.Pin = KEY2_GPIO_Pin;
    HAL_GPIO_Init(KEY2_GPIO_PORT,&GPIO_InitDtructure);
    HAL_NVIC_SetPriority(KEY2_EXTI_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(KEY2_EXTI_IRQ);
}
