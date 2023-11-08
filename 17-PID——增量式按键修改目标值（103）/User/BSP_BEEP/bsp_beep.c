#include "bsp_beep.h"

void BEEP_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitDtructure;
    
    BEEP_GPIO_CLK_ENABLE();
    
    GPIO_InitDtructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitDtructure.Pin = BEEP_GPIO_Pin;
    GPIO_InitDtructure.Pull = GPIO_NOPULL;
    GPIO_InitDtructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BEEP_GPIO_PORT,&GPIO_InitDtructure);
}

