#include "bsp_key.h"

void KEY_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitDtructure;
    
    KEY1_GPIO_CLK_ENABLE();
    KEY2_GPIO_CLK_ENABLE();
    
    GPIO_InitDtructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitDtructure.Pin = KEY1_GPIO_Pin;
    GPIO_InitDtructure.Pull = GPIO_NOPULL;
    //GPIO_InitDtructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(KEY1_GPIO_PORT,&GPIO_InitDtructure);
    
    GPIO_InitDtructure.Pin = KEY2_GPIO_Pin;
    HAL_GPIO_Init(KEY2_GPIO_PORT,&GPIO_InitDtructure);
}

uint8_t KEY_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{
    if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==KEY_ON)
    {
        while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==KEY_ON);
        return KEY_ON;
    }
        return KEY_OFF;
}
