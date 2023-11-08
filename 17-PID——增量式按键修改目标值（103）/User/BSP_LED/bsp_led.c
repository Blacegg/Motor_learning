#include "bsp_led.h"

void LED_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitDtructure;

    LED1_GPIO_CLK_ENABLE();
    LED2_GPIO_CLK_ENABLE();
    LED3_GPIO_CLK_ENABLE();

    GPIO_InitDtructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitDtructure.Pin = LED1_PIN;
    GPIO_InitDtructure.Pull = GPIO_NOPULL;
    GPIO_InitDtructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LED1_GPIO_PORT, &GPIO_InitDtructure);

    GPIO_InitDtructure.Pin = LED2_PIN;
    HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitDtructure);

    GPIO_InitDtructure.Pin = LED3_PIN;
    HAL_GPIO_Init(LED3_GPIO_PORT, &GPIO_InitDtructure);

    LED_ALLOFF;
}
