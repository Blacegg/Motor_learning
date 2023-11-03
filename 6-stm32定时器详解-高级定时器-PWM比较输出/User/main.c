#include "main.h"
#include "bsp_key.h"
#include "bsp_advance_tim.h"

int main(void)
{
    SystemClock_Config();
    LED_GPIO_Config();
    TIM_Init();
    Advance_TIM_Init();
    
    while(1)
    {
        if(KEY_Scan(KEY1_GPIO_PORT,KEY1_GPIO_Pin) == KEY_ON)
        {
            ADVANCE_TIM_Pulse+=100;
            if(ADVANCE_TIM_Pulse > 9000)
            {
                ADVANCE_TIM_Pulse = (9000 - 1);
            }
            __HAL_TIM_SET_COMPARE(&ADVANCE_TIM_Handle,TIM_CHANNEL_1,ADVANCE_TIM_Pulse);
        }
        
        if(KEY_Scan(KEY2_GPIO_PORT,KEY2_GPIO_Pin) == KEY_ON)
        {
            ADVANCE_TIM_Pulse+=100;
            if(ADVANCE_TIM_Pulse < 1000)
            {
                ADVANCE_TIM_Pulse = (1000 - 1);
            }
            __HAL_TIM_SET_COMPARE(&ADVANCE_TIM_Handle,TIM_CHANNEL_1,ADVANCE_TIM_Pulse);
        }
    }
}


void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}
