#include "bsp_clkconfig.h"

void HSE_SetSystemClock(void)
{
    // ���� HSE
    RCC_OscInitTypeDef RCC_OscInitStructure;
    
    RCC_OscInitStructure.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStructure.HSEState = RCC_HSE_ON;
    RCC_OscInitStructure.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    
    // ѡ�� HSE ��Ϊ PLL ��ʱ��Դ
    RCC_OscInitStructure.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStructure.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStructure.PLL.PLLMUL = RCC_PLL_MUL9;
    
    // �ȴ� HSE �ȶ�
    if(HAL_RCC_OscConfig(&RCC_OscInitStructure) != HAL_OK)
    {
        while(1);
    }
    
    // ѡ�� PLL ��Ϊϵͳʱ��Դ������HCLK,PCLK1 ��PCLK2 ��Ƶϵ��
    RCC_ClkInitTypeDef RCC_ClkInitStructure;
    
    RCC_ClkInitStructure.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 |RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStructure.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStructure.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStructure.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStructure.APB2CLKDivider = RCC_HCLK_DIV1;
    
    // �ȴ��������
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStructure, FLASH_LATENCY_2) != HAL_OK)
    {
        while(1);
    }
}


void HSI_SetSystemClock(void)
{
    // ���� HSI
    RCC_OscInitTypeDef RCC_OscInitStructure;
    
    RCC_OscInitStructure.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStructure.HSIState = RCC_HSI_ON;
    RCC_OscInitStructure.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    
    // ѡ�� HSI ��Ϊ PLL ��ʱ��Դ
    RCC_OscInitStructure.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStructure.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStructure.PLL.PLLMUL = RCC_PLL_MUL16;
    
    // �ȴ� HSI �ȶ�
    if(HAL_RCC_OscConfig(&RCC_OscInitStructure) != HAL_OK)
    {
        while(1);
    }
    
    // ѡ�� PLL ��Ϊϵͳʱ��Դ������HCLK,PCLK1 ��PCLK2 ��Ƶϵ��
    RCC_ClkInitTypeDef RCC_ClkInitStructure;
    
    RCC_ClkInitStructure.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 |RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStructure.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStructure.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStructure.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStructure.APB2CLKDivider = RCC_HCLK_DIV1;
    
    // �ȴ��������
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStructure, FLASH_LATENCY_2) != HAL_OK)
    {
        while(1);
    }
}

void Delay(__IO uint32_t nCount)
{
for (; nCount != 0; nCount--);
}

