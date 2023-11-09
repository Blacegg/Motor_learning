 /***********************************************************************
  * @file main.c
  * @brief 
  * @author blacegg
  * @version 1.0
  * @date 2023-08-01
  ***********************************************************************/
#include "bsp_usart.h"
#include "stdio.h"
#include <stdlib.h>
#include "bsp_motor_control.h"
#include "bsp_key.h"
#include "bsp_encode.h"

/* �����ת���� */
__IO int8_t Motor_Direction = 0;
/* ��ǰʱ���ܼ���ֵ */
__IO int32_t Capture_Count = 0;
/* ��һʱ���ܼ���ֵ */
__IO int32_t Last_Count = 0;
/* ���ת��ת�� */
__IO float Shaft_Speed = 0.0f;
__IO uint16_t ChannelPulse = PWM_MAX_PERIOD_COUNT /2;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  USART_Config();
  /* ����1msʱ��ΪSysTick */
  HAL_InitTick(5);
  KEY_GPIO_Config();

  printf("\r\n��������������������Ұ����ٵ��������������ʾ���򡪡�����������������\r\n");
  motor_init();
  set_motor_speed(ChannelPulse);
  Encoder_Init();

  while (1)
  {
    // /* ɨ��KEY1 */
    // if (KEY_Scan(KEY1_GPIO_PORT, KEY1_GPIO_Pin) == KEY_ON)
    // {
    //   set_motor_enable();
    // }

    //    /* ɨ��KEY2 */
    //    if (KEY_Scan(KEY2_GPIO_PORT, KEY2_GPIO_Pin) == KEY_ON)
    //    {
    //      set_motor_disable();
    //    }

    /* ɨ��KEY3 */
    if (KEY_Scan(KEY1_GPIO_PORT, KEY1_GPIO_Pin) == KEY_ON)
    {
      ChannelPulse += PWM_MAX_PERIOD_COUNT / 10;

      if (ChannelPulse > PWM_MAX_PERIOD_COUNT)
        ChannelPulse = PWM_MAX_PERIOD_COUNT;

      set_motor_speed(ChannelPulse);
    }

    /* ɨ��KEY4 */
    if (KEY_Scan(KEY2_GPIO_PORT, KEY2_GPIO_Pin) == KEY_ON)
    {
      if (ChannelPulse < PWM_MAX_PERIOD_COUNT / 10)
        ChannelPulse = 0;
      else
        ChannelPulse -= PWM_MAX_PERIOD_COUNT / 10;

      set_motor_speed(ChannelPulse);
    }
  }
}

/**
 * @brief  SysTick�жϻص�����
 * @param  ��
 * @retval ��
 */
void HAL_SYSTICK_Callback(void)
{
  static uint16_t i = 0;

  i++;
  if (i == 100) /* 100ms����һ�� */
  {
    /* �����ת���� = �������������� */
    Motor_Direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&TIM_EncoderHandle);

    /* ��ǰʱ���ܼ���ֵ = ������ֵ + ����������� * ENCODER_TIM_PERIOD  */
    Capture_Count = __HAL_TIM_GET_COUNTER(&TIM_EncoderHandle) + (Encoder_Overflow_Count * ENCODER_TIM_PERIOD);

    /* ת��ת�� = ��λʱ���ڵļ���ֵ / �������ֱܷ��� * ʱ��ϵ��  */
    Shaft_Speed = (float)(Capture_Count - Last_Count) / ENCODER_TOTAL_RESOLUTION * 10;

    printf("\n�������%d\r\n", Motor_Direction);
    printf("��λʱ������Ч����ֵ��%d\r\n", Capture_Count - Last_Count); /* ��λʱ�����ֵ = ��ǰʱ���ܼ���ֵ - ��һʱ���ܼ���ֵ */
    printf("���ת�ᴦת�٣�%.2f ת/�� \r\n", Shaft_Speed);
    printf("��������ת�٣�%.2f ת/�� \r\n", Shaft_Speed / REDUCTION_RATIO); /* �����ת�� = ת��ת�� / ���ٱ� */
    printf("��ǰռ�ձȣ�%.2f%%\n", (float)ChannelPulse *100/ PWM_PERIOD_COUNT);

    /* ��¼��ǰ�ܼ���ֵ������һʱ�̼���ʹ�� */
    Last_Count = Capture_Count;
    i = 0;
  }
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 72000000
 *            HCLK(Hz)                       = 72000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 2
 *            APB2 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            HSE PREDIV1                    = 2
 *            PLLMUL                         = 9
 *            Flash Latency(WS)              = 0
 * @param  None
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  oscinitstruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&oscinitstruct) != HAL_OK)
  {
    /* Initialization Error */
    while (1)
      ;
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2) != HAL_OK)
  {
    /* Initialization Error */
    while (1)
      ;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
