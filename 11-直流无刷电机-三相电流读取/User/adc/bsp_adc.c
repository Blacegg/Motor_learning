/**
  ******************************************************************************
  * @file    bsp_adc.c
  * @author  long
  * @version V1.0
  * @date    2023-01-13
  * @brief   NTC����
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 F103 ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
#include "./adc/bsp_adc.h"
#include ".\motor_control\bsp_motor_control.h"
#include "./led/bsp_led.h" 
#include "./usart/bsp_debug_usart.h"
#include <math.h>

DMA_HandleTypeDef DMA_Init_Handle;
ADC_HandleTypeDef ADC_Handle;

static int16_t adc_buff[ADC_NUM_MAX];    // ��ѹ�ɼ�������
static uint32_t adc_mean_sum_u = 0;        // ƽ��ֵ�ۼ�
static uint32_t adc_mean_sum_v = 0;        // ƽ��ֵ�ۼ�
static uint32_t adc_mean_sum_w = 0;        // ƽ��ֵ�ۼ�
static uint32_t adc_mean_count_u = 0;      // �ۼӼ���
static uint32_t adc_mean_count_v = 0;      // �ۼӼ���
static uint32_t adc_mean_count_w = 0;      // �ۼӼ���

/**
  * @brief  ADC ͨ�����ų�ʼ��
  * @param  ��
  * @retval ��
  */
static void ADC_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // ʹ�� GPIO ʱ��
		CURR_U_ADC_GPIO_CLK_ENABLE();
		CURR_V_ADC_GPIO_CLK_ENABLE();
		CURR_W_ADC_GPIO_CLK_ENABLE();
    // ���� IO
		GPIO_InitStructure.Pin = CURR_U_ADC_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
    GPIO_InitStructure.Pull = GPIO_NOPULL ; //������������		
    HAL_GPIO_Init(CURR_U_ADC_GPIO_PORT, &GPIO_InitStructure);	
	
		GPIO_InitStructure.Pin = CURR_V_ADC_GPIO_PIN;
    HAL_GPIO_Init(CURR_V_ADC_GPIO_PORT, &GPIO_InitStructure);
		
		GPIO_InitStructure.Pin = CURR_W_ADC_GPIO_PIN;
    HAL_GPIO_Init(CURR_W_ADC_GPIO_PORT, &GPIO_InitStructure);
}

void adc_dma_init(void)
{
    // ------------------DMA Init �ṹ����� ��ʼ��--------------------------
    // ADC1ʹ��DMA1��ͨ��1��������ֲ�̶�����
    // ����DMAʱ��
    CURR_ADC_DMA_CLK_ENABLE();
    // ���ݴ���ͨ��
    DMA_Init_Handle.Instance = CURR_ADC_DMA_STREAM;
    // ���ݴ��䷽��Ϊ���赽�洢��	
    DMA_Init_Handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    // ����Ĵ���ֻ��һ������ַ���õ���
    DMA_Init_Handle.Init.PeriphInc = DMA_PINC_DISABLE;
    // �洢����ַ�̶�
    DMA_Init_Handle.Init.MemInc = DMA_MINC_ENABLE;
    // �������ݴ�СΪ���֣��������ֽ�
    DMA_Init_Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    //	�洢�����ݴ�СҲΪ���֣����������ݴ�С��ͬ
    DMA_Init_Handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;	
    // ѭ������ģʽ
    DMA_Init_Handle.Init.Mode = DMA_CIRCULAR;
    // DMA ����ͨ�����ȼ�Ϊ�ߣ���ʹ��һ��DMAͨ��ʱ�����ȼ����ò�Ӱ��
    DMA_Init_Handle.Init.Priority = DMA_PRIORITY_HIGH;
    //��ʼ��DMA
    HAL_DMA_Init(&DMA_Init_Handle); 

    __HAL_LINKDMA(&ADC_Handle,DMA_Handle,DMA_Init_Handle);
}

/**
  * @brief  ADC �� DMA ��ʼ��
  * @param  ��
  * @retval ��
  */
static void ADC_Mode_Config(void)
{
    // ����ADCʱ��
    CURR_ADC_CLK_ENABLE();

    // -------------------ADC Init �ṹ�� ���� ��ʼ��------------------------
    // ADC1
    ADC_Handle.Instance = CURR_ADC;
    // ��ֹɨ��ģʽ����ͨ���ɼ�����Ҫ	
    ADC_Handle.Init.ScanConvMode = ENABLE; 
    // ����ת��	
    ADC_Handle.Init.ContinuousConvMode = ENABLE;
    // ������ת��	
    ADC_Handle.Init.DiscontinuousConvMode = DISABLE;
    // ������ת������
    ADC_Handle.Init.NbrOfDiscConversion   = 0;
    //ʹ���������
    ADC_Handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    //�����Ҷ���
    ADC_Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    //ת��ͨ�� 2��
    ADC_Handle.Init.NbrOfConversion = 3;
    // ��ʼ��ADC	                          
    HAL_ADC_Init(&ADC_Handle);
		
    //---------------------------------------------------------------------------
    ADC_ChannelConfTypeDef ADC_Config;
    
    ADC_Config.Channel      = CURR_U_ADC_CHANNEL;
    ADC_Config.Rank         = 1;
    // ����ʱ����	
    ADC_Config.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);
		
		ADC_Config.Channel      = CURR_V_ADC_CHANNEL;
    ADC_Config.Rank         = 2;
    // ����ʱ����	
    ADC_Config.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
		HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);
		
		ADC_Config.Channel      = CURR_W_ADC_CHANNEL;
    ADC_Config.Rank         = 3;
    // ����ʱ����	
    ADC_Config.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config) != HAL_OK)
    {
      while(1);
    }
    
    HAL_ADCEx_Calibration_Start(&ADC_Handle);     // У׼ ADC
    
    // �����ж����ȼ����ú�ʹ���ж�����
    HAL_NVIC_SetPriority(ADC_DMA_IRQ, 1, 1);
    HAL_NVIC_EnableIRQ(ADC_DMA_IRQ);

    HAL_ADC_Start_DMA(&ADC_Handle, (uint32_t*)&adc_buff, ADC_NUM_MAX);
}

/**
  * @brief  �����ɼ���ʼ��
  * @param  ��
  * @retval ��
  */
void ADC_Init(void)
{
	ADC_GPIO_Config();
  adc_dma_init();
	ADC_Mode_Config();
}

/**
  * @brief  ����ת���ڷ�����ģʽ����ɻص�
  * @param  hadc: ADC  ���.
  * @retval ��
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  uint32_t adc_mean = 0;

  HAL_ADC_Stop_DMA(hadc);       // ֹͣ ADC ������������һ�������ڼ�������
  
	/* �������ͨ��������ƽ��ֵ */
  for(uint32_t count = 0; count < ADC_NUM_MAX; count+=5)
  {
    adc_mean += (uint32_t)adc_buff[count];
  }
  
  adc_mean_sum_u += adc_mean / (ADC_NUM_MAX / 5);    // �ۼӵ�ѹ
  adc_mean_count_u++;
	adc_mean = 0;
	
	/* �������ͨ��������ƽ��ֵ */
  for(uint32_t count = 1; count < ADC_NUM_MAX; count+=5)
  {
    adc_mean += (uint32_t)adc_buff[count];
  }
  
  adc_mean_sum_v += adc_mean / (ADC_NUM_MAX / 5);    // �ۼӵ�ѹ
  adc_mean_count_v++;
	adc_mean = 0;
	
	/* �������ͨ��������ƽ��ֵ */
  for(uint32_t count = 2; count < ADC_NUM_MAX; count+=5)
  {
    adc_mean += (uint32_t)adc_buff[count];
  }
  
  adc_mean_sum_w += adc_mean / (ADC_NUM_MAX / 5);    // �ۼӵ�ѹ
  adc_mean_count_w++;
	adc_mean = 0;
 
  HAL_ADC_Start_DMA(&ADC_Handle, (uint32_t*)&adc_buff, ADC_NUM_MAX);    // ��ʼ ADC ����
}

/**
  * @brief  ��ȡU��ĵ���ֵ
  * @param  ��
  * @retval ת���õ��ĵ���ֵ
  */
int32_t get_curr_val_u(void)
{
  static uint8_t flag = 0;
  static uint32_t adc_offset = 0;    // ƫ�õ�ѹ
  int16_t curr_adc_mean = 0;         // ���� ACD �������ƽ��ֵ
  
  curr_adc_mean = adc_mean_sum_u / adc_mean_count_u;    // ����ƽ��ֵ
  

    adc_mean_count_u = 0;
    adc_mean_sum_u = 0;
    
    if (flag < 17)
    {
      adc_offset = curr_adc_mean;    // ��μ�¼ƫ�õ�ѹ����ϵͳ�ȶ�ƫ�õ�ѹ��Ϊ��Чֵ
      flag += 1;
    }
    if(curr_adc_mean>=adc_offset)
	{
		curr_adc_mean -= adc_offset;                     // ��ȥƫ�õ�ѹ
	}else
	{
		curr_adc_mean=0;
	}

  float vdc = GET_ADC_VDC_VAL(curr_adc_mean);      // ��ȡ��ѹֵ
  
  return GET_ADC_CURR_VAL(vdc);
}

/**
  * @brief  ��ȡV��ĵ���ֵ
  * @param  ��
  * @retval ת���õ��ĵ���ֵ
  */
int32_t get_curr_val_v(void)
{
  static uint8_t flag = 0;
  static uint32_t adc_offset = 0;    // ƫ�õ�ѹ
  int16_t curr_adc_mean = 0;         // ���� ACD �������ƽ��ֵ
  
  curr_adc_mean = adc_mean_sum_v / adc_mean_count_v;    // ����ƽ��ֵ
  

    adc_mean_count_v = 0;
    adc_mean_sum_v = 0;
    
    if (flag < 17)
    {
      adc_offset = curr_adc_mean;    // ��μ�¼ƫ�õ�ѹ����ϵͳ�ȶ�ƫ�õ�ѹ��Ϊ��Чֵ
      flag += 1;
    }
    if(curr_adc_mean>=adc_offset)
	{
		curr_adc_mean -= adc_offset;                     // ��ȥƫ�õ�ѹ
	}else
	{
		curr_adc_mean=0;
	}

  float vdc = GET_ADC_VDC_VAL(curr_adc_mean);      // ��ȡ��ѹֵ
  
  return GET_ADC_CURR_VAL(vdc);
}

/**
  * @brief  ��ȡW��ĵ���ֵ
  * @param  ��
  * @retval ת���õ��ĵ���ֵ
  */
int32_t get_curr_val_w(void)
{
  static uint8_t flag = 0;
  static uint32_t adc_offset = 0;    // ƫ�õ�ѹ
  int16_t curr_adc_mean = 0;         // ���� ACD �������ƽ��ֵ
  
  curr_adc_mean = adc_mean_sum_w / adc_mean_count_w;    // ����ƽ��ֵ
  

    adc_mean_count_w = 0;
    adc_mean_sum_w = 0;
    
    if (flag < 17)
    {
      adc_offset = curr_adc_mean;    // ��μ�¼ƫ�õ�ѹ����ϵͳ�ȶ�ƫ�õ�ѹ��Ϊ��Чֵ
      flag += 1;
    }
    if(curr_adc_mean>=adc_offset)
	{
		curr_adc_mean -= adc_offset;                     // ��ȥƫ�õ�ѹ
	}else
	{
		curr_adc_mean=0;
	}

  float vdc = GET_ADC_VDC_VAL(curr_adc_mean);      // ��ȡ��ѹֵ
  
  return GET_ADC_CURR_VAL(vdc);
}

/*********************************** END OF FILE *********************************************/
