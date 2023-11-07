#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "stm32f1xx.h"

// ADC ��ź궨��
#define CURR_ADC ADC1
#define CURR_ADC_CLK_ENABLE() __HAL_RCC_ADC1_CLK_ENABLE()

#define ADC_VBUS_IRQ ADC_IRQn
#define ADC_VBUS_IRQHandler ADC_IRQHandler

#define VREF 3.3f       // �ο���ѹ����������3.3
#define ADC_NUM_MAX 320 // ADC ת��������������ֵ

#define GET_ADC_VDC_VAL(val) ((float)val / (float)4096.0f * VREF) // �õ���ѹֵ

/*********************** �����ɼ� ******************/
// ADC GPIO �궨��
#define CURR_U_ADC_GPIO_PORT GPIOC
#define CURR_U_ADC_GPIO_PIN GPIO_PIN_0
#define CURR_U_ADC_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()

#define CURR_U_ADC_CHANNEL ADC_CHANNEL_10
// ADC GPIO �궨��
#define CURR_V_ADC_GPIO_PORT GPIOC
#define CURR_V_ADC_GPIO_PIN GPIO_PIN_1
#define CURR_V_ADC_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()

#define CURR_V_ADC_CHANNEL ADC_CHANNEL_11
// ADC GPIO �궨��
#define CURR_W_ADC_GPIO_PORT GPIOC
#define CURR_W_ADC_GPIO_PIN GPIO_PIN_2
#define CURR_W_ADC_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()

#define CURR_W_ADC_CHANNEL ADC_CHANNEL_12

// ADC DMA ͨ���궨�壬��������ʹ��DMA����
#define CURR_ADC_DMA_CLK_ENABLE() __HAL_RCC_DMA1_CLK_ENABLE()
#define CURR_ADC_DMA_STREAM DMA1_Channel1

#define ADC_DMA_IRQ DMA1_Channel1_IRQn
#define ADC_DMA_IRQ_Handler DMA1_Channel1_IRQHandler
#define GET_ADC_CURR_VAL(val) (((float)val) / (float)6.0 / (float)0.02 * (float)1000.0) // �õ�����ֵ����ѹ�Ŵ�8����0.02�ǲ������裬��λmA��

extern DMA_HandleTypeDef DMA_Init_Handle;
extern ADC_HandleTypeDef ADC_Handle;

void ADC_Init(void);          // ADC�ĳ�ʼ��
int32_t get_curr_val_v(void); // ��ȡV��ĵ���ֵ
int32_t get_curr_val_u(void); // ��ȡU��ĵ���ֵ
int32_t get_curr_val_w(void); // ��ȡW��ĵ���ֵ

#endif /* __BSP_ADC_H */
