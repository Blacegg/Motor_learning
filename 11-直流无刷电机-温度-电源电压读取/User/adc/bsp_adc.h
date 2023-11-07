#ifndef __BSP_ADC_H
#define	__BSP_ADC_H

#include "stm32f4xx.h"

// ADC ��ź궨��
#define TEMP_ADC                        ADC3
#define TEMP_ADC_CLK_ENABLE()           __ADC3_CLK_ENABLE()

#define ADC_VBUS_IRQ                    ADC_IRQn
#define ADC_VBUS_IRQHandler             ADC_IRQHandler

#define VREF                            3.3f     // �ο���ѹ����������3.3
#define ADC_NUM_MAX                     128      // ADC ת��������������ֵ

#define GET_ADC_VDC_VAL(val)            ((float)val/4096.0f*VREF)          // �õ���ѹֵ
  
/*********************** �¶ȴ�������ѹ�ɼ� ******************/
// ADC GPIO �궨��
#define TEMP_ADC_GPIO_PORT              GPIOF
#define TEMP_ADC_GPIO_PIN               GPIO_PIN_10
#define TEMP_ADC_GPIO_CLK_ENABLE()      __GPIOF_CLK_ENABLE()

#define TEMP_ADC_CHANNEL                ADC_CHANNEL_8

// ADC DMA ͨ���궨�壬��������ʹ��DMA����
#define TEMP_ADC_DMA_CLK_ENABLE()       __DMA2_CLK_ENABLE()
#define TEMP_ADC_DMA_CHANNEL            DMA_CHANNEL_2
#define TEMP_ADC_DMA_STREAM             DMA2_Stream0

#define ADC_DMA_IRQ                     DMA2_Stream0_IRQn
#define ADC_DMA_IRQ_Handler             DMA2_Stream0_IRQHandler

/*********************** ��Դ��ѹ�ɼ� ******************/

#define VBUS_GPIO_PORT                  GPIOF
#define VBUS_GPIO_PIN                   GPIO_PIN_9
#define VBUS_GPIO_CLK_ENABLE()          __GPIOF_CLK_ENABLE()

#define VBUS_ADC_CHANNEL                ADC_CHANNEL_7

#define GET_VBUS_VAL(val)               (((float)val - 1.24f) * 37.0f)      // ��ȡ��ѹֵ��������ѹ�ǵ�Դ��ѹ��1/37��

extern DMA_HandleTypeDef DMA_Init_Handle;
extern ADC_HandleTypeDef ADC_Handle;

void ADC_Init(void);
float get_ntc_v_val(void);
float get_ntc_r_val(void);
float get_ntc_t_val(void);
float get_vbus_val(void);
 
#endif /* __BSP_ADC_H */
