#ifndef __BSP_ADC_H
#define	__BSP_ADC_H

#include "stm32f4xx.h"

// ADC ��ź궨��
#define TEMP_ADC                        ADC3
#define TEMP_ADC_CLK_ENABLE()           __ADC3_CLK_ENABLE()

#define ADC_VBUS_IRQ                    ADC_IRQn
#define ADC_VBUS_IRQHandler             ADC_IRQHandler

#define VREF                            3.3f     // �ο���ѹ����������3.3
#define ADC_NUM_MAX                     320    // ADC ת��������������ֵ

#define GET_ADC_VDC_VAL(val)            ((float)val/4096.0f*VREF)          // �õ���ѹֵ
  
/*********************** �¶ȴ�������ѹ�ɼ� ******************/
// ADC GPIO �궨��
#define TEMP_ADC_GPIO_PORT              GPIOF
#define TEMP_ADC_GPIO_PIN               GPIO_PIN_10
#define TEMP_ADC_GPIO_CLK_ENABLE()      __GPIOF_CLK_ENABLE()

#define TEMP_ADC_CHANNEL                ADC_CHANNEL_8
///*********************** �����ɼ� ******************/
// ADC GPIO �궨��
#define CURR_U_ADC_GPIO_PORT              GPIOF
#define CURR_U_ADC_GPIO_PIN               GPIO_PIN_6
#define CURR_U_ADC_GPIO_CLK_ENABLE()      __GPIOF_CLK_ENABLE()

#define CURR_U_ADC_CHANNEL                ADC_CHANNEL_4
// ADC GPIO �궨��
#define CURR_V_ADC_GPIO_PORT              GPIOF
#define CURR_V_ADC_GPIO_PIN               GPIO_PIN_7
#define CURR_V_ADC_GPIO_CLK_ENABLE()      __GPIOF_CLK_ENABLE()

#define CURR_V_ADC_CHANNEL                ADC_CHANNEL_5
// ADC GPIO �궨��
#define CURR_W_ADC_GPIO_PORT              GPIOF
#define CURR_W_ADC_GPIO_PIN               GPIO_PIN_8
#define CURR_W_ADC_GPIO_CLK_ENABLE()      __GPIOF_CLK_ENABLE()

#define CURR_W_ADC_CHANNEL                ADC_CHANNEL_6
// ADC DMA ͨ���궨�壬��������ʹ��DMA����
#define TEMP_ADC_DMA_CLK_ENABLE()       __DMA2_CLK_ENABLE()
#define TEMP_ADC_DMA_CHANNEL            DMA_CHANNEL_2
#define TEMP_ADC_DMA_STREAM             DMA2_Stream0

#define ADC_DMA_IRQ                     DMA2_Stream0_IRQn
#define ADC_DMA_IRQ_Handler             DMA2_Stream0_IRQHandler
#define GET_ADC_CURR_VAL(val)           (((float)val)/(float)8.0/(float)0.02*(float)1000.0)        // �õ�����ֵ����ѹ�Ŵ�8����0.02�ǲ������裬��λmA��

/*********************** ��Դ��ѹ�ɼ� ******************/

#define VBUS_GPIO_PORT                  GPIOF
#define VBUS_GPIO_PIN                   GPIO_PIN_9
#define VBUS_GPIO_CLK_ENABLE()          __GPIOF_CLK_ENABLE()

#define VBUS_ADC_CHANNEL                ADC_CHANNEL_7

#define GET_VBUS_VAL(val)               (((float)val - 1.24f) * 37.0f )      // ��ȡ��ѹֵ��������ѹ�ǵ�Դ��ѹ��1/37��

extern DMA_HandleTypeDef DMA_Init_Handle;
extern ADC_HandleTypeDef ADC_Handle;

int32_t get_curr_val_v(void);//��ȡV��ĵ���ֵ
int32_t get_curr_val_u(void);//��ȡU��ĵ���ֵ
int32_t get_curr_val_w(void);//��ȡW��ĵ���ֵ
void ADC_Init(void);				//ADC�ĳ�ʼ��
float get_ntc_v_val(void);	//��ȡ�¶ȴ������˵ĵ�ѹֵ	
float get_ntc_r_val(void);	//��ȡ�¶ȴ������˵ĵ���ֵ
float get_ntc_t_val(void);	//��ȡ�¶ȴ��������¶�
float get_vbus_val(void);		//��ȡ��Դ��ѹֵ
 
#endif /* __BSP_ADC_H */
