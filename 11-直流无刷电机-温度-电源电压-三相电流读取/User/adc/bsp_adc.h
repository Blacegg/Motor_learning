#ifndef __BSP_ADC_H
#define	__BSP_ADC_H

#include "stm32f4xx.h"

// ADC 序号宏定义
#define TEMP_ADC                        ADC3
#define TEMP_ADC_CLK_ENABLE()           __ADC3_CLK_ENABLE()

#define ADC_VBUS_IRQ                    ADC_IRQn
#define ADC_VBUS_IRQHandler             ADC_IRQHandler

#define VREF                            3.3f     // 参考电压，理论上是3.3
#define ADC_NUM_MAX                     320    // ADC 转换结果缓冲区最大值

#define GET_ADC_VDC_VAL(val)            ((float)val/4096.0f*VREF)          // 得到电压值
  
/*********************** 温度传感器电压采集 ******************/
// ADC GPIO 宏定义
#define TEMP_ADC_GPIO_PORT              GPIOF
#define TEMP_ADC_GPIO_PIN               GPIO_PIN_10
#define TEMP_ADC_GPIO_CLK_ENABLE()      __GPIOF_CLK_ENABLE()

#define TEMP_ADC_CHANNEL                ADC_CHANNEL_8
///*********************** 电流采集 ******************/
// ADC GPIO 宏定义
#define CURR_U_ADC_GPIO_PORT              GPIOF
#define CURR_U_ADC_GPIO_PIN               GPIO_PIN_6
#define CURR_U_ADC_GPIO_CLK_ENABLE()      __GPIOF_CLK_ENABLE()

#define CURR_U_ADC_CHANNEL                ADC_CHANNEL_4
// ADC GPIO 宏定义
#define CURR_V_ADC_GPIO_PORT              GPIOF
#define CURR_V_ADC_GPIO_PIN               GPIO_PIN_7
#define CURR_V_ADC_GPIO_CLK_ENABLE()      __GPIOF_CLK_ENABLE()

#define CURR_V_ADC_CHANNEL                ADC_CHANNEL_5
// ADC GPIO 宏定义
#define CURR_W_ADC_GPIO_PORT              GPIOF
#define CURR_W_ADC_GPIO_PIN               GPIO_PIN_8
#define CURR_W_ADC_GPIO_CLK_ENABLE()      __GPIOF_CLK_ENABLE()

#define CURR_W_ADC_CHANNEL                ADC_CHANNEL_6
// ADC DMA 通道宏定义，这里我们使用DMA传输
#define TEMP_ADC_DMA_CLK_ENABLE()       __DMA2_CLK_ENABLE()
#define TEMP_ADC_DMA_CHANNEL            DMA_CHANNEL_2
#define TEMP_ADC_DMA_STREAM             DMA2_Stream0

#define ADC_DMA_IRQ                     DMA2_Stream0_IRQn
#define ADC_DMA_IRQ_Handler             DMA2_Stream0_IRQHandler
#define GET_ADC_CURR_VAL(val)           (((float)val)/(float)8.0/(float)0.02*(float)1000.0)        // 得到电流值，电压放大8倍，0.02是采样电阻，单位mA。

/*********************** 电源电压采集 ******************/

#define VBUS_GPIO_PORT                  GPIOF
#define VBUS_GPIO_PIN                   GPIO_PIN_9
#define VBUS_GPIO_CLK_ENABLE()          __GPIOF_CLK_ENABLE()

#define VBUS_ADC_CHANNEL                ADC_CHANNEL_7

#define GET_VBUS_VAL(val)               (((float)val - 1.24f) * 37.0f )      // 获取电压值（测量电压是电源电压的1/37）

extern DMA_HandleTypeDef DMA_Init_Handle;
extern ADC_HandleTypeDef ADC_Handle;

int32_t get_curr_val_v(void);//获取V相的电流值
int32_t get_curr_val_u(void);//获取U相的电流值
int32_t get_curr_val_w(void);//获取W相的电流值
void ADC_Init(void);				//ADC的初始化
float get_ntc_v_val(void);	//获取温度传感器端的电压值	
float get_ntc_r_val(void);	//获取温度传感器端的电阻值
float get_ntc_t_val(void);	//获取温度传感器的温度
float get_vbus_val(void);		//获取电源电压值
 
#endif /* __BSP_ADC_H */
