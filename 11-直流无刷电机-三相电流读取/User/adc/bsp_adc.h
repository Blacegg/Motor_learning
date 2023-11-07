#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "stm32f1xx.h"

// ADC 序号宏定义
#define CURR_ADC ADC1
#define CURR_ADC_CLK_ENABLE() __HAL_RCC_ADC1_CLK_ENABLE()

#define ADC_VBUS_IRQ ADC_IRQn
#define ADC_VBUS_IRQHandler ADC_IRQHandler

#define VREF 3.3f       // 参考电压，理论上是3.3
#define ADC_NUM_MAX 320 // ADC 转换结果缓冲区最大值

#define GET_ADC_VDC_VAL(val) ((float)val / (float)4096.0f * VREF) // 得到电压值

/*********************** 电流采集 ******************/
// ADC GPIO 宏定义
#define CURR_U_ADC_GPIO_PORT GPIOC
#define CURR_U_ADC_GPIO_PIN GPIO_PIN_0
#define CURR_U_ADC_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()

#define CURR_U_ADC_CHANNEL ADC_CHANNEL_10
// ADC GPIO 宏定义
#define CURR_V_ADC_GPIO_PORT GPIOC
#define CURR_V_ADC_GPIO_PIN GPIO_PIN_1
#define CURR_V_ADC_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()

#define CURR_V_ADC_CHANNEL ADC_CHANNEL_11
// ADC GPIO 宏定义
#define CURR_W_ADC_GPIO_PORT GPIOC
#define CURR_W_ADC_GPIO_PIN GPIO_PIN_2
#define CURR_W_ADC_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()

#define CURR_W_ADC_CHANNEL ADC_CHANNEL_12

// ADC DMA 通道宏定义，这里我们使用DMA传输
#define CURR_ADC_DMA_CLK_ENABLE() __HAL_RCC_DMA1_CLK_ENABLE()
#define CURR_ADC_DMA_STREAM DMA1_Channel1

#define ADC_DMA_IRQ DMA1_Channel1_IRQn
#define ADC_DMA_IRQ_Handler DMA1_Channel1_IRQHandler
#define GET_ADC_CURR_VAL(val) (((float)val) / (float)6.0 / (float)0.02 * (float)1000.0) // 得到电流值，电压放大8倍，0.02是采样电阻，单位mA。

extern DMA_HandleTypeDef DMA_Init_Handle;
extern ADC_HandleTypeDef ADC_Handle;

void ADC_Init(void);          // ADC的初始化
int32_t get_curr_val_v(void); // 获取V相的电流值
int32_t get_curr_val_u(void); // 获取U相的电流值
int32_t get_curr_val_w(void); // 获取W相的电流值

#endif /* __BSP_ADC_H */
