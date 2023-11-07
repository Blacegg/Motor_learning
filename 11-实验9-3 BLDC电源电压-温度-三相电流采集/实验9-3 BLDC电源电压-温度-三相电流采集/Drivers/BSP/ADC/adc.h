/**
 ****************************************************************************************************
 * @file        adc.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-18
 * @brief       ADC 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211018
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __ADC_H
#define __ADC_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* ADC及引脚 定义 */

#define ADC_ADCX_CH0_GPIO_PORT              GPIOB                                               /* 电源电压采集引脚 */
#define ADC_ADCX_CH0_GPIO_PIN               GPIO_PIN_1
#define ADC_ADCX_CH0_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define ADC_ADCX_CH1_GPIO_PORT              GPIOA                                               /* 温度采集引脚 */
#define ADC_ADCX_CH1_GPIO_PIN               GPIO_PIN_0
#define ADC_ADCX_CH1_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define ADC_ADCX_CH2_GPIO_PORT              GPIOB                                               /* U相采集引脚 */
#define ADC_ADCX_CH2_GPIO_PIN               GPIO_PIN_0
#define ADC_ADCX_CH2_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define ADC_ADCX_CH3_GPIO_PORT              GPIOA                                               /* V相采集引脚 */
#define ADC_ADCX_CH3_GPIO_PIN               GPIO_PIN_6
#define ADC_ADCX_CH3_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define ADC_ADCX_CH4_GPIO_PORT              GPIOA                                               /* W相采集引脚 */
#define ADC_ADCX_CH4_GPIO_PIN               GPIO_PIN_3
#define ADC_ADCX_CH4_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define ADC_ADCX                            ADC1 
#define ADC_ADCX_CH0                        ADC_CHANNEL_9                                       /* 通道Y,  0 <= Y <= 17 */ 
#define ADC_ADCX_CH1                        ADC_CHANNEL_0
#define ADC_ADCX_CH2                        ADC_CHANNEL_8
#define ADC_ADCX_CH3                        ADC_CHANNEL_6
#define ADC_ADCX_CH4                        ADC_CHANNEL_3

#define ADC_ADCX_CHY_CLK_ENABLE()           do{ __HAL_RCC_ADC1_CLK_ENABLE(); }while(0)          /* ADC1 时钟使能 */

#define ADC_CH_NUM                          5                                                   /* 需要转换的通道数目 */
#define ADC_COLL                            50                                                  /* 单采集次数 */
#define ADC_SUM                             ADC_CH_NUM * ADC_COLL                               /* 总采集次数 */

/* ADC单通道/多通道 DMA采集 DMA数据流相关 定义
 * 注意: 这里我们的通道还是使用上面的定义.
 */
#define ADC_ADCX_DMASx                      DMA2_Stream4
#define ADC_ADCX_DMASx_Chanel               DMA_CHANNEL_0                                       /* ADC1_DMA请求源 */
#define ADC_ADCX_DMASx_IRQn                 DMA2_Stream4_IRQn
#define ADC_ADCX_DMASx_IRQHandler           DMA2_Stream4_IRQHandler

extern uint16_t g_adc_val[ADC_CH_NUM];


/******************************************************************************************/

void adc_init(void);                                                /* ADC初始化 */
uint32_t adc_get_result_average(uint8_t ch);                        /* 获得某个通道值  */
void adc_nch_dma_init(void);                                        /* ADC DMA采集初始化 */
                        

#endif 















