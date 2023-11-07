/**
 ****************************************************************************************************
 * @file        adc.c
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

#include "./BSP/ADC/adc.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/DMA/dma.h"
#include "./BSP/BLDC/bldc.h"
#include "./BSP/TIMER/bldc_tim.h"

/* 多通道ADC采集 DMA读取 */
ADC_HandleTypeDef g_adc_nch_dma_handle;             /* 与DMA关联的ADC句柄 */
DMA_HandleTypeDef g_dma_nch_adc_handle;             /* 与ADC关联的DMA句柄 */
uint8_t g_adc_dma_sta = 0;                          /* DMA传输状态标志, 0,未完成; 1, 已完成 */

uint16_t g_adc_value[ADC_CH_NUM * ADC_COLL] = {0};  /* 存储ADC原始值 */
float g_adc_u_value[ADC_CH_NUM] = {0};              /* 存储ADC转换后的电压值 */

/***************************************多通道ADC采集(DMA读取)程序*****************************************/


/**
 * @brief       adc 初始化函数
 * @note        配置ADC转换通道
 * @param       无
 * @retval      无
 */
void adc_init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    g_adc_nch_dma_handle.Instance = ADC_ADCX;
    g_adc_nch_dma_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;            /* 4分频，ADCCLK = PCLK2/4 = 84/4 = 21Mhz */
    g_adc_nch_dma_handle.Init.Resolution = ADC_RESOLUTION_12B;                      /* 12位模式 */
    g_adc_nch_dma_handle.Init.ScanConvMode = ENABLE;                                /* 扫描模式 多通道使用 */
    g_adc_nch_dma_handle.Init.ContinuousConvMode = ENABLE;                          /* 连续转换模式，转换完成之后接着继续转换 */
    g_adc_nch_dma_handle.Init.DiscontinuousConvMode = DISABLE;                      /* 禁止不连续采样模式 */
    g_adc_nch_dma_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; /* 使用软件触发 */
    g_adc_nch_dma_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;                /* 软件触发 */
    g_adc_nch_dma_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;                      /* 右对齐 */
    g_adc_nch_dma_handle.Init.NbrOfConversion = ADC_CH_NUM;                         /* 使用转换通道数，需根据实际转换通道去设置 */
    g_adc_nch_dma_handle.Init.DMAContinuousRequests = ENABLE;                       /* 开启DMA连续转换 */
    g_adc_nch_dma_handle.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&g_adc_nch_dma_handle);

    /* 配置使用的ADC通道，采样序列里的第几个转换，增加或者减少通道需要修改这部分 */
    sConfig.Channel = ADC_ADCX_CH0;                         /* 电源电压采集 */
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&g_adc_nch_dma_handle, &sConfig);

    sConfig.Channel = ADC_ADCX_CH1;                         /* 温度采集 */
    sConfig.Rank = 2;
    HAL_ADC_ConfigChannel(&g_adc_nch_dma_handle, &sConfig);

    sConfig.Channel = ADC_ADCX_CH2;                         /* U相电压采集 */
    sConfig.Rank = 3;
    HAL_ADC_ConfigChannel(&g_adc_nch_dma_handle, &sConfig);
    
    sConfig.Channel = ADC_ADCX_CH3;                         /* V相电压采集 */
    sConfig.Rank = 4;
    HAL_ADC_ConfigChannel(&g_adc_nch_dma_handle, &sConfig);
    
    sConfig.Channel = ADC_ADCX_CH4;                         /* W相电压采集 */
    sConfig.Rank = 5;
    HAL_ADC_ConfigChannel(&g_adc_nch_dma_handle, &sConfig);
    
}

/**
 * @brief       ADC DMA读取 初始化函数
 * @note        本函数还是使用adc_init对ADC进行大部分配置,有差异的地方再单独配置
 * @param       par         : 外设地址
 * @param       mar         : 存储器地址
 * @retval      无
 */
void adc_nch_dma_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
      
    ADC_ADCX_CHY_CLK_ENABLE();                                  /* 使能ADCx时钟 */
    ADC_ADCX_CH0_GPIO_CLK_ENABLE();                             /* 开启GPIO时钟 */
    ADC_ADCX_CH1_GPIO_CLK_ENABLE();
    ADC_ADCX_CH2_GPIO_CLK_ENABLE();
    ADC_ADCX_CH3_GPIO_CLK_ENABLE();
    ADC_ADCX_CH4_GPIO_CLK_ENABLE();
    
    /* AD采集引脚模式设置,模拟输入 */
    GPIO_InitStruct.Pin = ADC_ADCX_CH0_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADC_ADCX_CH0_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ADC_ADCX_CH1_GPIO_PIN;
    HAL_GPIO_Init(ADC_ADCX_CH1_GPIO_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = ADC_ADCX_CH2_GPIO_PIN;   
    HAL_GPIO_Init(ADC_ADCX_CH2_GPIO_PORT, &GPIO_InitStruct); 
    
    GPIO_InitStruct.Pin = ADC_ADCX_CH3_GPIO_PIN;   
    HAL_GPIO_Init(ADC_ADCX_CH3_GPIO_PORT, &GPIO_InitStruct); 
    
    GPIO_InitStruct.Pin = ADC_ADCX_CH4_GPIO_PIN;   
    HAL_GPIO_Init(ADC_ADCX_CH4_GPIO_PORT, &GPIO_InitStruct); 
    
    adc_init();
    
    if ((uint32_t)ADC_ADCX_DMASx > (uint32_t)DMA2)              /* 大于DMA1_Channel7, 则为DMA2的通道了 */
    {
        __HAL_RCC_DMA2_CLK_ENABLE();                            /* DMA2时钟使能 */
    }
    else 
    {
        __HAL_RCC_DMA1_CLK_ENABLE();                            /* DMA1时钟使能 */
    }

    /* DMA配置 */
    g_dma_nch_adc_handle.Instance = ADC_ADCX_DMASx;                             /* 设置DMA通道 */
    g_dma_nch_adc_handle.Init.Channel = DMA_CHANNEL_0;
    g_dma_nch_adc_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;                 /* DIR = 1 ,  外设到存储器模式 */
    g_dma_nch_adc_handle.Init.PeriphInc = DMA_PINC_DISABLE;                     /* 外设非增量模式 */
    g_dma_nch_adc_handle.Init.MemInc = DMA_MINC_ENABLE;                         /* 存储器增量模式 */
    g_dma_nch_adc_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;    /* 外设数据长度:16位 */
    g_dma_nch_adc_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;       /* 存储器数据长度:16位 */
    g_dma_nch_adc_handle.Init.Mode = DMA_CIRCULAR;                              /* 外设流控模式 */
    g_dma_nch_adc_handle.Init.Priority = DMA_PRIORITY_MEDIUM;                   /* 中等优先级 */
    HAL_DMA_Init(&g_dma_nch_adc_handle);
 
    __HAL_LINKDMA(&g_adc_nch_dma_handle,DMA_Handle,g_dma_nch_adc_handle);

    HAL_NVIC_SetPriority(ADC_ADCX_DMASx_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(ADC_ADCX_DMASx_IRQn);
    
    HAL_ADC_Start_DMA(&g_adc_nch_dma_handle,(uint32_t *)g_adc_value,ADC_CH_NUM * ADC_COLL);
}

/**
 * @brief       ADC DMA采集中断服务函数
 * @param       无 
 * @retval      无
 */
void ADC_ADCX_DMASx_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_dma_nch_adc_handle);
}

uint16_t g_adc_val[ADC_CH_NUM];                     /* ADC平均值存放数组 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)                     /* 大约2.6ms采集完成进入中断 */
    { 
        HAL_ADC_Stop_DMA(&g_adc_nch_dma_handle);    /* 关闭DMA转换 */
        calc_adc_val(g_adc_val);                    /* ADC数值转换 */
        HAL_ADC_Start_DMA(&g_adc_nch_dma_handle, (uint32_t *)&g_adc_value, (uint32_t)(ADC_SUM)); /* 再启动DMA转换*/
    }
}


/**
 * @brief       获取通道ch的转换值，取times次, 然后平均
 * @param       ch: 通道号, 0~17
 * @retval      通道ch的times次转换结果平均值
 */
uint32_t adc_get_result_average(uint8_t ch)
{
    uint32_t temp_val = 0;
    uint16_t t;

    for (t = ch; t < ADC_SUM; t += ADC_CH_NUM )     /* 获取times次数据 */
    {
        temp_val += g_adc_value[t];
    }

    return temp_val / ADC_COLL;                     /* 返回平均值 */
}
