/**
  ******************************************************************************
  * @file    bsp_adc.c
  * @author  long
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   NTC驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */

#include "./adc/bsp_adc.h"
#include "./led/bsp_led.h" 
#include "./usart/bsp_debug_usart.h"
#include <math.h>


DMA_HandleTypeDef DMA_Init_Handle;
ADC_HandleTypeDef ADC_Handle;

static int16_t adc_buff[ADC_NUM_MAX];    // 电压采集缓冲区
static int16_t vbus_adc_mean = 0;        // 电源电压 ACD 采样结果平均值
static uint32_t adc_mean_t = 0;        // 平均值累加
static uint32_t adc_mean_sum_u = 0;        // 平均值累加
static uint32_t adc_mean_sum_v = 0;        // 平均值累加
static uint32_t adc_mean_sum_w = 0;        // 平均值累加
static uint32_t adc_mean_count_u = 0;      // 累加计数
static uint32_t adc_mean_count_v = 0;      // 累加计数
static uint32_t adc_mean_count_w = 0;      // 累加计数
/**
  * @brief  ADC 通道引脚初始化
  * @param  无
  * @retval 无
  */
static void ADC_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // 使能 GPIO 时钟
    TEMP_ADC_GPIO_CLK_ENABLE();
    VBUS_GPIO_CLK_ENABLE();
		CURR_U_ADC_GPIO_CLK_ENABLE();
		CURR_V_ADC_GPIO_CLK_ENABLE();
		CURR_W_ADC_GPIO_CLK_ENABLE();
    // 配置 IO
    GPIO_InitStructure.Pin = TEMP_ADC_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
    GPIO_InitStructure.Pull = GPIO_NOPULL ; //不上拉不下拉
    HAL_GPIO_Init(TEMP_ADC_GPIO_PORT, &GPIO_InitStructure);	

    GPIO_InitStructure.Pin = VBUS_GPIO_PIN;
    HAL_GPIO_Init(VBUS_GPIO_PORT, &GPIO_InitStructure);	
	
		GPIO_InitStructure.Pin = CURR_U_ADC_GPIO_PIN;
    HAL_GPIO_Init(CURR_U_ADC_GPIO_PORT, &GPIO_InitStructure);	
	
		GPIO_InitStructure.Pin = CURR_V_ADC_GPIO_PIN;
    HAL_GPIO_Init(CURR_V_ADC_GPIO_PORT, &GPIO_InitStructure);
		
		GPIO_InitStructure.Pin = CURR_W_ADC_GPIO_PIN;
    HAL_GPIO_Init(CURR_W_ADC_GPIO_PORT, &GPIO_InitStructure);
}

void adc_dma_init(void)
{
    // ------------------DMA Init 结构体参数 初始化--------------------------
    // ADC3使用DMA2，数据流0，通道0，这个是手册固定死的
    // 开启DMA时钟
    TEMP_ADC_DMA_CLK_ENABLE();
    // 数据传输通道
    DMA_Init_Handle.Instance = TEMP_ADC_DMA_STREAM;
    // 数据传输方向为外设到存储器	
    DMA_Init_Handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    // 外设寄存器只有一个，地址不用递增
    DMA_Init_Handle.Init.PeriphInc = DMA_PINC_DISABLE;
    // 存储器地址固定
    DMA_Init_Handle.Init.MemInc = DMA_MINC_ENABLE;
    // 外设数据大小为半字，即两个字节
    DMA_Init_Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    //	存储器数据大小也为半字，跟外设数据大小相同
    DMA_Init_Handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;	
    // 循环传输模式
    DMA_Init_Handle.Init.Mode = DMA_CIRCULAR;
    // DMA 传输通道优先级为高，当使用一个DMA通道时，优先级设置不影响
    DMA_Init_Handle.Init.Priority = DMA_PRIORITY_HIGH;
    // 禁止DMA FIFO	，使用直连模式
    DMA_Init_Handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;  
    // FIFO 大小，FIFO模式禁止时，这个不用配置
    DMA_Init_Handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    DMA_Init_Handle.Init.MemBurst = DMA_MBURST_SINGLE;
    DMA_Init_Handle.Init.PeriphBurst = DMA_PBURST_SINGLE;  
    // 选择 DMA 通道，通道存在于流中
    DMA_Init_Handle.Init.Channel = TEMP_ADC_DMA_CHANNEL; 
    //初始化DMA流，流相当于一个大的管道，管道里面有很多通道
    HAL_DMA_Init(&DMA_Init_Handle); 

    __HAL_LINKDMA(&ADC_Handle,DMA_Handle,DMA_Init_Handle);
}

/**
  * @brief  ADC 模式配置
  * @param  无
  * @retval 无
  */
static void ADC_Mode_Config(void)
{
    // 开启ADC时钟
    TEMP_ADC_CLK_ENABLE();
    // -------------------ADC Init 结构体 参数 初始化------------------------
    // ADC3
    ADC_Handle.Instance = TEMP_ADC;
    // 时钟为fpclk 4分频	
    ADC_Handle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
    // ADC 分辨率
    ADC_Handle.Init.Resolution = ADC_RESOLUTION_12B;
    // 扫描模式，多通道采集才需要	
    ADC_Handle.Init.ScanConvMode = ENABLE; 
    // 连续转换	
    ADC_Handle.Init.ContinuousConvMode = ENABLE;
    // 非连续转换	
    ADC_Handle.Init.DiscontinuousConvMode = DISABLE;
    // 非连续转换个数
    ADC_Handle.Init.NbrOfDiscConversion   = 0;
    //禁止外部边沿触发    
    ADC_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    //使用软件触发
    ADC_Handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    //数据右对齐	
    ADC_Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    //转换通道 5个
    ADC_Handle.Init.NbrOfConversion = 5;
    //使能连续转换请求
    ADC_Handle.Init.DMAContinuousRequests = ENABLE;
    //转换完成标志
    ADC_Handle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;    
    // 初始化ADC	                          
    HAL_ADC_Init(&ADC_Handle);
    
    //---------------------------------------------------------------------------
    ADC_ChannelConfTypeDef ADC_Config;
    
    ADC_Config.Channel      = TEMP_ADC_CHANNEL;
    ADC_Config.Rank         = 1;
    // 采样时间间隔	
    ADC_Config.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    ADC_Config.Offset       = 0;
    // 配置 ADC 通道转换顺序为1，第一个转换，采样时间为3个时钟周期
    HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);
    
    // 配置 ADC 通道转换顺序为1，第一个转换，采样时间为3个时钟周期

    ADC_Config.Channel = VBUS_ADC_CHANNEL;
    ADC_Config.Rank				 = 2;
    // 采样时间间隔	
    ADC_Config.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    ADC_Config.Offset       = 0;
		HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);
		
    ADC_Config.Channel      = CURR_U_ADC_CHANNEL;
    ADC_Config.Rank         = 3;
    // 采样时间间隔	
    ADC_Config.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    ADC_Config.Offset       = 0;
    HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);
		
		ADC_Config.Channel      = CURR_V_ADC_CHANNEL;
    ADC_Config.Rank         = 4;
    // 采样时间间隔	
    ADC_Config.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    ADC_Config.Offset       = 0;
		HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);
		
		ADC_Config.Channel      = CURR_W_ADC_CHANNEL;
    ADC_Config.Rank         = 5;
    // 采样时间间隔	
    ADC_Config.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    ADC_Config.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config) != HAL_OK)
    {
      while(1);
    }
    
    // 外设中断优先级配置和使能中断配置
    HAL_NVIC_SetPriority(ADC_DMA_IRQ, 1, 1);
    HAL_NVIC_EnableIRQ(ADC_DMA_IRQ);

    HAL_ADC_Start_DMA(&ADC_Handle, (uint32_t*)&adc_buff, ADC_NUM_MAX);
}

/**
  * @brief  ADC 采集初始化
  * @param  无
  * @retval 无
  */
void ADC_Init(void)
{
	ADC_GPIO_Config();
  adc_dma_init();
	ADC_Mode_Config();
}

/**
  * @brief  常规转换在非阻塞模式下完成回调
  * @param  hadc: ADC  句柄.
  * @retval 无
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	int32_t adc_mean = 0;
  HAL_ADC_Stop_DMA(hadc);       // 停止 ADC 采样，处理完一次数据在继续采样
  
  /* 计算温度通道采样的平均值 */
  for(uint32_t count = 0; count < ADC_NUM_MAX; count+=5)
  {
    adc_mean += (int32_t)adc_buff[count];
  }
	  adc_mean_t = adc_mean / (ADC_NUM_MAX / 5);    // 保存平均值
		adc_mean = 0;

  
  /* 计算电压通道采样的平均值 */
  for(uint32_t count = 1; count < ADC_NUM_MAX; count+=5)
  {
    adc_mean += (int32_t)adc_buff[count];
  }
  
  vbus_adc_mean = adc_mean / (ADC_NUM_MAX / 5);    // 保存平均值
	  adc_mean = 0;
#if 1 
		  /* 计算电流通道采样的平均值 */
   for(uint32_t count = 2; count < ADC_NUM_MAX; count+=5)
  {
    adc_mean += (uint32_t)adc_buff[count];
  }
  
  adc_mean_sum_u += adc_mean / (ADC_NUM_MAX / 5);    // 累加电压
  adc_mean_count_u++;
	  adc_mean = 0;
			  /* 计算电流通道采样的平均值 */
   for(uint32_t count = 3; count < ADC_NUM_MAX; count+=5)
  {
    adc_mean += (uint32_t)adc_buff[count];
  }
  
  adc_mean_sum_v += adc_mean / (ADC_NUM_MAX / 5);    // 累加电压
  adc_mean_count_v++;
		adc_mean = 0;
			  /* 计算电流通道采样的平均值 */
   for(uint32_t count = 4; count < ADC_NUM_MAX; count+=5)
  {
    adc_mean += (uint32_t)adc_buff[count];
  }
  
  adc_mean_sum_w += adc_mean / (ADC_NUM_MAX / 5);    // 累加电压
  adc_mean_count_w++;
	adc_mean = 0;
#else
	  vbus_adc_mean = adc_buff[1];
	      /* 计算电流通道采样的平均值 */
  
#endif
  
  HAL_ADC_Start_DMA(&ADC_Handle, (uint32_t*)&adc_buff, ADC_NUM_MAX);    // 开始 ADC 采样
}

/**
  * @brief  获取温度传感器端的电压值
  * @param  无
  * @retval 转换得到的电压值
  */
float get_ntc_v_val(void)
{
  float vdc = GET_ADC_VDC_VAL(adc_mean_t);      // 获取电压值
  
  return vdc;
}

/**
  * @brief  获取温度传感器端的电阻值
  * @param  无
  * @retval 转换得到的电阻值
  */
float get_ntc_r_val(void)
{
  float r = 0;
  float vdc = get_ntc_v_val();
  
  r = (VREF - vdc) / (vdc / (float)4700.0);
  
  return r;
}

/**
  * @brief  获取温度传感器的温度
  * @param  无
  * @retval 转换得到的温度，单位：（℃）
  */
float get_ntc_t_val(void)
{
  float t = 0;             // 测量温度
  float Rt = 0;            // 测量电阻
  float Ka = 273.15;       // 0℃ 时对应的温度（开尔文）
  float R25 = 10000.0;     // 25℃ 电阻值
  float T25 = Ka + 25;     // 25℃ 时对应的温度（开尔文）
  float B = 3950.0;        /* B-常数：B = ln(R25 / Rt) / (1 / T – 1 / T25)，
                             其中 T = 25 + 273.15 */

  Rt = get_ntc_r_val();    // 获取当前电阻值

  t = B * T25 / (B + log(Rt / R25) * T25) - Ka ;    // 使用公式计算

  return t;
}
/**
  * @brief  获取V相的电流值
  * @param  无
  * @retval 转换得到的电流值
  */
int32_t get_curr_val_v(void)
{
  static uint8_t flag = 0;
  static uint32_t adc_offset = 0;    // 偏置电压
  int16_t curr_adc_mean = 0;         // 电流 ACD 采样结果平均值
  
  curr_adc_mean = adc_mean_sum_v / adc_mean_count_v;    // 保存平均值
  

    adc_mean_count_v = 0;
    adc_mean_sum_v = 0;
    
    if (flag < 17)
    {
      adc_offset = curr_adc_mean;    // 多次记录偏置电压，待系统稳定偏置电压才为有效值
      flag += 1;
    }
    if(curr_adc_mean>=adc_offset)
	{
		curr_adc_mean -= adc_offset;                     // 减去偏置电压
	}else
	{
		curr_adc_mean=0;
	}

  float vdc = GET_ADC_VDC_VAL(curr_adc_mean);      // 获取电压值
  
  return GET_ADC_CURR_VAL(vdc);
}
/**
  * @brief  获取U相的电流值
  * @param  无
  * @retval 转换得到的电流值
  */
int32_t get_curr_val_u(void)
{
  static uint8_t flag = 0;
  static uint32_t adc_offset = 0;    // 偏置电压
  int16_t curr_adc_mean = 0;         // 电流 ACD 采样结果平均值
  
  curr_adc_mean = adc_mean_sum_u / adc_mean_count_u;    // 保存平均值
  

    adc_mean_count_u = 0;
    adc_mean_sum_u = 0;
    
    if (flag < 17)
    {
      adc_offset = curr_adc_mean;    // 多次记录偏置电压，待系统稳定偏置电压才为有效值
      flag += 1;
    }
    if(curr_adc_mean>=adc_offset)
	{
		curr_adc_mean -= adc_offset;                     // 减去偏置电压
	}else
	{
		curr_adc_mean=0;
	}

  float vdc = GET_ADC_VDC_VAL(curr_adc_mean);      // 获取电压值
  
  return GET_ADC_CURR_VAL(vdc);
}
/**
  * @brief  获取W相的电流值
  * @param  无
  * @retval 转换得到的电流值
  */
int32_t get_curr_val_w(void)
{
  static uint8_t flag = 0;
  static uint32_t adc_offset = 0;    // 偏置电压
  int16_t curr_adc_mean = 0;         // 电流 ACD 采样结果平均值
  
  curr_adc_mean = adc_mean_sum_w / adc_mean_count_w;    // 保存平均值
  

    adc_mean_count_w = 0;
    adc_mean_sum_w = 0;
    
    if (flag < 17)
    {
      adc_offset = curr_adc_mean;    // 多次记录偏置电压，待系统稳定偏置电压才为有效值
      flag += 1;
    }
    if(curr_adc_mean>=adc_offset)
	{
		curr_adc_mean -= adc_offset;                     // 减去偏置电压
	}else
	{
		curr_adc_mean=0;
	}

  float vdc = GET_ADC_VDC_VAL(curr_adc_mean);      // 获取电压值
  
  return GET_ADC_CURR_VAL(vdc);
}
/**
  * @brief  获取电源电压值
  * @param  无
  * @retval 转换得到的电压值
  */
float get_vbus_val(void)
{
  float vdc = GET_ADC_VDC_VAL(vbus_adc_mean);      // 获取电压值
  return GET_VBUS_VAL(vdc);
}

/*********************************** END OF FILE *********************************************/
