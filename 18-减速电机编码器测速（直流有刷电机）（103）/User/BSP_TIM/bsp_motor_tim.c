#include "bsp_motor_tim.h"
TIM_HandleTypeDef TIM_TimeBaseStructure;
static void TIMx_GPIO_Config(void)
{
     GPIO_InitTypeDef GPIO_InitStruct;

     PWM_TIM_CH1_GPIO_CLK();
     PWM_TIM_CH2_GPIO_CLK();

     /* 定时器通道1功能引脚IO初始化 */
     /*设置输出类型*/
     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
     /*设置引脚速率 */
     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
     /*设置复用*/
     // PWM_TIM_GPIO_AF_ENALBE();

     /*选择要控制的GPIO引脚*/
     GPIO_InitStruct.Pin = PWM_TIM_CH1_PIN;
     /*调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/
     HAL_GPIO_Init(PWM_TIM_CH1_GPIO_PORT, &GPIO_InitStruct);

     GPIO_InitStruct.Pin = PWM_TIM_CH2_PIN;
     HAL_GPIO_Init(PWM_TIM_CH2_GPIO_PORT, &GPIO_InitStruct);
}

static void TIM_PWMOUTPUT_Config(void)
{
     TIM_OC_InitTypeDef TIM_OCInitStructure;

     /*使能定时器*/
     PWM_TIM_CLK_ENABLE();

     TIM_TimeBaseStructure.Instance = PWM_TIM;
     /* 累计 TIM_Period个后产生一个更新或者中断*/
     // 当定时器从0计数到PWM_PERIOD_COUNT-1，即为PWM_PERIOD_COUNT次，为一个定时周期
     TIM_TimeBaseStructure.Init.Period = PWM_PERIOD_COUNT - 1;
     // 通用控制定时器时钟源TIMxCLK = HCLK/2=84MHz
     // 设定定时器频率为=TIMxCLK/(PWM_PRESCALER_COUNT+1)
     TIM_TimeBaseStructure.Init.Prescaler = PWM_PRESCALER_COUNT - 1;

     /*计数方式*/
     TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;
     /*采样时钟分频*/
     TIM_TimeBaseStructure.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
     /*初始化定时器*/
     HAL_TIM_PWM_Init(&TIM_TimeBaseStructure);

     /*PWM模式配置*/
     TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
     TIM_OCInitStructure.Pulse = 0;
     TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
     TIM_OCInitStructure.OCNPolarity = TIM_OCPOLARITY_HIGH;
     TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
     TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
     TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;

     /*配置PWM通道*/
     HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure, &TIM_OCInitStructure, PWM_CHANNEL_1);
     /*开始输出PWM*/
     HAL_TIM_PWM_Start(&TIM_TimeBaseStructure, PWM_CHANNEL_1);

     /*配置脉宽*/
     TIM_OCInitStructure.Pulse = 0; // 默认占空比为50%
                                    /*配置PWM通道*/
     HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure, &TIM_OCInitStructure, PWM_CHANNEL_2);
     /*开始输出PWM*/
     HAL_TIM_PWM_Start(&TIM_TimeBaseStructure, PWM_CHANNEL_2);
}

void TIM1_SetPWM_pulse(uint32_t channel, int compare)
{
     switch (channel)
     {
     case TIM_CHANNEL_1:
          __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure, TIM_CHANNEL_1, compare);
          break;
     case TIM_CHANNEL_2:
          __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure, TIM_CHANNEL_2, compare);
          break;
     case TIM_CHANNEL_3:
          __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure, TIM_CHANNEL_3, compare);
          break;
     case TIM_CHANNEL_4:
          __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure, TIM_CHANNEL_4, compare);
          break;
     }
}
#include "bsp_led.h"
void TIMx_Configuration(void)
{
     TIMx_GPIO_Config();

     TIM_PWMOUTPUT_Config();
     // SetRegister_H(GPIOE, GPIO_PIN_9);
}
