/**
 ******************************************************************************
 * @file    bsp_motor_control.c
 * @author  fire
 * @version V1.0
 * @date    2019-xx-xx
 * @brief   电机控制接口
 ******************************************************************************
 * @attention
 *
 * 实验平台:野火  STM32 F103 开发板
 * 论坛    :http://www.firebbs.cn
 * 淘宝    :http://firestm32.taobao.com
 *
 ******************************************************************************
 */

#include ".\motor_control\bsp_motor_control.h"
#include "./usart/bsp_debug_usart.h"
#include "./pid/pid.h"
#include <math.h>
#include <stdlib.h>

/* 私有变量 */
static bldcm_data_t bldcm_data;

/* 局部函数 */
static void sd_gpio_config(void);

/**
 * @brief  电机初始化
 * @param  无
 * @retval 无
 */
void bldcm_init(void)
{
  Motor_TIMx_Configuration(); // 电机控制定时器，引脚初始化
  hall_tim_config();          // 霍尔传感器初始化
  sd_gpio_config();           // sd 引脚初始化
}

/**
 * @brief  电机 SD 控制引脚初始化
 * @param  无
 * @retval 无
 */
static void sd_gpio_config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* 定时器通道功能引脚端口时钟使能 */
  SHUTDOWN_GPIO_CLK_ENABLE();

  /* 引脚IO初始化 */
  /*设置输出类型*/
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  /*设置引脚速率 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  /*选择要控制的GPIO引脚*/
  GPIO_InitStruct.Pin = SHUTDOWN_PIN;

  /*调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/
  HAL_GPIO_Init(SHUTDOWN_GPIO_PORT, &GPIO_InitStruct);
BLDCM_ENABLE_SD();     // 默认开启
	HAL_Delay(1);
}

/**
 * @brief  设置电机速度
 * @param  v: 速度（占空比）
 * @retval 无
 */
void set_bldcm_speed(uint16_t v)
{
  bldcm_data.dutyfactor = v;

  set_pwm_pulse(v); // 设置速度
}

/**
 * @brief  设置电机方向
 * @param  无
 * @retval 无
 */
void set_bldcm_direction(motor_dir_t dir)
{
  bldcm_data.direction = dir;
}

/**
 * @brief  获取电机当前方向
 * @param  无
 * @retval 无
 */
motor_dir_t get_bldcm_direction(void)
{
  return bldcm_data.direction;
}

/**
 * @brief  使能电机
 * @param  无
 * @retval 无
 */
void set_bldcm_enable(void)
{
  hall_enable();
  bldcm_data.is_enable = 1;
}

/**
 * @brief  禁用电机
 * @param  无
 * @retval 无
 */
void set_bldcm_disable(void)
{
  /* 禁用霍尔传感器接口 */
  hall_disable();

  /* 停止 PWM 输出 */
  stop_pwm_output();
  bldcm_data.is_enable = 0;
}

void bldcm_pid_control(void)
{
  int32_t speed_actual = get_motor_speed();
  if (bldcm_data.is_enable)
  {
    float cont_val = 0;
    cont_val = PID_realize(speed_actual);

    if (cont_val < 0)
    {
      cont_val = -cont_val;
      bldcm_data.direction = MOTOR_FWD;
    }
    else
    {
      bldcm_data.direction = MOTOR_REV;
    }

    cont_val = (cont_val > PWM_PERIOD_COUNT) ? PWM_PERIOD_COUNT : cont_val;
    set_bldcm_speed(cont_val);
#ifdef PID_ASSISTANT_EN
    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &speed_actual, 1);
#else
    printf("实际值：%d, 目标值：%.0f，控制值: %.0f\n", speed_actual, get_pid_target(), cont_val);
#endif
  }
}
