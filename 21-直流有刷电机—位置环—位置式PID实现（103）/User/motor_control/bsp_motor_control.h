#ifndef __BSP_MOTOR_CONTROL_H
#define __BSP_MOTOR_CONTROL_H

#include "stm32f1xx.h"
#include "./tim/bsp_motor_tim.h"
#include "main.h"
#include "./tim/bsp_basic_tim.h"
#include "bsp_encoder.h"
#include "bsp_pid.h"
// ���Ŷ���
/*******************************************************/
// ����MOS�ܴ���? SD �ţ���������L298N���? EN ��
#define SHUTDOWN_PIN GPIO_PIN_12
#define SHUTDOWN_GPIO_PORT GPIOG
#define SHUTDOWN_GPIO_CLK_ENABLE() __HAL_RCC_GPIOG_CLK_ENABLE()
/*******************************************************/

/* ���? SD or EN ʹ�ܽ� */
#define MOTOR_ENABLE_SD() HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT, SHUTDOWN_PIN, GPIO_PIN_SET)    // �ߵ�ƽ��-�ߵ�ƽʹ��
#define MOTOR_DISABLE_SD() HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT, SHUTDOWN_PIN, GPIO_PIN_RESET) // �͵�ƽ�ض�-�͵�ƽ����

/* ����������ö�� */
typedef enum
{
  MOTOR_FWD = 0,
  MOTOR_REV,
} motor_dir_t;

/* �����ٶȣ�ռ�ձȣ� */
#define SET_FWD_COMPAER(ChannelPulse) TIM1_SetPWM_pulse(PWM_CHANNEL_1, ChannelPulse) // ���ñȽϼĴ�����ֵ
#define SET_REV_COMPAER(ChannelPulse) TIM1_SetPWM_pulse(PWM_CHANNEL_2, ChannelPulse) // ���ñȽϼĴ�����ֵ

/* ʹ�����? */
#define MOTOR_FWD_ENABLE() HAL_TIM_PWM_Start(&DCM_TimeBaseStructure, PWM_CHANNEL_1);
#define MOTOR_REV_ENABLE() HAL_TIM_PWM_Start(&DCM_TimeBaseStructure, PWM_CHANNEL_2);

/* �������? */
#define MOTOR_FWD_DISABLE() HAL_TIM_PWM_Stop(&DCM_TimeBaseStructure, PWM_CHANNEL_1);
#define MOTOR_REV_DISABLE() HAL_TIM_PWM_Stop(&DCM_TimeBaseStructure, PWM_CHANNEL_2);
#define CIRCLE_PULSES (ENCODER_TOTAL_RESOLUTION * REDUCTION_RATIO)
void motor_init(void);
void set_motor_speed(uint16_t v);
void set_motor_direction(motor_dir_t dir);
void set_motor_enable(void);
void set_motor_disable(void);
void show_help(void);
void deal_serial_data(void);
void motor_pid_control(void);

#endif /* __LED_H */
