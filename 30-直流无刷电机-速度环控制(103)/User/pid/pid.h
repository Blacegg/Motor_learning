/***********************************************************************
 * @file pid.h
 * @brief
 * @author blacegg
 * @version 1.0
 * @date 2023-08-21
 ***********************************************************************/
#ifndef __PID__
#define __PID__

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "./protocol/protocol.h"

#define PID_ASSISTANT_EN

typedef struct
{
    float target_val; // Ŀ��ֵ
    float actual_val; // ʵ��ֵ
    float err;        // ����ƫ��ֵ
    float err_last;   // ������һ��ƫ��ֵ
    float Kp, Ki, Kd; // ������������֡�΢��ϵ��
    float integral;   // �������ֵ
} _pid;

extern _pid pid;

void PID_param_init(void);
void set_pid_target(float temp_val);
float get_pid_target(void);
void set_p_i_d(float p, float i, float d);
float PID_realize(float actual_val);

#endif /*__PID__*/
