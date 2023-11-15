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
    float target_val; // 目标值
    float actual_val; // 实际值
    float err;        // 定义偏差值
    float err_last;   // 定义上一个偏差值
    float Kp, Ki, Kd; // 定义比例、积分、微分系数
    float integral;   // 定义积分值
} _pid;

extern _pid pid;

void PID_param_init(void);
void set_pid_target(float temp_val);
float get_pid_target(void);
void set_p_i_d(float p, float i, float d);
float PID_realize(float actual_val);

#endif /*__PID__*/
