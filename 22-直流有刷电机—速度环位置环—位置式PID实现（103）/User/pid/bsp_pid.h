#ifndef __BSP_PID_H
#define __BSP_PID_H
#include "stm32f1xx.h"
#include "./usart/bsp_debug_usart.h"
#include <stdio.h>
#include <stdlib.h>

/*pid*/
typedef struct
{
    float target_val; // Ŀ��ֵ
    float actual_val; // ʵ��ֵ
    float err;        // ����ƫ��ֵ
    float err_last;   // ������һ��ƫ��ֵ
    float Kp, Ki, Kd; // ������������֡�΢��ϵ��
    float integral;   // �������ֵ
} _pid;
extern _pid pid_location;
extern _pid pid_speed;
extern void PID_param_init(void);
extern void set_pid_target(_pid *pid, float temp_val);
extern float get_pid_target(_pid *pid);
extern void set_p_i_d(_pid *pid, float p, float i, float d);
extern float location_pid_realize(_pid *pid, float actual_val);
extern float speed_pid_realize(_pid *pid, float actual_val);

#endif
