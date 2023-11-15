/***********************************************************************
 * @file pid.c
 * @brief
 * @author blacegg
 * @version 1.0
 * @date 2023-08-21
 ***********************************************************************/

#include "pid.h"

_pid pid;

/***********************************************************************
 * @brief PID_param_init
 ***********************************************************************/
void PID_param_init(void)
{
    pid.target_val = 0;
    pid.actual_val = 0;
    pid.err = 0;
    pid.err_last = 0;
    pid.integral = 0;
    pid.Kd = 0.00;
    pid.Ki = 0.10;
    pid.Kp = 0.07;
#if defined(PID_ASSISTANT_EN)
    float pid_temp[3] = {pid.Kp, pid.Ki, pid.Kd};
    set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);
#endif
}

/***********************************************************************
 * @brief Set the pid target object
 * @param [in/out] temp_val
 ***********************************************************************/
void set_pid_target(float temp_val)
{
    pid.target_val = temp_val;
}

/***********************************************************************
 * @brief Get the pid target object
 * @return float
 ***********************************************************************/
float get_pid_target(void)
{
    return pid.target_val;
}

/***********************************************************************
 * @brief Set the p i d object
 * @param [in/out] p
 * @param [in/out] i
 * @param [in/out] d
 ***********************************************************************/
void set_p_i_d(float p, float i, float d)
{
    pid.Kp = p;
    pid.Ki = i;
    pid.Kd = d;
}

/***********************************************************************
 * @brief PID_realize
 * @param [in/out] actual_val
 * @return float
 ***********************************************************************/
float PID_realize(float actual_val)
{
    pid.err = actual_val - pid.target_val;
    pid.integral += pid.err;

    pid.actual_val = pid.Kp * pid.err +
                     pid.Ki * pid.integral +
                     pid.Kd * (pid.err - pid.err_last);

    pid.err_last = pid.err;
    return pid.actual_val;
}
