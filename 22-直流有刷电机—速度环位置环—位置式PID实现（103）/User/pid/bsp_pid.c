#include "./pid/bsp_pid.h"
#include "math.h"
#include "./key/bsp_key.h"
#include "./protocol/protocol.h"
#include "bsp_motor_control.h"

// ����ȫ�ֱ���

_pid pid_location;
_pid pid_speed;

/**
 * @brief  PID������ʼ��
 *	@note 	��
 * @retval ��
 */
void PID_param_init()
{
	pid_location.target_val = PER_CYCLE_PULSES;
	pid_location.actual_val = 0.0;
	pid_location.err = 0.0;
	pid_location.err_last = 0.0;
	pid_location.integral = 0.0;

	pid_location.Kp = 0.045;
	pid_location.Ki = 0.0;
	pid_location.Kd = 0.0;

	pid_speed.target_val = 100.0;
	pid_speed.actual_val = 0.0;
	pid_speed.err = 0.0;
	pid_speed.err_last = 0.0;
	pid_speed.integral = 0.0;

	pid_speed.Kp = 10.0;
	pid_speed.Ki = 5.0;
	pid_speed.Kd = 0.0;
#if defined(PID_ASSISTANT_EN)
	float pid_temp[3] = {pid_location.Kp, pid_location.Ki, pid_location.Kd};
	set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);

	pid_temp[0] = pid_speed.Kp;
	pid_temp[1] = pid_speed.Ki;
	pid_temp[2] = pid_speed.Kd;
	set_computer_value(SEND_P_I_D_CMD, CURVES_CH2, pid_temp, 3);
#endif
}

/**
 * @brief  ����Ŀ��ֵ
 * @param  val		Ŀ��ֵ
 *	@note 	��
 * @retval ��
 */
void set_pid_target(_pid *pid, float temp_val)
{
	pid->target_val = temp_val; // ���õ�ǰ��Ŀ��ֵ
}

/**
 * @brief  ��ȡĿ��ֵ
 * @param  ��
 *	@note 	��
 * @retval Ŀ��ֵ
 */
float get_pid_target(_pid *pid)
{
	return pid->target_val; // ���õ�ǰ��Ŀ��ֵ
}

/**
 * @brief  ���ñ��������֡�΢��ϵ��
 * @param  p������ϵ�� P
 * @param  i������ϵ�� i
 * @param  d��΢��ϵ�� d
 *	@note 	��
 * @retval ��
 */
void set_p_i_d(_pid *pid, float p, float i, float d)
{
	pid->Kp = p; // ���ñ���ϵ�� P
	pid->Ki = i; // ���û���ϵ�� I
	pid->Kd = d; // ����΢��ϵ�� D
}

/**
 * @brief  PID�㷨ʵ��
 * @param  val		ʵ��ֵ
 * @note 	��
 * @retval ͨ��PID���������
 */
float location_pid_realize(_pid *pid, float actual_val)
{
	/*����Ŀ��ֵ��ʵ��ֵ�����*/
	pid->err = pid->target_val - actual_val;

	if ((pid->err >= -20) && (pid->err <= 20))
	{
		pid->err = 0;
		pid->integral = 0;
	}
	/*����ۻ�*/
	pid->integral += pid->err;
	/*PID�㷨ʵ��*/
	pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);

	pid->err_last = pid->err;
	/*���ص�ǰʵ��ֵ*/
	return pid->actual_val;
}

float speed_pid_realize(_pid *pid, float actual_val)
{
	pid->err = pid->target_val - actual_val;

	if ((pid->err < 0.2f) && (pid->err > -0.2f))
		pid->err = 0.0f;

	pid->integral += pid->err;

	pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);

	pid->err_last = pid->err;

	return pid->actual_val;
}
