/**
 ******************************************************************************
 * @file    bsp_stepper_T_speed.c
 * @author  fire
 * @version V1.0
 * @date    2020-xx-xx
 * @brief   ����������μӼ����㷨
 ******************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:Ұ��  STM32 F103 ������
 * ��̳    :http://www.firebbs.cn
 * �Ա�    :http://firestm32.taobao.com
 *
 ******************************************************************************
 */
#include "./stepper/bsp_stepper_T_speed.h"

// ϵͳ�Ӽ��ٲ���
speedRampData srd;
// ��¼���������λ��
// int stepPosition = 0;
// ϵͳ���������״̬
struct GLOBAL_FLAGS status = {FALSE, FALSE, TRUE};

/*! \brief �Ը����Ĳ����ƶ��������
 *  ͨ��������ٵ�����ٶȣ��Ը����Ĳ�����ʼ����
 *  ������ٶȺͼ��ٶȺ�С������������ƶ���������û�ﵽ����ٶȾ�Ҫ��ʼ����
 *  \param step   �ƶ��Ĳ��� (����Ϊ˳ʱ�룬����Ϊ��ʱ��).
 *  \param accel  ���ٶ�,���ȡֵΪ100��ʵ��ֵΪ100*0.01*rad/sec^2=1rad/sec^2
 *  \param decel  ���ٶ�,���ȡֵΪ100��ʵ��ֵΪ100*0.01*rad/sec^2=1rad/sec^2
 *  \param speed  ����ٶ�,���ȡֵΪ100��ʵ��ֵΪ100*0.01*rad/sec=1rad/sec
 */
void stepper_move_T(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
	// �ﵽ����ٶ�ʱ�Ĳ���.
	unsigned int max_s_lim;
	// ���뿪ʼ���ٵĲ���(�����û���ٵ�������ٶ�ʱ)��
	unsigned int accel_lim;

	/*���ݲ����������жϣ�����Ϊ˳ʱ�룬����Ϊ��ʱ��*/
	if (step == 0) // ��������ֱ�ӷ���
	{
		return;
	}
	else if (step < 0) // ��ʱ��
	{
		srd.dir = CCW;
		// �������������������
		step = -step;
	}
	else // ˳ʱ��
	{
		srd.dir = CW;
	}
	// ���õ������
	MOTOR_DIR(srd.dir);

	// ���ֻ�ƶ�һ��
	if (step == 1)
	{
		// ֻ�ƶ�һ��
		srd.accel_count = -1;
		// ����״̬
		srd.run_state = DECEL;
		// ����ʱ
		srd.step_delay = 1000;
		// ���õ��Ϊ����״̬
		status.running = TRUE;
	}

	// ������Ϊ����ƶ�
	else if (step != 0)
	{
		// ��������ٶȼ���, ����õ�min_delay���ڶ�ʱ���ļ�������ֵ��
		// min_delay = (alpha / tt)/ w
		srd.min_delay = (int32_t)(A_T_x10 / speed);

		// ͨ�������һ��(c0) �Ĳ�����ʱ���趨���ٶȣ�����accel��λΪ0.1rad/sec^2
		// step_delay = 1/tt * sqrt(2*alpha/accel)
		// step_delay = ( tfreq*0.676/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100
		srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel)) / 10);

		// ������ٲ�֮��ﵽ����ٶȵ�����
		// max_s_lim = speed^2 / (2*alpha*accel)
		max_s_lim = (uint32_t)(speed * speed / (A_x200 * accel / 10));
		// ����ﵽ����ٶ�С��0.5�������ǽ���������Ϊ0
		// ��ʵ�����Ǳ����ƶ�����һ�����ܴﵽ��Ҫ���ٶ�
		if (max_s_lim == 0)
		{
			max_s_lim = 1;
		}

		// ������ٲ�֮�����Ǳ��뿪ʼ����
		// n1 = (n1+n2)decel / (accel + decel)
		accel_lim = (uint32_t)(step * decel / (accel + decel));
		// ���Ǳ����������1�����ܲ��ܿ�ʼ����.
		if (accel_lim == 0)
		{
			accel_lim = 1;
		}

		// ʹ�������������ǿ��Լ������һ�ο�ʼ���ٵ�λ��
		if (accel_lim <= max_s_lim)
		{
			// srd.decel_valΪ����
			srd.decel_val = accel_lim - step;
		}
		else
		{
			// ���ݡ����ٵ������ٶȵ��ڼ��ٿ�ʼ���ٶȡ�������ٵĲ���srd.decel_val
			srd.decel_val = -(max_s_lim * accel / decel);
		}
		// ��ֻʣ��һ�����Ǳ������
		if (srd.decel_val == 0)
		{
			srd.decel_val = -1;
		}

		// ���㿪ʼ����ʱ�Ĳ���
		srd.decel_start = step + srd.decel_val;

		// �������ٶȺ��������ǾͲ���Ҫ���м����˶�
		// srd.min_delay��������ٶȣ�ʱ��������ԼС����������ٶ�Խ��
		if (srd.step_delay <= srd.min_delay)
		{
			// ʱ���������С��ʱ����
			srd.step_delay = srd.min_delay;
			// ���µ��״̬Ϊ����״̬������������ٶ��������н׶�
			srd.run_state = RUN;
		}
		else
		{
			// ���µ��״̬Ϊ����״̬
			srd.run_state = ACCEL;
		}
		// ��λ���ٶȼ���ֵ
		srd.accel_count = 0;
		// ���µ��״̬Ϊ����״̬
		status.running = TRUE;
	}
	/*��ȡ��ǰ����ֵ*/
	int tim_count = __HAL_TIM_GET_COUNTER(&TIM_TimeBaseStructure);
	/*�ڵ�ǰ����ֵ���������ö�ʱ���Ƚ�ֵ*/
	__HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure, MOTOR_PUL_CHANNEL_x, tim_count + srd.step_delay);
	/*ʹ�ܶ�ʱ��ͨ��*/
	TIM_CCxChannelCmd(MOTOR_PUL_TIM, MOTOR_PUL_CHANNEL_x, TIM_CCx_ENABLE);
	// ʹ�ܵ��
	MOTOR_EN(ON);
}

/**
 * @brief  �ٶȾ���
 *	@note 	���ж���ʹ�ã�ÿ��һ���жϣ�����һ��
 * @retval ��
 */
void speed_decision()
{
	uint32_t tim_count = 0;
	uint32_t tmp = 0;
	// �����£��£�һ����ʱ����
	uint16_t new_step_delay = 0;
	// ���ٹ��������һ����ʱ���������ڣ�.
	static uint16_t last_accel_delay = 0;
	// ���ƶ�����������
	static uint32_t step_count = 0;
	// �����ۼ�
	static int32_t rest = 0;
	// ��ʱ��ʹ�÷�תģʽ����Ҫ���������жϲ����һ����������
	static uint8_t i = 0;

	/*�жϴ����жϵ�����*/
	if (__HAL_TIM_GET_IT_SOURCE(&TIM_TimeBaseStructure, MOTOR_TIM_IT_CCx) != RESET)
	{
		// �����ʱ���ж�
		__HAL_TIM_CLEAR_IT(&TIM_TimeBaseStructure, MOTOR_TIM_IT_CCx);

		// ���ñȽ�ֵ
		// �õ���ǰ�ļ������еļ���ֵ
		tim_count = __HAL_TIM_GET_COUNTER(&TIM_TimeBaseStructure);
		/*
			������һ���жϵıȽϼĴ�����ֵ��Ҳ���������pwm�źŵ���һ�η�ת�ļ�������ֵ
			��������ν����жϣ����ʾһ������ֻ�����һ�룬����Ĳ�������if���û�н�ȥ��
			srd.step_delayû�и��£���ô�����pwm�ߵ͵�ƽ��ʱ��һ��������pwm�źŵ�ռ�ձ�Ϊ50%
		*/
		tmp = tim_count + srd.step_delay;
		/*���µļ���ֵд��ȽϼĴ���*/
		__HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure, MOTOR_PUL_CHANNEL_x, tmp);

		i++;		// ��ʱ���жϴ�������ֵ
		if (i == 2) // 2�Σ�˵���Ѿ����һ����������
		{
			i = 0; // ���㶨ʱ���жϴ�������ֵ
			/*����ϵͳ�Ӽ��ٵ�״̬��������ز����������ٶȾ���*/
			switch (srd.run_state)
			{
			/*�������ֹͣ״̬*/
			case STOP:
				step_count = 0; // ���㲽��������
				rest = 0;		// ������ֵ
				// �ر�ͨ��
				TIM_CCxChannelCmd(MOTOR_PUL_TIM, MOTOR_PUL_CHANNEL_x, TIM_CCx_DISABLE);
				// ����жϱ�־λ
				__HAL_TIM_CLEAR_FLAG(&TIM_TimeBaseStructure, MOTOR_TIM_FLAG_CCx);

				/*���µ��״̬Ϊfalse*/
				status.running = FALSE;
				break;
			/*�����������״̬*/
			case ACCEL:
				/*���ƶ�������һ*/
				step_count++;
				/*���ٲ�����һ*/
				srd.accel_count++;

				// ������(��)һ����������new_step_delay(ʱ����)����ع�ʽ������һƪ������������μӼ��ٵ�ԭ��������������
				new_step_delay = srd.step_delay - (((2 * srd.step_delay) + rest) / (4 * srd.accel_count + 1));
				// �����������´μ��㲹���������������
				rest = ((2 * srd.step_delay) + rest) % (4 * srd.accel_count + 1);

				// ����Ƿ�Ӧ�ÿ�ʼ���٣����ܲ����ǲ��Ǵﵽ�˼��ٿ�ʼ�Ĳ���
				if (step_count >= srd.decel_start)
				{
					// ��������������Ҫ�Ĳ���srd.decel_val��ֵ�����ٲ���srd.accel_count��ע��srd.decel_valΪ��ֵ����ʾ����
					srd.accel_count = srd.decel_val;
					/*���µ��״̬ΪDECEL����ʾ���ڼ���*/
					srd.run_state = DECEL;
				}
				/* ����Ƿ񵽴�����������ٶȣ��ٶ�Խ������ʱ����ԽС�������һ�ε�������new_step_delayС��
				��С��������srd.min_delay�����������ٶȣ�����ʾ��������һ�ε�����ٶ�Ҫ���������ٶȣ���������ֵ���� */
				else if (new_step_delay <= srd.min_delay)
				{
					// ����һ�ε�������new_step_delay��ֵ�����һ�μ���ʱ����last_accel_delay���Ա����ٵ�ʱ����
					last_accel_delay = new_step_delay;
					// ��һ�ε�����ʱ����new_step_delay����С������ʱ����srd.min_delay����֤���������ٶ���Ҫ����ٶȷ�Χ��
					new_step_delay = srd.min_delay;
					// ��������
					rest = 0;
					// ���µ��״̬Ϊ����״̬������������ٶ��������н׶�
					srd.run_state = RUN;
				}
				break;
			/*�����������ٶ�����״̬*/
			case RUN:
				/*���ƶ�������һ*/
				step_count++;
				// ��һ�ε�����ʱ����new_step_delayһֱ����С����ʱ����srd.min_delay������������ٶȲ�������
				new_step_delay = srd.min_delay;

				// ����Ƿ���Ҫ��ʼ����
				if (step_count >= srd.decel_start)
				{
					// ��������������Ҫ�Ĳ���srd.decel_val��ֵ�����ٲ���srd.accel_count��ע��srd.decel_valΪ��ֵ����ʾ����
					srd.accel_count = srd.decel_val;
					// �����һ�μ��ٵ���ʱ��Ϊ��ʼ���ٵ���ʱ
					new_step_delay = last_accel_delay;
					/*���µ��״̬ΪDECEL����ʾ���ڼ���*/
					srd.run_state = DECEL;
				}
				break;
			/*�����������״̬*/
			case DECEL:
				/*���ƶ�������һ*/
				step_count++;
				// ���ٲ�����һ,��ʱ�ļ��ٲ������������жϿ�ʼ���ٵ�ʱ��ֵ�ļ�������Ҫ�Ĳ�������һ������
				srd.accel_count++;
				// ������(��)һ����������new_step_delay(ʱ����)����ع�ʽ������һƪ������������μӼ��ٵ�ԭ��������������
				new_step_delay = srd.step_delay - (((2 * srd.step_delay) + rest) / (4 * srd.accel_count + 1));
				// �����������´μ��㲹������������
				rest = ((2 * srd.step_delay) + rest) % (4 * srd.accel_count + 1);
				// ����Ƿ�Ϊ���һ�����������ӵ�0ʱ����ʾ���ٵĲ�����ȫ������ˣ�������˼���
				if (srd.accel_count >= 0)
				{
					/*���µ��״̬ΪSTOP����ʾ��ֹͣ*/
					srd.run_state = STOP;
				}
				break;
			}
			// ����������������һ������ʱ����new_step_delay��ֵ����������ʱ����srd.step_delay���Ա������´μ���
			srd.step_delay = new_step_delay;
		}
	}
}
