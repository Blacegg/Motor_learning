/**
 ****************************************************************************************************
 * @file        pid.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       PID�㷨����
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� F407���������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20211014
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#include "./BSP/PID/pid.h"
#include "./BSP/BLDC/bldc.h"

PID_TypeDef  g_location_pid;       /*λ��PID�����ṹ��*/
PID_TypeDef  g_speed_pid;       /*λ��PID�����ṹ��*/
/**
 * @brief       ��ʼ��LED���IO��, ��ʹ��ʱ��
 * @param       ��
 * @retval      ��
 */
void pid_init(void)
{
    g_speed_pid.SetPoint = 0; /* �趨Ŀ��Desired Value*/
    g_speed_pid.ActualValue = 0.0; /*  �趨Ŀ��Desired Value*/
    g_speed_pid.SumError = 0.0; /* ����ֵ*/
    g_speed_pid.Error = 0.0;   /*  Error[1]*/
    g_speed_pid.LastError = 0.0; /*  Error[-1]*/
    g_speed_pid.PrevError = 0.0; /*  Error[-2]*/
    g_speed_pid.Proportion = KP; /*  �������� Proportional Const*/
    g_speed_pid.Integral = KI; /*  ���ֳ��� Integral Const*/
    g_speed_pid.Derivative = KD; /*  ΢�ֳ��� Derivative Const*/ 
    g_speed_pid.IngMax = 20;
    g_speed_pid.IngMin = -20;
    g_speed_pid.OutMax = 150;                  /* ������� */
    g_speed_pid.OutMin = -150;    
}


/**
  * �������ƣ�λ�ñջ�PID�������
  * �����������ǰ������
  * �� �� ֵ��Ŀ�������
  * ˵    ������
  */
int32_t increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value)
{
    PID->Error = (float)(PID->SetPoint - Feedback_value);   /*�ٶȵ�λƫ��*/
#if  INCR_LOCT_SELECT
    PID->ActualValue += (PID->Proportion * (PID->Error - PID->LastError)) /*E[k]��*/
                        + (PID->Integral * PID->Error)              /*E[k-1]��*/
                        + (PID->Derivative * (PID->Error - 2 * PID->LastError + PID->PrevError)); /*E[k-2]��*/
    PID->PrevError = PID->LastError;                      /*�洢�������´μ���*/
    PID->LastError = PID->Error;
#else
    PID->SumError += PID->Error;
    PID->ActualValue = (PID->Proportion * PID->Error) /*E[k]��*/
                       + (PID->Integral * PID->SumError)              /*E[k-1]��*/
                       + (PID->Derivative * (PID->Error - PID->LastError)); /*E[k-2]��*/
    PID->LastError = PID->Error;
#endif
    return ((int32_t)(PID->ActualValue));                /*����ʵ�ʿ�����ֵ*/
    

}
