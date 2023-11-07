/**
 ****************************************************************************************************
 * @file        pid.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       PID����
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� STM32F407���������
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
#ifndef __PID_H
#define __PID_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* PID��ز��� */

#define  INCR_LOCT_SELECT  1/*0ѡ��λ��ʽ1������ʽ����*/
#if INCR_LOCT_SELECT
/*����PID������غ�*/
#define  KP      8.50f       /* P����*/
#define  KI      5.00f       /* I����*/
#define  KD      0.08f       /* D����*/
#define SMAPLSE_PID_SPEED  50        /*������ ��λms*/
#else
/*����PID������غ�*/
#define  KP      10.0f       /* P����*/
#define  KI      8.00f       /* I����*/
#define  KD      0.5f        /* D����*/
#define SMAPLSE_PID_SPEED  50        /*������ ��λms*/
#endif

/*����λ��PID������غ�*/
/*PID�ṹ��*/
typedef struct
{
    __IO float  SetPoint;    /*�趨Ŀ�� */
    __IO float  ActualValue; /*ʵ��ֵ*/
    __IO float  SumError;    /*����ۼ�*/
    __IO float  Proportion;  /*�������� P*/
    __IO float  Integral;    /*���ֳ��� I*/
    __IO float  Derivative;  /*΢�ֳ��� D*/
    __IO float  Error;       /*Error[-1]*/
    __IO float  LastError;   /*Error[-1]*/
    __IO float  PrevError;   /*Error[-2]*/
    __IO float  IngMin;
    __IO float  IngMax;
    __IO float  OutMin;
    __IO float  OutMax;
} PID_TypeDef;

extern PID_TypeDef  g_location_pid;       /*λ��PID�����ṹ��*/
extern PID_TypeDef  g_speed_pid;       /*λ��PID�����ṹ��*/
/******************************************************************************************/
/* �ⲿ�ӿں���*/
void pid_init(void);
int32_t increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value);
#endif
