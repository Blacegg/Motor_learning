/**
 ****************************************************************************************************
 * @file        pid.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       PID代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211014
 * 第一次发布
 *
 ****************************************************************************************************
 */
#ifndef __PID_H
#define __PID_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* PID相关参数 */

#define  INCR_LOCT_SELECT  1/*0选择位置式1：增量式控制*/
#if INCR_LOCT_SELECT
/*定义PID参数相关宏*/
#define  KP      8.50f       /* P参数*/
#define  KI      5.00f       /* I参数*/
#define  KD      0.08f       /* D参数*/
#define SMAPLSE_PID_SPEED  50        /*采样率 单位ms*/
#else
/*定义PID参数相关宏*/
#define  KP      10.0f       /* P参数*/
#define  KI      8.00f       /* I参数*/
#define  KD      0.5f        /* D参数*/
#define SMAPLSE_PID_SPEED  50        /*采样率 单位ms*/
#endif

/*定义位置PID参数相关宏*/
/*PID结构体*/
typedef struct
{
    __IO float  SetPoint;    /*设定目标 */
    __IO float  ActualValue; /*实际值*/
    __IO float  SumError;    /*误差累计*/
    __IO float  Proportion;  /*比例常数 P*/
    __IO float  Integral;    /*积分常数 I*/
    __IO float  Derivative;  /*微分常数 D*/
    __IO float  Error;       /*Error[-1]*/
    __IO float  LastError;   /*Error[-1]*/
    __IO float  PrevError;   /*Error[-2]*/
    __IO float  IngMin;
    __IO float  IngMax;
    __IO float  OutMin;
    __IO float  OutMax;
} PID_TypeDef;

extern PID_TypeDef  g_location_pid;       /*位置PID参数结构体*/
extern PID_TypeDef  g_speed_pid;       /*位置PID参数结构体*/
/******************************************************************************************/
/* 外部接口函数*/
void pid_init(void);
int32_t increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value);
#endif
