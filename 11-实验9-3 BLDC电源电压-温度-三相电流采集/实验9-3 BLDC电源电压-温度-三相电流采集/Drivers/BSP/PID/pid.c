/**
 ****************************************************************************************************
 * @file        pid.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       PID算法代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
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

#include "./BSP/PID/pid.h"
#include "./BSP/BLDC/bldc.h"

PID_TypeDef  g_location_pid;       /*位置PID参数结构体*/
PID_TypeDef  g_speed_pid;       /*位置PID参数结构体*/
/**
 * @brief       初始化LED相关IO口, 并使能时钟
 * @param       无
 * @retval      无
 */
void pid_init(void)
{
    g_speed_pid.SetPoint = 0; /* 设定目标Desired Value*/
    g_speed_pid.ActualValue = 0.0; /*  设定目标Desired Value*/
    g_speed_pid.SumError = 0.0; /* 积分值*/
    g_speed_pid.Error = 0.0;   /*  Error[1]*/
    g_speed_pid.LastError = 0.0; /*  Error[-1]*/
    g_speed_pid.PrevError = 0.0; /*  Error[-2]*/
    g_speed_pid.Proportion = KP; /*  比例常数 Proportional Const*/
    g_speed_pid.Integral = KI; /*  积分常数 Integral Const*/
    g_speed_pid.Derivative = KD; /*  微分常数 Derivative Const*/ 
    g_speed_pid.IngMax = 20;
    g_speed_pid.IngMin = -20;
    g_speed_pid.OutMax = 150;                  /* 输出限制 */
    g_speed_pid.OutMin = -150;    
}


/**
  * 函数名称：位置闭环PID控制设计
  * 输入参数：当前控制量
  * 返 回 值：目标控制量
  * 说    明：无
  */
int32_t increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value)
{
    PID->Error = (float)(PID->SetPoint - Feedback_value);   /*速度档位偏差*/
#if  INCR_LOCT_SELECT
    PID->ActualValue += (PID->Proportion * (PID->Error - PID->LastError)) /*E[k]项*/
                        + (PID->Integral * PID->Error)              /*E[k-1]项*/
                        + (PID->Derivative * (PID->Error - 2 * PID->LastError + PID->PrevError)); /*E[k-2]项*/
    PID->PrevError = PID->LastError;                      /*存储误差，用于下次计算*/
    PID->LastError = PID->Error;
#else
    PID->SumError += PID->Error;
    PID->ActualValue = (PID->Proportion * PID->Error) /*E[k]项*/
                       + (PID->Integral * PID->SumError)              /*E[k-1]项*/
                       + (PID->Derivative * (PID->Error - PID->LastError)); /*E[k-2]项*/
    PID->LastError = PID->Error;
#endif
    return ((int32_t)(PID->ActualValue));                /*返回实际控制数值*/
    

}
