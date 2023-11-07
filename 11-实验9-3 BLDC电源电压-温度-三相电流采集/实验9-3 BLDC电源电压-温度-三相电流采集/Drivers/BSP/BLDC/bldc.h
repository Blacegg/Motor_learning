/**
 ****************************************************************************************************
 * @file        bldc.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       BLDC 驱动代码
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
#ifndef __BLDC_H
#define __BLDC_H

#include "./SYSTEM/sys/sys.h"

/***************************************** 电机状态结构体 *************************************************/
typedef struct 
{
    __IO uint8_t    run_flag;       /* 运行标志 */
    __IO uint8_t    locked_rotor;   /* 堵转标记*/     
    __IO uint8_t    step_sta;       /* 本次霍尔状态 */
    __IO uint8_t    hall_single_sta;/* 单个霍尔状态 */
    __IO uint8_t    hall_sta_edge;  /* 单个霍尔状态跳变 */
    __IO uint8_t    step_last;      /* 上次霍尔状态 */
    __IO uint8_t    dir;            /* 电机旋转方向 */
    __IO int32_t    pos;            /* 电机位置 */
    __IO int32_t    speed;          /* 电机速度 */
    __IO int16_t    current;        /* 电机速度 */
    __IO uint16_t   pwm_duty;       /* 电机占空比 */
    __IO uint32_t   hall_keep_t;    /* 霍尔保持时间 */
    __IO uint32_t   hall_pul_num;   /* 霍尔传感器脉冲数 */
    __IO uint32_t   lock_time;      /* 电机堵转时间 */
    __IO uint32_t   no_single;
    __IO uint32_t   count_j;
    __IO uint64_t   sum_pos;
} _bldc_obj;

/******************************************************************************************/

#define MOTOR_1                     1
#define MOTOR_2                     2

extern _bldc_obj g_bldc_motor1;
/****************************************** 刹车引脚 **************************************************/

#define SHUTDOWN_PIN                        GPIO_PIN_10     
#define SHUTDOWN_PIN_GPIO                   GPIOF
#define SHUTDOWN_PIN_GPIO_CLK_ENABLE()      do{  __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)    /* PF口时钟使能 */

#define SHUTDOWN2_PIN                       GPIO_PIN_2    
#define SHUTDOWN2_PIN_GPIO                  GPIOF
#define SHUTDOWN2_PIN_GPIO_CLK_ENABLE()     do{  __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)    /* PF口时钟使能 */

#define SHUTDOWN_EN                         HAL_GPIO_WritePin(SHUTDOWN_PIN_GPIO,SHUTDOWN_PIN,GPIO_PIN_SET);
#define SHUTDOWN_OFF                        HAL_GPIO_WritePin(SHUTDOWN_PIN_GPIO,SHUTDOWN_PIN,GPIO_PIN_RESET);

#define SHUTDOWN2_EN                        HAL_GPIO_WritePin(SHUTDOWN2_PIN_GPIO,SHUTDOWN2_PIN,GPIO_PIN_SET);
#define SHUTDOWN2_OFF                       HAL_GPIO_WritePin(SHUTDOWN2_PIN_GPIO,SHUTDOWN2_PIN,GPIO_PIN_RESET);

/***************************************** 霍尔传感器接口 *************************************************/

#define HALL1_TIM_CH1_PIN            GPIO_PIN_10     /* U */
#define HALL1_TIM_CH1_GPIO           GPIOH
#define HALL1_U_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)    /* PH口时钟使能 */

#define HALL1_TIM_CH2_PIN            GPIO_PIN_11     /* V */
#define HALL1_TIM_CH2_GPIO           GPIOH
#define HALL1_V_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)    /* PH口时钟使能 */

#define HALL1_TIM_CH3_PIN            GPIO_PIN_12     /* W */
#define HALL1_TIM_CH3_GPIO           GPIOH
#define HALL1_W_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)    /* PH口时钟使能 */


#define HALL2_TIM_CH1_PIN            GPIO_PIN_12     /* U */
#define HALL2_TIM_CH1_GPIO           GPIOD
#define HALL2_U_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)    /* PD口时钟使能 */

#define HALL2_TIM_CH2_PIN            GPIO_PIN_13     /* V */
#define HALL2_TIM_CH2_GPIO           GPIOD
#define HALL2_V_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)    /* PD口时钟使能 */

#define HALL2_TIM_CH3_PIN            GPIO_PIN_8      /* W */
#define HALL2_TIM_CH3_GPIO           GPIOB
#define HALL2_W_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    /* PB口时钟使能 */




/***************************************** 相关系数 ***************************************************/
#define MAX_PWM_DUTY    (((10000) - 1)*0.96)

#define H_PWM_L_ON

#define CCW                         (1)                 /* 逆时针 */
#define CW                          (2)                 /* 顺时针 */
#define HALL_ERROR                  (0xF0)              /* 霍尔错误标志 */
#define RUN                         (1)                 /* 电机运动标志 */
#define STOP                        (0)                 /* 电机停机标志 */

#define SPEED_COEFF      (uint32_t)((18000/4)*60)       /* 旋转一圈变化4个信号 */

#define NUM_CLEAR(para,val)     {if(para >= val){para=0;}}
#define NUM_MAX_LIMIT(para,val) {if(para > val){para=val;}}
#define NUM_MIN_LIMIT(para,val) {if(para < val){para=val;}}

typedef void(*pctr) (void);

#define ADC2CURT    (float)(3.3f / 4.096f / 0.12f)      /* ADC采集值 * 3.3/4.096 （mv） = 6 * ( 0.02*I ) */
#define ADC2VBUS    (float)(3.3f * 25 / 4096)           /* ADC采集值 * 3.3/4096 （V）= POWER/(12K+12K+1K)*1K */


/* 电机参数结构体 */
typedef struct {
    uint8_t state;                      /* 电机状态 */
    float current;                      /* 电机电流 */
    float volatage;                     /* 电机电压 */
    float power;                        /* 电机功率 */
    float speed;                        /* 电机实际速度 */
    int32_t motor_pwm;                  /* 设置比较值大小 */
} Motor_TypeDef;

extern Motor_TypeDef  g_motor_data;     /* 电机参数变量 */

void stop_motor1(void);
void start_motor1(void);

#define FirstOrderRC_LPF(Yn_1,Xn,a) Yn_1 = (1-a)*Yn_1 + a*Xn; /* Yn:out;Xn:in;a:系数 */

/****************************************** 函数声明 ************************************************/

void bldc_init(uint16_t arr, uint16_t psc);                 /* BLDC初始化 */
uint8_t check_hall_dir(_bldc_obj * obj);                    /* 检测电机旋转方向 */
extern pctr pfunclist_m1[6];                                /* 六步换相函数指针数组 */
void bldc_ctrl(uint8_t motor_id,int32_t dir,float duty);    /* bldc控制函数 */
uint8_t uemf_edge(uint8_t val);                             /* 波形状态检测 */
float get_temp(uint16_t para);                              /* 获取温度值 */
void calc_adc_val(uint16_t * p);                            /* adc数据软件滤波函数 */
void hall_gpio_init(void);                                  /* 霍尔接口初始化 */
uint32_t hallsensor_get_state(uint8_t motor_id);            /* 获取霍尔状态 */

/*  六步换相 */
void m1_uhvl(void);
void m1_uhwl(void);
void m1_vhwl(void);
void m1_vhul(void);
void m1_whul(void);
void m1_whvl(void);

#endif
