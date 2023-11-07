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

/******************************************************************************************/
typedef struct {
    __IO uint8_t    run_flag;       /* 运行标志 */
    __IO uint8_t    locked_rotor;   /* 堵转标记 */
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
} _bldc_obj;

/******************************************************************************************/
#define MOTOR_1                     1
#define MOTOR_2                     2

extern _bldc_obj g_bldc_motor1;
extern _bldc_obj g_bldc_motor2;
/********************************************************************************************/
/*刹车引脚*/
#define SHUTDOWN_PIN                      GPIO_PIN_10   
#define SHUTDOWN_PIN_GPIO                 GPIOF
#define SHUTDOWN_PIN_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)    /* PF口时钟使能 */

#define SHUTDOWN2_PIN                     GPIO_PIN_2    
#define SHUTDOWN2_PIN_GPIO                GPIOF
#define SHUTDOWN2_PIN_GPIO_CLK_ENABLE()   do{  __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)    /* PF口时钟使能 */

#define SHUTDOWN_EN                       HAL_GPIO_WritePin(SHUTDOWN_PIN_GPIO,SHUTDOWN_PIN,GPIO_PIN_SET);
#define SHUTDOWN_OFF                      HAL_GPIO_WritePin(SHUTDOWN_PIN_GPIO,SHUTDOWN_PIN,GPIO_PIN_RESET);


#define SHUTDOWN2_EN                       HAL_GPIO_WritePin(SHUTDOWN2_PIN_GPIO,SHUTDOWN2_PIN,GPIO_PIN_SET);
#define SHUTDOWN2_OFF                      HAL_GPIO_WritePin(SHUTDOWN2_PIN_GPIO,SHUTDOWN2_PIN,GPIO_PIN_RESET);
/******************************************************************************************/
/*霍尔传感器接口*/

#define HALL1_TIM_CH1_PIN           GPIO_PIN_10     /* U */
#define HALL1_TIM_CH1_GPIO          GPIOH
#define HALL1_U_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)    /* PH口时钟使能 */

#define HALL1_TIM_CH2_PIN           GPIO_PIN_11     /* V */
#define HALL1_TIM_CH2_GPIO          GPIOH
#define HALL1_V_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)    /* PH口时钟使能 */

#define HALL1_TIM_CH3_PIN           GPIO_PIN_12     /* W */
#define HALL1_TIM_CH3_GPIO          GPIOH
#define HALL1_W_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)    /* PH口时钟使能 */


#define HALL2_TIM_CH1_PIN           GPIO_PIN_12     /* U */
#define HALL2_TIM_CH1_GPIO          GPIOD
#define HALL2_U_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)    /* PD口时钟使能 */

#define HALL2_TIM_CH2_PIN           GPIO_PIN_13     /* V */
#define HALL2_TIM_CH2_GPIO          GPIOD
#define HALL2_V_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)    /* PD口时钟使能 */

#define HALL2_TIM_CH3_PIN           GPIO_PIN_8      /* W */
#define HALL2_TIM_CH3_GPIO          GPIOB
#define HALL2_W_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    /* PB口时钟使能 */

/********************************************************************************************/
#define MAX_PWM_DUTY    (((10000) - 1)*0.96)

#define H_PWM_L_ON
#ifndef H_PWM_L_ON
#define H_PWM_L_PWM
#endif

#define CCW                         (1)                 /* 逆时针 */
#define CW                          (2)                 /* 顺时针 */
#define HALL_ERROR                  (0xF0)              /* 霍尔错误标志 */
#define RUN                         (1)                 /* 电机运动标志 */
#define STOP                        (0)                 /* 电机停机标志 */

typedef void(*pctr) (void);

void stop_motor1(void);                                 /* 停机 */
void start_motor1(void);                                /* 启动电机 */
void stop_motor2(void);
void start_motor2(void);

/******************************************************************************************/
/* 外部接口函数*/
void bldc_init(uint16_t arr, uint16_t psc);             /* BLDC初始化 */
uint8_t check_hall_dir(_bldc_obj * obj);                /* 检测电机旋转方向 */
void hall_gpio_init(void);                              /* 霍尔接口初始化 */
uint32_t hallsensor_get_state(uint8_t motor_id);        /* 获取霍尔状态 */

extern pctr pfunclist_m1[6];                            /* 六步换相函数指针数组 */
void bldc_ctrl(uint8_t motor_id,int32_t dir,float duty);/* bldc控制函数 */
extern pctr pfunclist_m2[6];                            /* 六步换相函数指针数组 */

/*  六步换相 */
void m1_uhvl(void);
void m1_uhwl(void);
void m1_vhwl(void);
void m1_vhul(void);
void m1_whul(void);
void m1_whvl(void);

void m2_uhvl(void);
void m2_uhwl(void);
void m2_vhwl(void);
void m2_vhul(void);
void m2_whul(void);
void m2_whvl(void);

#endif
