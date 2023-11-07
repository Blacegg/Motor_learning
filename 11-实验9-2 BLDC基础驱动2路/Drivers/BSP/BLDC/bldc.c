/**
 ****************************************************************************************************
 * @file        bldc.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       BLDC 驱动代码
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
 
#include "./BSP/BLDC/bldc.h"
#include "./BSP/TIMER/bldc_tim.h"

_bldc_obj g_bldc_motor1 = {STOP,0,0,CCW,0,0,0,0,0,0};   /* 电机结构体 */
_bldc_obj g_bldc_motor2 = {STOP,0,0,CCW,0,0,0,0,0,0};   /* 电机结构体 */

const uint8_t hall_table_cw[6] = {6,2,3,1,5,4};         /* 顺时针旋转表 */
const uint8_t hall_table_ccw[6] = {5,1,3,2,6,4};        /* 逆时针旋转表 */

const uint8_t hall_cw_table[12] = {0x62,0x23,0x31,0x15,0x54,0x46,0x63,0x21,0x35,0x14,0x56,0x42};
const uint8_t hall_ccw_table[12] = {0x45,0x51,0x13,0x32,0x26,0x64,0x41,0x53,0x12,0x36,0x24,0x65};
/**
 * @brief       无刷电机初始化，包括定时器，编码器以及SD引脚初始化
 * @param       arr: 自动重装值
 * @param       psc: 时钟预分频数
 * @retval      无
 */
void bldc_init(uint16_t arr, uint16_t psc)
{       
    GPIO_InitTypeDef gpio_init_struct;
    
    SHUTDOWN_PIN_GPIO_CLK_ENABLE();
    SHUTDOWN2_PIN_GPIO_CLK_ENABLE();
  
    gpio_init_struct.Pin = SHUTDOWN_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SHUTDOWN_PIN_GPIO, &gpio_init_struct);    
    
    gpio_init_struct.Pin = SHUTDOWN2_PIN;
    HAL_GPIO_Init(SHUTDOWN2_PIN_GPIO, &gpio_init_struct);  
    hall_gpio_init();               /* 霍尔接口初始化 */
    atim_timx_oc_chy_init(arr,  psc);
    atim_timx2_oc_chy_init(arr,  psc);
}

/**
 * @brief       BLDC控制函数
 * @param       dir :电机方向, Duty:PWM占空比
 * @retval      无
 */
void bldc_ctrl(uint8_t motor_id,int32_t dir,float duty)
{
    if(motor_id == MOTOR_1)
    {
        g_bldc_motor1.dir = dir;            /* 方向 */
        g_bldc_motor1.pwm_duty = duty;      /* 占空比 */
    }
    if(motor_id == MOTOR_2)
    {
        g_bldc_motor2.dir = dir;            /* 方向 */
        g_bldc_motor2.pwm_duty = duty;      /* 占空比 */
    }
}
/**
 * @brief       方向检测函数
 * @param       obj ： 电机控制句柄
 * @retval      res ： 旋转方向
 */
uint8_t check_hall_dir(_bldc_obj * obj)
{
    uint8_t temp,res = HALL_ERROR;
    if((obj->step_last <= 6)&&(obj->step_sta <= 6))
    {
        temp = ((obj->step_last & 0x0F) << 4)|(obj->step_sta & 0x0F);
        if((temp == hall_ccw_table[0])||(temp == hall_ccw_table[1])||\
                (temp == hall_ccw_table[2])||(temp == hall_ccw_table[3])||\
                (temp == hall_ccw_table[4])||(temp == hall_ccw_table[5]))
        {
            res  = CCW;
        }
        else if((temp == hall_cw_table[0])||(temp == hall_cw_table[1])||\
                (temp == hall_cw_table[2])||(temp == hall_cw_table[3])||\
                (temp == hall_cw_table[4])||(temp == hall_cw_table[5]))
        {
            res  = CW;
        }
    }
    return res;
}
/**************************************** 霍尔接口初始化 *************************************************/

/**
  * @brief  霍尔传感器定时器初始化
  * @param  无
  * @retval 无
  */
void hall_gpio_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    HALL1_U_GPIO_CLK_ENABLE();
    HALL1_V_GPIO_CLK_ENABLE();
    HALL1_W_GPIO_CLK_ENABLE();

    HALL2_U_GPIO_CLK_ENABLE();
    HALL2_V_GPIO_CLK_ENABLE();
    HALL2_W_GPIO_CLK_ENABLE();

    gpio_init_struct.Pin = HALL1_TIM_CH1_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(HALL1_TIM_CH1_GPIO, &gpio_init_struct);

    /* 定时器通道 2 引脚初始化 */
    gpio_init_struct.Pin = HALL1_TIM_CH2_PIN;
    HAL_GPIO_Init(HALL1_TIM_CH2_GPIO, &gpio_init_struct);

    /* 定时器通道 3 引脚初始化 */
    gpio_init_struct.Pin = HALL1_TIM_CH3_PIN;
    HAL_GPIO_Init(HALL1_TIM_CH3_GPIO, &gpio_init_struct);
    
    gpio_init_struct.Pin = HALL2_TIM_CH1_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(HALL2_TIM_CH1_GPIO, &gpio_init_struct);

    /* 定时器通道 2 引脚初始化 */
    gpio_init_struct.Pin = HALL2_TIM_CH2_PIN;
    HAL_GPIO_Init(HALL2_TIM_CH2_GPIO, &gpio_init_struct);

    /* 定时器通道 3 引脚初始化 */
    gpio_init_struct.Pin = HALL2_TIM_CH3_PIN;
    HAL_GPIO_Init(HALL2_TIM_CH3_GPIO, &gpio_init_struct);
}

/**
 * @brief       获取霍尔传感器引脚状态
 * @param       motor_id ：无刷接口编号
 * @retval      霍尔传感器引脚状态
 */
uint32_t hallsensor_get_state(uint8_t motor_id)
{
    __IO static uint32_t state ;
    state  = 0;
    if(motor_id == MOTOR_1)
    {
        if(HAL_GPIO_ReadPin(HALL1_TIM_CH1_GPIO,HALL1_TIM_CH1_PIN) != GPIO_PIN_RESET)  /* 霍尔传感器状态获取 */
        {
            state |= 0x01U;
        }
        if(HAL_GPIO_ReadPin(HALL1_TIM_CH2_GPIO,HALL1_TIM_CH2_PIN) != GPIO_PIN_RESET)  /* 霍尔传感器状态获取 */
        {
            state |= 0x02U;
        }
        if(HAL_GPIO_ReadPin(HALL1_TIM_CH3_GPIO,HALL1_TIM_CH3_PIN) != GPIO_PIN_RESET)  /* 霍尔传感器状态获取 */
        {
            state |= 0x04U;
        }
    }
    else if(motor_id == MOTOR_2)
    {
        if(HAL_GPIO_ReadPin(HALL2_TIM_CH1_GPIO,HALL2_TIM_CH1_PIN) != GPIO_PIN_RESET)  /* 霍尔传感器状态获取 */
        {
            state |= 0x01U;
        }
        if(HAL_GPIO_ReadPin(HALL2_TIM_CH2_GPIO,HALL2_TIM_CH2_PIN) != GPIO_PIN_RESET)  /* 霍尔传感器状态获取 */
        {
            state |= 0x02U;
        }
        if(HAL_GPIO_ReadPin(HALL2_TIM_CH3_GPIO,HALL2_TIM_CH3_PIN) != GPIO_PIN_RESET)  /* 霍尔传感器状态获取 */
        {
            state |= 0x04U;
        }
    }
    return state;
}
/************************************* BLDC相关函数 *************************************/

/**
  * @brief  关闭电机运转
  * @param  无
  * @retval 无
  */
void stop_motor1(void)
{
    /* 关闭半桥芯片输出 */
    SHUTDOWN_OFF;
    /* 关闭PWM输出 */
    HAL_TIM_PWM_Stop(&g_atimx_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&g_atimx_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&g_atimx_handle,TIM_CHANNEL_3);
    /* 上下桥臂全部关断 */
    g_atimx_handle.Instance->CCR2 = 0;
    g_atimx_handle.Instance->CCR1 = 0;
    g_atimx_handle.Instance->CCR3 = 0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_RESET);
}

/* 开启电机运转 */
void start_motor1(void)
{
    SHUTDOWN_EN;
    /* 使能PWM输出 */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_3);
}

/*************************** 上下桥臂的导通情况，共6种，也称为6步换向（接口一） ****************************/

/*  六步换向函数指针 */
pctr pfunclist_m1[6] =
{
    &m1_uhwl, &m1_vhul, &m1_vhwl,
    &m1_whvl, &m1_uhvl, &m1_whul
};

/* 上下桥臂的导通情况，共6种，也称为6步换向 */
void m1_uhvl(void)
{
    g_atimx_handle.Instance->CCR2 = 0;
    g_atimx_handle.Instance->CCR1 = g_bldc_motor1.pwm_duty;                 /* U相上桥臂PWM */
    g_atimx_handle.Instance->CCR3 = 0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_SET);   /* V相下桥臂导通 */
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_RESET); /* U相下桥臂关闭 */
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_RESET); /* W相下桥臂关闭 */
}

void m1_uhwl(void)
{
    g_atimx_handle.Instance->CCR2 = 0;
    g_atimx_handle.Instance->CCR1 = g_bldc_motor1.pwm_duty;
    g_atimx_handle.Instance->CCR3 = 0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_RESET);
}

void m1_vhwl(void)
{
    g_atimx_handle.Instance->CCR1=0;
    g_atimx_handle.Instance->CCR2 = g_bldc_motor1.pwm_duty;
    g_atimx_handle.Instance->CCR3=0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_RESET);
}

void m1_vhul(void)
{
    g_atimx_handle.Instance->CCR1 = 0;
    g_atimx_handle.Instance->CCR2 = g_bldc_motor1.pwm_duty;
    g_atimx_handle.Instance->CCR3 = 0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_RESET);
}


void m1_whul(void)
{
    g_atimx_handle.Instance->CCR2 = 0;
    g_atimx_handle.Instance->CCR3 = g_bldc_motor1.pwm_duty;
    g_atimx_handle.Instance->CCR1 = 0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_RESET);
}

void m1_whvl(void)
{
    g_atimx_handle.Instance->CCR2 = 0;
    g_atimx_handle.Instance->CCR3 = g_bldc_motor1.pwm_duty;
    g_atimx_handle.Instance->CCR1 = 0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_RESET);
}

/*************************** 上下桥臂的导通情况，共6种，也称为6步换向（接口二） ****************************/

pctr pfunclist_m2[6] =
{
    &m2_uhwl, &m2_vhul, &m2_vhwl,
    &m2_whvl, &m2_uhvl, &m2_whul
};


void stop_motor2(void)
{
    SHUTDOWN2_OFF;
    HAL_TIM_PWM_Stop(&g_atimx2_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&g_atimx2_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&g_atimx2_handle,TIM_CHANNEL_3);
    HAL_GPIO_WritePin(M2_LOW_SIDE_U_PORT,M2_LOW_SIDE_U_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_LOW_SIDE_V_PORT,M2_LOW_SIDE_V_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_LOW_SIDE_W_PORT,M2_LOW_SIDE_W_PIN,GPIO_PIN_RESET);
}

void start_motor2(void)
{
    SHUTDOWN2_EN;
    HAL_TIM_PWM_Start(&g_atimx2_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&g_atimx2_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&g_atimx2_handle,TIM_CHANNEL_3);
}

void m2_uhvl(void)
{
    g_atimx2_handle.Instance->CCR2 = 0;
    g_atimx2_handle.Instance->CCR1 = g_bldc_motor2.pwm_duty;/*U相上桥臂PWM*/
    g_atimx2_handle.Instance->CCR3 = 0;
    HAL_GPIO_WritePin(M2_LOW_SIDE_V_PORT,M2_LOW_SIDE_V_PIN,GPIO_PIN_SET);/*V相下桥臂导通*/
    HAL_GPIO_WritePin(M2_LOW_SIDE_U_PORT,M2_LOW_SIDE_U_PIN,GPIO_PIN_RESET);/*U相下桥臂关闭*/
    HAL_GPIO_WritePin(M2_LOW_SIDE_W_PORT,M2_LOW_SIDE_W_PIN,GPIO_PIN_RESET);/*W相下桥臂关闭*/
}

void m2_uhwl(void)
{
    g_atimx2_handle.Instance->CCR2 = 0;
    g_atimx2_handle.Instance->CCR1 = g_bldc_motor2.pwm_duty;
    g_atimx2_handle.Instance->CCR3 = 0;
    HAL_GPIO_WritePin(M2_LOW_SIDE_W_PORT,M2_LOW_SIDE_W_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_LOW_SIDE_U_PORT,M2_LOW_SIDE_U_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_LOW_SIDE_V_PORT,M2_LOW_SIDE_V_PIN,GPIO_PIN_RESET);
}

void m2_vhwl(void)
{
    g_atimx2_handle.Instance->CCR1=0;
    g_atimx2_handle.Instance->CCR2 = g_bldc_motor2.pwm_duty;
    g_atimx2_handle.Instance->CCR3=0;
    HAL_GPIO_WritePin(M2_LOW_SIDE_W_PORT,M2_LOW_SIDE_W_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_LOW_SIDE_U_PORT,M2_LOW_SIDE_U_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_LOW_SIDE_V_PORT,M2_LOW_SIDE_V_PIN,GPIO_PIN_RESET);
}

void m2_vhul(void)
{
    g_atimx2_handle.Instance->CCR1 = 0;
    g_atimx2_handle.Instance->CCR2 = g_bldc_motor2.pwm_duty;
    g_atimx2_handle.Instance->CCR3 = 0;
    HAL_GPIO_WritePin(M2_LOW_SIDE_U_PORT,M2_LOW_SIDE_U_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_LOW_SIDE_V_PORT,M2_LOW_SIDE_V_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_LOW_SIDE_W_PORT,M2_LOW_SIDE_W_PIN,GPIO_PIN_RESET);
}


void m2_whul(void)
{
    g_atimx2_handle.Instance->CCR2 = 0;
    g_atimx2_handle.Instance->CCR3 = g_bldc_motor2.pwm_duty;
    g_atimx2_handle.Instance->CCR1 = 0;
    HAL_GPIO_WritePin(M2_LOW_SIDE_U_PORT,M2_LOW_SIDE_U_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_LOW_SIDE_V_PORT,M2_LOW_SIDE_V_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_LOW_SIDE_W_PORT,M2_LOW_SIDE_W_PIN,GPIO_PIN_RESET);
}

void m2_whvl(void)
{
    g_atimx2_handle.Instance->CCR2 = 0;
    g_atimx2_handle.Instance->CCR3 = g_bldc_motor2.pwm_duty;
    g_atimx2_handle.Instance->CCR1 = 0;
    HAL_GPIO_WritePin(M2_LOW_SIDE_V_PORT,M2_LOW_SIDE_V_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_LOW_SIDE_U_PORT,M2_LOW_SIDE_U_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_LOW_SIDE_W_PORT,M2_LOW_SIDE_W_PIN,GPIO_PIN_RESET);
}

