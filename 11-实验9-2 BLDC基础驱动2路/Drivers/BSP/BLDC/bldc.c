/**
 ****************************************************************************************************
 * @file        bldc.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       BLDC ��������
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
 
#include "./BSP/BLDC/bldc.h"
#include "./BSP/TIMER/bldc_tim.h"

_bldc_obj g_bldc_motor1 = {STOP,0,0,CCW,0,0,0,0,0,0};   /* ����ṹ�� */
_bldc_obj g_bldc_motor2 = {STOP,0,0,CCW,0,0,0,0,0,0};   /* ����ṹ�� */

const uint8_t hall_table_cw[6] = {6,2,3,1,5,4};         /* ˳ʱ����ת�� */
const uint8_t hall_table_ccw[6] = {5,1,3,2,6,4};        /* ��ʱ����ת�� */

const uint8_t hall_cw_table[12] = {0x62,0x23,0x31,0x15,0x54,0x46,0x63,0x21,0x35,0x14,0x56,0x42};
const uint8_t hall_ccw_table[12] = {0x45,0x51,0x13,0x32,0x26,0x64,0x41,0x53,0x12,0x36,0x24,0x65};
/**
 * @brief       ��ˢ�����ʼ����������ʱ�����������Լ�SD���ų�ʼ��
 * @param       arr: �Զ���װֵ
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
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
    hall_gpio_init();               /* �����ӿڳ�ʼ�� */
    atim_timx_oc_chy_init(arr,  psc);
    atim_timx2_oc_chy_init(arr,  psc);
}

/**
 * @brief       BLDC���ƺ���
 * @param       dir :�������, Duty:PWMռ�ձ�
 * @retval      ��
 */
void bldc_ctrl(uint8_t motor_id,int32_t dir,float duty)
{
    if(motor_id == MOTOR_1)
    {
        g_bldc_motor1.dir = dir;            /* ���� */
        g_bldc_motor1.pwm_duty = duty;      /* ռ�ձ� */
    }
    if(motor_id == MOTOR_2)
    {
        g_bldc_motor2.dir = dir;            /* ���� */
        g_bldc_motor2.pwm_duty = duty;      /* ռ�ձ� */
    }
}
/**
 * @brief       �����⺯��
 * @param       obj �� ������ƾ��
 * @retval      res �� ��ת����
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
/**************************************** �����ӿڳ�ʼ�� *************************************************/

/**
  * @brief  ������������ʱ����ʼ��
  * @param  ��
  * @retval ��
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

    /* ��ʱ��ͨ�� 2 ���ų�ʼ�� */
    gpio_init_struct.Pin = HALL1_TIM_CH2_PIN;
    HAL_GPIO_Init(HALL1_TIM_CH2_GPIO, &gpio_init_struct);

    /* ��ʱ��ͨ�� 3 ���ų�ʼ�� */
    gpio_init_struct.Pin = HALL1_TIM_CH3_PIN;
    HAL_GPIO_Init(HALL1_TIM_CH3_GPIO, &gpio_init_struct);
    
    gpio_init_struct.Pin = HALL2_TIM_CH1_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(HALL2_TIM_CH1_GPIO, &gpio_init_struct);

    /* ��ʱ��ͨ�� 2 ���ų�ʼ�� */
    gpio_init_struct.Pin = HALL2_TIM_CH2_PIN;
    HAL_GPIO_Init(HALL2_TIM_CH2_GPIO, &gpio_init_struct);

    /* ��ʱ��ͨ�� 3 ���ų�ʼ�� */
    gpio_init_struct.Pin = HALL2_TIM_CH3_PIN;
    HAL_GPIO_Init(HALL2_TIM_CH3_GPIO, &gpio_init_struct);
}

/**
 * @brief       ��ȡ��������������״̬
 * @param       motor_id ����ˢ�ӿڱ��
 * @retval      ��������������״̬
 */
uint32_t hallsensor_get_state(uint8_t motor_id)
{
    __IO static uint32_t state ;
    state  = 0;
    if(motor_id == MOTOR_1)
    {
        if(HAL_GPIO_ReadPin(HALL1_TIM_CH1_GPIO,HALL1_TIM_CH1_PIN) != GPIO_PIN_RESET)  /* ����������״̬��ȡ */
        {
            state |= 0x01U;
        }
        if(HAL_GPIO_ReadPin(HALL1_TIM_CH2_GPIO,HALL1_TIM_CH2_PIN) != GPIO_PIN_RESET)  /* ����������״̬��ȡ */
        {
            state |= 0x02U;
        }
        if(HAL_GPIO_ReadPin(HALL1_TIM_CH3_GPIO,HALL1_TIM_CH3_PIN) != GPIO_PIN_RESET)  /* ����������״̬��ȡ */
        {
            state |= 0x04U;
        }
    }
    else if(motor_id == MOTOR_2)
    {
        if(HAL_GPIO_ReadPin(HALL2_TIM_CH1_GPIO,HALL2_TIM_CH1_PIN) != GPIO_PIN_RESET)  /* ����������״̬��ȡ */
        {
            state |= 0x01U;
        }
        if(HAL_GPIO_ReadPin(HALL2_TIM_CH2_GPIO,HALL2_TIM_CH2_PIN) != GPIO_PIN_RESET)  /* ����������״̬��ȡ */
        {
            state |= 0x02U;
        }
        if(HAL_GPIO_ReadPin(HALL2_TIM_CH3_GPIO,HALL2_TIM_CH3_PIN) != GPIO_PIN_RESET)  /* ����������״̬��ȡ */
        {
            state |= 0x04U;
        }
    }
    return state;
}
/************************************* BLDC��غ��� *************************************/

/**
  * @brief  �رյ����ת
  * @param  ��
  * @retval ��
  */
void stop_motor1(void)
{
    /* �رհ���оƬ��� */
    SHUTDOWN_OFF;
    /* �ر�PWM��� */
    HAL_TIM_PWM_Stop(&g_atimx_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&g_atimx_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&g_atimx_handle,TIM_CHANNEL_3);
    /* �����ű�ȫ���ض� */
    g_atimx_handle.Instance->CCR2 = 0;
    g_atimx_handle.Instance->CCR1 = 0;
    g_atimx_handle.Instance->CCR3 = 0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_RESET);
}

/* ���������ת */
void start_motor1(void)
{
    SHUTDOWN_EN;
    /* ʹ��PWM��� */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_3);
}

/*************************** �����ű۵ĵ�ͨ�������6�֣�Ҳ��Ϊ6�����򣨽ӿ�һ�� ****************************/

/*  ����������ָ�� */
pctr pfunclist_m1[6] =
{
    &m1_uhwl, &m1_vhul, &m1_vhwl,
    &m1_whvl, &m1_uhvl, &m1_whul
};

/* �����ű۵ĵ�ͨ�������6�֣�Ҳ��Ϊ6������ */
void m1_uhvl(void)
{
    g_atimx_handle.Instance->CCR2 = 0;
    g_atimx_handle.Instance->CCR1 = g_bldc_motor1.pwm_duty;                 /* U�����ű�PWM */
    g_atimx_handle.Instance->CCR3 = 0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_SET);   /* V�����ű۵�ͨ */
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_RESET); /* U�����ű۹ر� */
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_RESET); /* W�����ű۹ر� */
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

/*************************** �����ű۵ĵ�ͨ�������6�֣�Ҳ��Ϊ6�����򣨽ӿڶ��� ****************************/

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
    g_atimx2_handle.Instance->CCR1 = g_bldc_motor2.pwm_duty;/*U�����ű�PWM*/
    g_atimx2_handle.Instance->CCR3 = 0;
    HAL_GPIO_WritePin(M2_LOW_SIDE_V_PORT,M2_LOW_SIDE_V_PIN,GPIO_PIN_SET);/*V�����ű۵�ͨ*/
    HAL_GPIO_WritePin(M2_LOW_SIDE_U_PORT,M2_LOW_SIDE_U_PIN,GPIO_PIN_RESET);/*U�����ű۹ر�*/
    HAL_GPIO_WritePin(M2_LOW_SIDE_W_PORT,M2_LOW_SIDE_W_PIN,GPIO_PIN_RESET);/*W�����ű۹ر�*/
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

