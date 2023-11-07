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
#include "./BSP/ADC/adc.h"
#include "math.h"

_bldc_obj g_bldc_motor1 = {STOP,0,0,CCW,0,0,0,0,0,0};   /* ����ṹ�� */

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

    gpio_init_struct.Pin = GPIO_PIN_6;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_RESET);
    
    gpio_init_struct.Pin = SHUTDOWN2_PIN;
    HAL_GPIO_Init(SHUTDOWN2_PIN_GPIO, &gpio_init_struct);
    hall_gpio_init();                       /* �����ӿڳ�ʼ�� */
    atim_timx_oc_chy_init(arr,  psc);
    btim_timx_int_init(1000-1,84-1);
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
/***************************************** �����ӿڳ�ʼ�� ************************************/

/**
  * @brief  �����������ӿڳ�ʼ��
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

    /* ����ͨ�� 1 ���ų�ʼ�� */
    gpio_init_struct.Pin = HALL1_TIM_CH1_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(HALL1_TIM_CH1_GPIO, &gpio_init_struct);

    /* ����ͨ�� 2 ���ų�ʼ�� */
    gpio_init_struct.Pin = HALL1_TIM_CH2_PIN;
    HAL_GPIO_Init(HALL1_TIM_CH2_GPIO, &gpio_init_struct);

    /* ����ͨ�� 3 ���ų�ʼ�� */
    gpio_init_struct.Pin = HALL1_TIM_CH3_PIN;
    HAL_GPIO_Init(HALL1_TIM_CH3_GPIO, &gpio_init_struct);
}


/**
 * @brief       ��ȡ��������������״̬
 * @param       motor_id �� ��ˢ�ӿڱ��
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
    return state;
}

/************************************* BLDC��غ��� *************************************/
/* �رյ����ת */
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

/*************************** �����ű۵ĵ�ͨ�������6�֣�Ҳ��Ϊ6������ ****************************/

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

/**
 * @brief       ��������ź��Ƿ����仯
 * @param       val :�����ź�
 * @note        �����ٶ�ʹ�ã���ȡ�����ź�״̬��ת����������ٶ�
 * @retval      0������ߵ�ƽʱ�䣬1������͵�ƽʱ�䣬2���ź�δ�ı�
 */
uint8_t uemf_edge(uint8_t val)
{
    /* ��Ҫ�Ǽ��val�źŴ�0 - 1 �ڴ� 1 - 0�Ĺ��̣����ߵ�ƽ�������Ĺ��� */
    static uint8_t oldval = 0;
    if(oldval != val)
    {
        oldval = val;
        if(val == 0) return 0;
        else return 1;
    }
    return 2;
}
/*************************************    �ڶ�����    ��ѹ�����¶Ȳɼ�    **********************************************/
/*
    Rt = Rp *exp(B*(1/T1-1/T2))
    Rt ������������T1�¶��µ���ֵ��
    Rp������������T2�����µı����ֵ��
    exp��e��n�η���e����Ȼ������������Ȼ�����ĵ��������Ƶ��� 2.7182818��
    Bֵ�������������Ҫ�������̳����õ�����������BֵΪ3380��
    ����T1��T2ָ���ǿ������¶ȣ�T2�ǳ���25�棬��(273.15+25)K
    T1����������¶�
*/

const float Rp = 10000.0f;                  /* 10K */
const float T2 = (273.15f + 25.0f);         /* T2 */
const float Bx = 3380.0f;                   /* B */
const float Ka = 273.15f;

/**
 * @brief       �����¶�ֵ
 * @param       para: �¶Ȳɼ���ӦADCͨ����ֵ�����˲���
 * @note        �����¶ȷ�Ϊ������
                1.����ADC�ɼ�����ֵ���㵱ǰ��Ӧ��Rt
                2.����Rt�����Ӧ���¶�ֵ
 * @retval      �¶�ֵ
 */
float get_temp(uint16_t para)
{
    float Rt;
    float temp;
    Rt = 3.3f * 4700.0f / (para * 3.3f / 4096.0f) - 4700.0f;
    /* like this R=5000, T2=273.15+25,B=3470, RT=5000*EXP(3470*(1/T1-1/(273.15+25)) */
    temp = Rt / Rp;
    temp = log(temp);       /* ln(Rt/Rp) */
    temp /= Bx;             /* ln(Rt/Rp)/B */
    temp += (1.0f / T2);
    temp = 1.0f / (temp);
    temp -= Ka;
    return temp;
}

extern uint16_t g_adc_value[ADC_CH_NUM * ADC_COLL];

/**
 * @brief       ����ADC��ƽ��ֵ���˲���
 * @param       * p �����ADCֵ��ָ���ַ
 * @note        �˺����Ե�ѹ���¶ȡ�������Ӧ��ADCֵ�����˲�
 * @retval      ��
 */
void calc_adc_val(uint16_t * p)
{
    uint32_t temp[ADC_CH_NUM] = {0,0,0};            /* ����һ���������� */
    int i,j;
    for(i = 0; i < ADC_COLL; i++)                   /* ѭ���ɼ�ADC_COLL���� */
    {
        for(j = 0; j < ADC_CH_NUM; j++)             /* ����ADCͨ����ѭ����ȡ�����ۼ� */
        {
            temp[j] += g_adc_value[j+i*ADC_CH_NUM]; /* ���ɼ�����ADCֵ����ͨ�������ۼ� */
        }
    }
    for(j = 0; j < ADC_CH_NUM; j++)
    {
        temp[j] /= ADC_COLL;                        /* ��ȡƽ��ֵ */
        p[j] = temp[j];                             /* �浽*p */
    }
}


