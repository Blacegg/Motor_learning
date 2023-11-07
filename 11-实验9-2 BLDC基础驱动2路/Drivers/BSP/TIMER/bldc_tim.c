/**
 ****************************************************************************************************
 * @file        bldc_tim.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-19
 * @brief       ��ʱ�� ��������
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
 * V1.0 20211019
 * ��һ�η���
 *
 ****************************************************************************************************
 */


#include "./BSP/TIMER/bldc_tim.h"
#include "./BSP/LED/led.h"
#include "./BSP/BLDC/bldc.h"

/******************************************************************************************/
/* ��ʱ�����þ�� ���� */

/* �߼���ʱ��PWM */
TIM_HandleTypeDef g_atimx_handle;               /* ��ʱ��x��� */
TIM_HandleTypeDef g_atimx2_handle;              /* ��ʱ��x��� */
TIM_OC_InitTypeDef g_atimx_oc_chy_handle;       /* ��ʱ�������� */
TIM_OC_InitTypeDef g_atimx2_oc_chy_handle;      /* ��ʱ�������� */

extern _bldc_obj g_bldc_motor1;
extern _bldc_obj g_bldc_motor2;
/******************************************************************************************/

/**
 * @brief       �߼���ʱ��TIM1 PWM��� ��ʼ������
 * @note
 *              �߼���ʱ����ʱ������APB2, ��PCLK2 = 168Mhz, ��������PPRE2����Ƶ, ���
 *              �߼���ʱ��ʱ�� = 168Mhz
 *              ��ʱ�����ʱ����㷽��: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=��ʱ������Ƶ��,��λ:Mhz
 *
 * @param       arr: �Զ���װֵ
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */
void atim_timx_oc_chy_init(uint16_t arr, uint16_t psc)
{
    ATIM_TIMX_PWM_CHY_CLK_ENABLE();                             /* TIMX ʱ��ʹ�� */


    g_atimx_handle.Instance = ATIM_TIMX_PWM;                    /* ��ʱ��x */
    g_atimx_handle.Init.Prescaler = psc;                        /* ��ʱ����Ƶ */
    g_atimx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;       /* ���ϼ���ģʽ */
    g_atimx_handle.Init.Period = arr;                           /* �Զ���װ��ֵ */
    g_atimx_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   /* ��Ƶ���� */
    g_atimx_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; /*ʹ��TIMx_ARR���л���*/
    g_atimx_handle.Init.RepetitionCounter = 0;                  /* ��ʼʱ������*/
    HAL_TIM_PWM_Init(&g_atimx_handle);                          /* ��ʼ��PWM */

    g_atimx_oc_chy_handle.OCMode = TIM_OCMODE_PWM1;             /* ģʽѡ��PWM1 */
    g_atimx_oc_chy_handle.Pulse = 0;
    g_atimx_oc_chy_handle.OCPolarity = TIM_OCPOLARITY_HIGH;     /* ����Ƚϼ���Ϊ�� */
    g_atimx_oc_chy_handle.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    g_atimx_oc_chy_handle.OCFastMode = TIM_OCFAST_DISABLE;
    g_atimx_oc_chy_handle.OCIdleState = TIM_OCIDLESTATE_RESET;
    g_atimx_oc_chy_handle.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH1); /* ����TIMxͨ��y */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH2); /* ����TIMxͨ��y */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH3); /* ����TIMxͨ��y */

    /* ������ʱ��ͨ��1���PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_1);

    /* ������ʱ��ͨ��2���PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_2);

    /* ������ʱ��ͨ��3���PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_3);

    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2, 2);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
 
    HAL_TIM_Base_Start_IT(&g_atimx_handle);/*�����߼���ʱ��1*/
}

/**
 * @brief       �߼���ʱ��TIM8 PWM��� ��ʼ������
 * @param       arr: �Զ���װֵ
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */
void atim_timx2_oc_chy_init(uint16_t arr, uint16_t psc)
{
    ATIM_TIMX2_PWM_CHY_CLK_ENABLE();                                /* TIMX ʱ��ʹ�� */


    g_atimx2_handle.Instance = ATIM_TIMX2_PWM;                      /* ��ʱ��x */
    g_atimx2_handle.Init.Prescaler = psc;                           /* ��ʱ����Ƶ */
    g_atimx2_handle.Init.CounterMode = TIM_COUNTERMODE_UP;          /* ���ϼ���ģʽ */
    g_atimx2_handle.Init.Period = arr;                              /* �Զ���װ��ֵ */
    g_atimx2_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;      /* ��Ƶ���� */
    g_atimx2_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; /*ʹ��TIMx_ARR���л���*/
    g_atimx2_handle.Init.RepetitionCounter = 0;                     /* ��ʼʱ������*/
    HAL_TIM_PWM_Init(&g_atimx2_handle);                             /* ��ʼ��PWM */

    g_atimx2_oc_chy_handle.OCMode = TIM_OCMODE_PWM1;                /* ģʽѡ��PWM1 */
    g_atimx2_oc_chy_handle.Pulse = 0;
    g_atimx2_oc_chy_handle.OCPolarity = TIM_OCPOLARITY_HIGH;        /* ����Ƚϼ���Ϊ�� */
    g_atimx2_oc_chy_handle.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    g_atimx2_oc_chy_handle.OCFastMode = TIM_OCFAST_DISABLE;
    g_atimx2_oc_chy_handle.OCIdleState = TIM_OCIDLESTATE_RESET;
    g_atimx2_oc_chy_handle.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&g_atimx2_handle, &g_atimx2_oc_chy_handle, ATIM_TIMX2_PWM_CH1); /* ����TIMxͨ��y */
    HAL_TIM_PWM_ConfigChannel(&g_atimx2_handle, &g_atimx2_oc_chy_handle, ATIM_TIMX2_PWM_CH2); /* ����TIMxͨ��y */
    HAL_TIM_PWM_ConfigChannel(&g_atimx2_handle, &g_atimx2_oc_chy_handle, ATIM_TIMX2_PWM_CH3); /* ����TIMxͨ��y */

    /* ������ʱ��ͨ��1���PWM */
    HAL_TIM_PWM_Start(&g_atimx2_handle,TIM_CHANNEL_1);

    /* ������ʱ��ͨ��2���PWM */
    HAL_TIM_PWM_Start(&g_atimx2_handle,TIM_CHANNEL_2);

    /* ������ʱ��ͨ��3���PWM */
    HAL_TIM_PWM_Start(&g_atimx2_handle,TIM_CHANNEL_3);

    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 2, 3);
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
   
    HAL_TIM_Base_Start_IT(&g_atimx2_handle);                        /* �����߼���ʱ��8 */
}

/**
 * @brief       ��ʱ���ײ�������ʱ��ʹ�ܣ���������
                �˺����ᱻHAL_TIM_PWM_Init()����
 * @param       htim:��ʱ�����
 * @retval      ��
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == ATIM_TIMX_PWM)
    {
        GPIO_InitTypeDef gpio_init_struct;
        ATIM_TIMX_PWM_CHY_CLK_ENABLE();                             /* ����ͨ��y��GPIOʱ�� */
        ATIM_TIMX_PWM_CH1_GPIO_CLK_ENABLE();                        /* IOʱ��ʹ�� */
        ATIM_TIMX_PWM_CH2_GPIO_CLK_ENABLE();                        /* IOʱ��ʹ�� */
        ATIM_TIMX_PWM_CH3_GPIO_CLK_ENABLE();                        /* IOʱ��ʹ�� */

        M1_LOW_SIDE_U_GPIO_CLK_ENABLE();                            /* IOʱ��ʹ�� */
        M1_LOW_SIDE_V_GPIO_CLK_ENABLE();                            /* IOʱ��ʹ�� */
        M1_LOW_SIDE_W_GPIO_CLK_ENABLE();                            /* IOʱ��ʹ�� */

        /* UVW_LOW��IO��ʼ�� */
        gpio_init_struct.Pin = M1_LOW_SIDE_U_PIN;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_HIGH;
        gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;                /* �������ģʽ */
        HAL_GPIO_Init(M1_LOW_SIDE_U_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = M1_LOW_SIDE_V_PIN;
        HAL_GPIO_Init(M1_LOW_SIDE_V_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = M1_LOW_SIDE_W_PIN;
        HAL_GPIO_Init(M1_LOW_SIDE_W_PORT, &gpio_init_struct);


        /*��ʱ��IO��ʼ��*/
        gpio_init_struct.Pin = ATIM_TIMX_PWM_CH1_GPIO_PIN;          /* ͨ��y��CPIO�� */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                    /* ����������� */
        gpio_init_struct.Pull = GPIO_NOPULL;                        /* ���� */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* ���� */
        gpio_init_struct.Alternate = ATIM_TIMX_PWM_CHY_GPIO_AF;     /* �˿ڸ��� */
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH1_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = ATIM_TIMX_PWM_CH2_GPIO_PIN;          /* ͨ��y��CPIO�� */
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH2_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = ATIM_TIMX_PWM_CH3_GPIO_PIN;          /* ͨ��y��CPIO�� */
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH3_GPIO_PORT, &gpio_init_struct);

    }
    else if (htim->Instance == ATIM_TIMX2_PWM)
    {
        GPIO_InitTypeDef gpio_init_struct;
        ATIM_TIMX2_PWM_CHY_CLK_ENABLE();                            /* ����ͨ��y��CPIOʱ�� */
        ATIM_TIMX2_PWM_CH1_GPIO_CLK_ENABLE();                       /* IOʱ��ʹ�� */
        ATIM_TIMX2_PWM_CH2_GPIO_CLK_ENABLE();                       /* IOʱ��ʹ�� */
        ATIM_TIMX2_PWM_CH3_GPIO_CLK_ENABLE();                       /* IOʱ��ʹ�� */

        M2_LOW_SIDE_U_GPIO_CLK_ENABLE();                            /* IOʱ��ʹ�� */
        M2_LOW_SIDE_V_GPIO_CLK_ENABLE();                            /* IOʱ��ʹ�� */
        M2_LOW_SIDE_W_GPIO_CLK_ENABLE();                            /* IOʱ��ʹ�� */

        /* UVW_LOW��IO��ʼ�� */
        gpio_init_struct.Pin = M2_LOW_SIDE_U_PIN;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_HIGH;
        gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;                /* �������ģʽ */
        HAL_GPIO_Init(M2_LOW_SIDE_U_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = M2_LOW_SIDE_V_PIN;
        HAL_GPIO_Init(M2_LOW_SIDE_V_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = M2_LOW_SIDE_W_PIN;
        HAL_GPIO_Init(M2_LOW_SIDE_W_PORT, &gpio_init_struct);


        /*��ʱ��IO��ʼ��*/
        gpio_init_struct.Pin = ATIM_TIMX2_PWM_CH1_GPIO_PIN;         /* ͨ��y��CPIO�� */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                    /* ����������� */
        gpio_init_struct.Pull = GPIO_NOPULL;                        /* ���� */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* ���� */
        gpio_init_struct.Alternate = ATIM_TIMX2_PWM_CHY_GPIO_AF;    /* �˿ڸ��� */
        HAL_GPIO_Init(ATIM_TIMX2_PWM_CH1_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = ATIM_TIMX2_PWM_CH2_GPIO_PIN;         /* ͨ��y��CPIO�� */
        HAL_GPIO_Init(ATIM_TIMX2_PWM_CH2_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = ATIM_TIMX2_PWM_CH3_GPIO_PIN;         /* ͨ��y��CPIO�� */
        HAL_GPIO_Init(ATIM_TIMX2_PWM_CH3_GPIO_PORT, &gpio_init_struct);

    }
}
/**
 * @brief       ��ʱ���жϷ�����
 * @param       ��
 * @retval      ��
 */
void ATIM_TIMX_PWM_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_atimx_handle);
}

/**
 * @brief       ��ʱ���жϷ�����
 * @param       ��
 * @retval      ��
 */
void ATIM_TIMX2_PWM_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_atimx2_handle);
}

/**
 * @brief       ��ʱ���жϻص�
 * @param       htim:��ʱ�����
 * @retval      ��
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t bldc_dir=0;
    if(htim->Instance == ATIM_TIMX_PWM)                                     /* 55us */
    {
#ifdef H_PWM_L_ON
        if(g_bldc_motor1.run_flag == RUN)
        {
            if(g_bldc_motor1.dir == CW)                                     /* ��ת */
            {
                g_bldc_motor1.step_sta = hallsensor_get_state(MOTOR_1);
            }
            else                                                            /* ��ת */
            {
                g_bldc_motor1.step_sta = 7 - hallsensor_get_state(MOTOR_1);
            }
            if((g_bldc_motor1.step_sta <= 6)&&(g_bldc_motor1.step_sta >= 1))/* �жϻ������ֵ�Ƿ����� */
            {
                pfunclist_m1[g_bldc_motor1.step_sta-1]();
            }
            else                                                            /* ���������󡢽Ӵ��������Ͽ������ */
            {
                stop_motor1();
                g_bldc_motor1.run_flag = STOP;
            }
            if(g_bldc_motor1.step_last != g_bldc_motor1.step_sta)           /* ����״̬�����ı䣬������ */
            {
                g_bldc_motor1.hall_keep_t=0;
                bldc_dir = check_hall_dir(&g_bldc_motor1);                  /* �����ת���򣬸��ݷ��򣬽���λ�ü��� */
                if(bldc_dir == CCW)
                {
                    g_bldc_motor1.pos -=1;
                }
                else if(bldc_dir == CW)
                {
                    g_bldc_motor1.pos +=1;
                }
                g_bldc_motor1.step_last = g_bldc_motor1.step_sta;
            }
            else if(g_bldc_motor1.run_flag == RUN)                          /* �����һ�������ʱ */
            {
                g_bldc_motor1.hall_keep_t++;                                /* ����һ���������ֵ��ʱ�䣩 ��λ1/18k  */

            }
        }

#endif
    }
    if(htim->Instance == ATIM_TIMX2_PWM)                                    /* 55us */
    {
#ifdef H_PWM_L_ON
        if(g_bldc_motor2.run_flag == RUN)
        {
            if(g_bldc_motor2.dir == CW)
            {
                g_bldc_motor2.step_sta = hallsensor_get_state(MOTOR_2);
            }
            else
            {
                g_bldc_motor2.step_sta = 7 - hallsensor_get_state(MOTOR_2);
            }
            if((g_bldc_motor2.step_sta <= 6)&&(g_bldc_motor2.step_sta >= 1))
            {
                pfunclist_m2[g_bldc_motor2.step_sta-1]();
            }
            else                                                            /* ���������󡢽Ӵ��������Ͽ������ */
            {
                stop_motor2();
                g_bldc_motor2.run_flag = STOP;
            }
            if(g_bldc_motor2.step_last != g_bldc_motor2.step_sta)
            {
                g_bldc_motor2.hall_keep_t=0;
                bldc_dir = check_hall_dir(&g_bldc_motor2);
                if(bldc_dir == CCW)
                {
                    g_bldc_motor2.pos -=1;
                }
                else if(bldc_dir == CW)
                {
                    g_bldc_motor2.pos +=1;
                }
                g_bldc_motor2.step_last = g_bldc_motor2.step_sta;
            }
            else if(g_bldc_motor2.run_flag == RUN)                          /* �����һ�������ʱ */
            {
                g_bldc_motor2.hall_keep_t++;                                /* ����һ���������ֵ��ʱ�䣩 ��λ1/18k  */

            }
        }

#endif
    }
}


