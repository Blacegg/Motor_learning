/**
 ****************************************************************************************************
 * @file        main.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-16
 * @brief       BLDC��ѹ�����¶Ȳɼ� ʵ��
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� ���������F407������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/TIMER/bldc_tim.h"
#include "./BSP/KEY/key.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/BLDC/bldc.h"
#include "./BSP/ADC/adc.h"

extern int16_t adc_amp_un[3];
extern float  adc_amp_bus;

int main(void)
{
    uint8_t key,t;
    char buf[32];
    int16_t pwm_duty_temp=0;
    float current[3]= {0.0f};
    float current_lpf[4]= {0.0f};

    HAL_Init();                              /* ��ʼ��HAL�� */
    sys_stm32_clock_init(336, 8, 2, 7);      /* ����ʱ��,168Mhz */
    delay_init(168);                         /* ��ʱ��ʼ�� */
    usart_init(115200);                      /* ���ڳ�ʼ��Ϊ115200 */
    led_init();                              /* ��ʼ��LED */
    key_init();                              /* ��ʼ������ */
    lcd_init();                              /* ��ʼ��LCD */
    bldc_init(10000-1,0);
    bldc_ctrl(MOTOR_1,CCW,0);                /* ��ʼ��ˢ����ӿ�1�ٶ� */

    g_point_color = WHITE;
    g_back_color  = BLACK;
    lcd_show_string(10, 10, 200, 16, 16, "BLDC Motor Test",g_point_color);
    lcd_show_string(10, 30, 200, 16, 16, "KEY0:Start forward", g_point_color);
    lcd_show_string(10, 50, 200, 16, 16, "KEY1:Start backward", g_point_color);
    lcd_show_string(10, 70, 200, 16, 16, "KEY2:Stop", g_point_color);
    adc_nch_dma_init();
    
    printf("����KEY0 ���ӱȽ�ֵ����ת����\r\n");
    printf("����KEY1 ���ٱȽ�ֵ����ת����\r\n");
    printf("����KEY2 ֹͣ���\r\n");

    while (1)
    {
        t++;
        if(t % 20 == 0)
        {
            sprintf(buf,"PWM_Duty:%.1f%%",(float)((g_bldc_motor1.pwm_duty/MAX_PWM_DUTY)*100));/* ��ʾ����PWMռ�ձ� */
            lcd_show_string(10,110,200,16,16,buf,g_point_color);
            
            sprintf(buf,"Power:%.3fV ",g_adc_val[0]*ADC2VBUS);
            lcd_show_string(10,190,200,16,16,buf,g_point_color);
            sprintf(buf,"Temp:%.1fC ",get_temp(g_adc_val[1]));
            lcd_show_string(10,210,200,16,16,buf,g_point_color);           
            LED0_TOGGLE();                                      /* LED0(���) ��ת */
            

            current[0] = adc_amp_un[0]* ADC2CURT;               /* ������������ֵ��U */
            current[1] = adc_amp_un[1]* ADC2CURT;               /* ������������ֵ��V */
            current[2] = adc_amp_un[2]* ADC2CURT;               /* ������������ֵ��W */
            
            /*һ�������˲� �˲�ϵ��0.1 ������ʾ*/
            FirstOrderRC_LPF(current_lpf[0],current[0],0.1f);   /* U����� */
            FirstOrderRC_LPF(current_lpf[1],current[1],0.1f);   /* V����� */
            FirstOrderRC_LPF(current_lpf[2],current[2],0.1f);   /* W����� */
            FirstOrderRC_LPF(current_lpf[3],adc_amp_bus,0.1f);  /* ĸ�ߵ��� */
            
            if(g_bldc_motor1.run_flag == STOP)                  /* ͣ���ĵ�����ʾ */
            {
                current_lpf[0] = 0;
                current_lpf[1] = 0;
                current_lpf[2] = 0;
                current_lpf[3] = 0;
            }
            /* LCD��ʾ��ʾ��Ϣ */
            sprintf(buf,"Amp U:%.3fmA ",(float)current_lpf[0]);
            lcd_show_string(10,230,200,16,16,buf,g_point_color);
            sprintf(buf,"Amp V:%.3fmA ",(float)current_lpf[1]);
            lcd_show_string(10,250,200,16,16,buf,g_point_color);
            sprintf(buf,"Amp W:%.3fmA ",(float)current_lpf[2]);
            lcd_show_string(10,270,200,16,16,buf,g_point_color);
            sprintf(buf,"Amp Bus:%.3fmA ",(float)current_lpf[3]);
            lcd_show_string(10,290,200,16,16,buf,g_point_color);
            
            /* ���ڴ�ӡ��Ϣ */
            printf("Valtage:%.1fV \r\n", g_adc_val[0]*ADC2VBUS);
            printf("Temp:%.1fC \r\n", get_temp(g_adc_val[1]));
            printf("U�����Ϊ��%.3fmA\r\n", (current_lpf[0]));
            printf("V�����Ϊ��%.3fmA\r\n", (current_lpf[1]));
            printf("W�����Ϊ��%.3fmA\r\n", (current_lpf[2]));
            printf("ĸ�ߵ���Ϊ��%.3fmA\r\n", (current_lpf[3]));
            printf("\r\n");
        }

        key = key_scan(0);
        if(key == KEY0_PRES)                            /* ����KEY0ռ�ձ�++ */
        {
            pwm_duty_temp += 500;
            if(pwm_duty_temp >= MAX_PWM_DUTY)
                pwm_duty_temp = MAX_PWM_DUTY;
            if(pwm_duty_temp > 0)
            {
                g_bldc_motor1.pwm_duty = pwm_duty_temp;/* ����ռ�ձ� */
                g_bldc_motor1.dir = CW;                /* ���÷��� */
            }
            else
            {
                g_bldc_motor1.pwm_duty = -pwm_duty_temp;
                g_bldc_motor1.dir = CCW;
            }
            g_bldc_motor1.run_flag = RUN;               /* ���б�־ */
            start_motor1();                             /* ��������*/
        }
        else if(key == KEY1_PRES)                       /* ����KEY1ռ�ձ�-- */
        {
            pwm_duty_temp -= 500;
            if(pwm_duty_temp <= -MAX_PWM_DUTY)
                pwm_duty_temp = -MAX_PWM_DUTY;
            if(pwm_duty_temp < 0)
            {
                g_bldc_motor1.pwm_duty = -pwm_duty_temp;
                g_bldc_motor1.dir = CCW;
            }
            else
            {
                g_bldc_motor1.pwm_duty = pwm_duty_temp;
                g_bldc_motor1.dir = CW;
            }
            g_bldc_motor1.run_flag = RUN;               /* ��������*/
            start_motor1();                             /* ���е��*/
        }
        else if(key == KEY2_PRES)                       /* ����KEY2�رյ�� */
        {
            stop_motor1();                              /* ͣ��*/
            g_bldc_motor1.run_flag = STOP;              /* ���ͣ��*/
            pwm_duty_temp = 0;
            g_bldc_motor1.pwm_duty = 0;
        }
        delay_ms(10);
    }
}
