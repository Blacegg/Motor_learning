/**
 ****************************************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-16
 * @brief       BLDC电压电流温度采集 实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 电机开发板F407开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
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

    HAL_Init();                              /* 初始化HAL库 */
    sys_stm32_clock_init(336, 8, 2, 7);      /* 设置时钟,168Mhz */
    delay_init(168);                         /* 延时初始化 */
    usart_init(115200);                      /* 串口初始化为115200 */
    led_init();                              /* 初始化LED */
    key_init();                              /* 初始化按键 */
    lcd_init();                              /* 初始化LCD */
    bldc_init(10000-1,0);
    bldc_ctrl(MOTOR_1,CCW,0);                /* 初始无刷电机接口1速度 */

    g_point_color = WHITE;
    g_back_color  = BLACK;
    lcd_show_string(10, 10, 200, 16, 16, "BLDC Motor Test",g_point_color);
    lcd_show_string(10, 30, 200, 16, 16, "KEY0:Start forward", g_point_color);
    lcd_show_string(10, 50, 200, 16, 16, "KEY1:Start backward", g_point_color);
    lcd_show_string(10, 70, 200, 16, 16, "KEY2:Stop", g_point_color);
    adc_nch_dma_init();
    
    printf("按下KEY0 增加比较值即正转加速\r\n");
    printf("按下KEY1 减少比较值即反转加速\r\n");
    printf("按下KEY2 停止电机\r\n");

    while (1)
    {
        t++;
        if(t % 20 == 0)
        {
            sprintf(buf,"PWM_Duty:%.1f%%",(float)((g_bldc_motor1.pwm_duty/MAX_PWM_DUTY)*100));/* 显示控制PWM占空比 */
            lcd_show_string(10,110,200,16,16,buf,g_point_color);
            
            sprintf(buf,"Power:%.3fV ",g_adc_val[0]*ADC2VBUS);
            lcd_show_string(10,190,200,16,16,buf,g_point_color);
            sprintf(buf,"Temp:%.1fC ",get_temp(g_adc_val[1]));
            lcd_show_string(10,210,200,16,16,buf,g_point_color);           
            LED0_TOGGLE();                                      /* LED0(红灯) 翻转 */
            

            current[0] = adc_amp_un[0]* ADC2CURT;               /* 计算出三相电流值，U */
            current[1] = adc_amp_un[1]* ADC2CURT;               /* 计算出三相电流值，V */
            current[2] = adc_amp_un[2]* ADC2CURT;               /* 计算出三相电流值，W */
            
            /*一阶数字滤波 滤波系数0.1 用于显示*/
            FirstOrderRC_LPF(current_lpf[0],current[0],0.1f);   /* U相电流 */
            FirstOrderRC_LPF(current_lpf[1],current[1],0.1f);   /* V相电流 */
            FirstOrderRC_LPF(current_lpf[2],current[2],0.1f);   /* W相电流 */
            FirstOrderRC_LPF(current_lpf[3],adc_amp_bus,0.1f);  /* 母线电流 */
            
            if(g_bldc_motor1.run_flag == STOP)                  /* 停机的电流显示 */
            {
                current_lpf[0] = 0;
                current_lpf[1] = 0;
                current_lpf[2] = 0;
                current_lpf[3] = 0;
            }
            /* LCD显示提示信息 */
            sprintf(buf,"Amp U:%.3fmA ",(float)current_lpf[0]);
            lcd_show_string(10,230,200,16,16,buf,g_point_color);
            sprintf(buf,"Amp V:%.3fmA ",(float)current_lpf[1]);
            lcd_show_string(10,250,200,16,16,buf,g_point_color);
            sprintf(buf,"Amp W:%.3fmA ",(float)current_lpf[2]);
            lcd_show_string(10,270,200,16,16,buf,g_point_color);
            sprintf(buf,"Amp Bus:%.3fmA ",(float)current_lpf[3]);
            lcd_show_string(10,290,200,16,16,buf,g_point_color);
            
            /* 串口打印信息 */
            printf("Valtage:%.1fV \r\n", g_adc_val[0]*ADC2VBUS);
            printf("Temp:%.1fC \r\n", get_temp(g_adc_val[1]));
            printf("U相电流为：%.3fmA\r\n", (current_lpf[0]));
            printf("V相电流为：%.3fmA\r\n", (current_lpf[1]));
            printf("W相电流为：%.3fmA\r\n", (current_lpf[2]));
            printf("母线电流为：%.3fmA\r\n", (current_lpf[3]));
            printf("\r\n");
        }

        key = key_scan(0);
        if(key == KEY0_PRES)                            /* 按下KEY0占空比++ */
        {
            pwm_duty_temp += 500;
            if(pwm_duty_temp >= MAX_PWM_DUTY)
                pwm_duty_temp = MAX_PWM_DUTY;
            if(pwm_duty_temp > 0)
            {
                g_bldc_motor1.pwm_duty = pwm_duty_temp;/* 设置占空比 */
                g_bldc_motor1.dir = CW;                /* 设置方向 */
            }
            else
            {
                g_bldc_motor1.pwm_duty = -pwm_duty_temp;
                g_bldc_motor1.dir = CCW;
            }
            g_bldc_motor1.run_flag = RUN;               /* 运行标志 */
            start_motor1();                             /* 开启运行*/
        }
        else if(key == KEY1_PRES)                       /* 按下KEY1占空比-- */
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
            g_bldc_motor1.run_flag = RUN;               /* 开启运行*/
            start_motor1();                             /* 运行电机*/
        }
        else if(key == KEY2_PRES)                       /* 按下KEY2关闭电机 */
        {
            stop_motor1();                              /* 停机*/
            g_bldc_motor1.run_flag = STOP;              /* 标记停机*/
            pwm_duty_temp = 0;
            g_bldc_motor1.pwm_duty = 0;
        }
        delay_ms(10);
    }
}
