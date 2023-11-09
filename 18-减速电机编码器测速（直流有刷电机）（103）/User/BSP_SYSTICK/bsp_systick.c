#include "bsp_systick.h"

#if ITORNO

uint32_t TimingDelay = 0;

static void Systick_Init_us(void)
{
    //������װ�ص�ֵ=72M/100000=720
    //�����ǰ���ݼĴ���
    //ʹ�ܶ�ʱ��
    if(HAL_SYSTICK_Config(SystemCoreClock/1000000)) //һ���ж�1us
    {
        while(1);
    }
}

void Systick_Delay_us(uint32_t count)
{
    Systick_Init_us();
    
    TimingDelay = count;
    while(TimingDelay);
    
    //�ر�SysTick��ʱ��
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void Systick_Init_ms(void)
{
    //������װ�ص�ֵ=72M/100000=720
    //�����ǰ���ݼĴ���
    //ʹ�ܶ�ʱ��
    if(HAL_SYSTICK_Config(SystemCoreClock/1000)) //һ���ж�1ms
    {
        while(1);
    }
}

void Systick_Delay_ms(uint32_t count)
{
    Systick_Init_ms();

    TimingDelay = count;
    while(TimingDelay);
    
    //�ر�SysTick��ʱ��
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}
#else

void Systick_Delay_us(uint32_t count)
{
    //��ʼ������SysTick��ʱ��
    HAL_SYSTICK_Config(SystemCoreClock/1000000);
    
    uint32_t i = 0; 
    for(i = 0; i < count; i++)
    {
        while(!(SysTick->CTRL)&(1<<16));
    }
    
    //�ر�SysTick��ʱ��
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void Systick_Delay_ms(uint32_t count)
{
    //��ʼ������SysTick��ʱ��
    HAL_SYSTICK_Config(SystemCoreClock/1000);
    
    uint32_t i = 0; 
    for(i = 0; i < count; i++)
    {
        while(!(SysTick->CTRL)&(1<<16));
    }
    
    //�ر�SysTick��ʱ��
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

#endif
