#include "bsp_systick.h"

#if ITORNO

uint32_t TimingDelay = 0;

static void Systick_Init_us(void)
{
    //设置重装载的值=72M/100000=720
    //清除当前数据寄存器
    //使能定时器
    if(HAL_SYSTICK_Config(SystemCoreClock/1000000)) //一次中断1us
    {
        while(1);
    }
}

void Systick_Delay_us(uint32_t count)
{
    Systick_Init_us();
    
    TimingDelay = count;
    while(TimingDelay);
    
    //关闭SysTick定时器
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void Systick_Init_ms(void)
{
    //设置重装载的值=72M/100000=720
    //清除当前数据寄存器
    //使能定时器
    if(HAL_SYSTICK_Config(SystemCoreClock/1000)) //一次中断1ms
    {
        while(1);
    }
}

void Systick_Delay_ms(uint32_t count)
{
    Systick_Init_ms();

    TimingDelay = count;
    while(TimingDelay);
    
    //关闭SysTick定时器
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}
#else

void Systick_Delay_us(uint32_t count)
{
    //初始化，开SysTick定时器
    HAL_SYSTICK_Config(SystemCoreClock/1000000);
    
    uint32_t i = 0; 
    for(i = 0; i < count; i++)
    {
        while(!(SysTick->CTRL)&(1<<16));
    }
    
    //关闭SysTick定时器
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void Systick_Delay_ms(uint32_t count)
{
    //初始化，开SysTick定时器
    HAL_SYSTICK_Config(SystemCoreClock/1000);
    
    uint32_t i = 0; 
    for(i = 0; i < count; i++)
    {
        while(!(SysTick->CTRL)&(1<<16));
    }
    
    //关闭SysTick定时器
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

#endif
