#include "bsp_usart.h"

UART_HandleTypeDef UartHandle;

void USART_Config(void)
{
    // 使能RX 和 TX 引脚 GPIO 时钟和 USART 时钟
    DEBUG_USART_CLK_ENABLE();
    DEBUG_USART_Tx_CLK_ENABLE();
    DEBUG_USART_Rx_CLK_ENABLE();
    
    // 初始化 GPIO，并将 GPIO 复用到 USART 上
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pin = DEBUG_USART_Tx_Pin;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    //UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;//无硬件流控
    HAL_GPIO_Init(DEBUG_USART_Tx_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.Mode = GPIO_MODE_AF_INPUT;
    GPIO_InitStructure.Pin = DEBUG_USART_Rx_Pin;
    HAL_GPIO_Init(DEBUG_USART_Rx_PORT, &GPIO_InitStructure);
    
    // 配置 USART 参数
    UartHandle.Instance = DEBUG_USART; //USART1 句柄
    UartHandle.Init.BaudRate = DEBUG_USART_BAUDRATE;
    UartHandle.Init.Mode = USART_MODE_TX_RX;
    UartHandle.Init.Parity = USART_PARITY_NONE;
    UartHandle.Init.StopBits = USART_STOPBITS_1;
    UartHandle.Init.WordLength = USART_WORDLENGTH_8B;
    HAL_UART_Init(&UartHandle);
    
    // 配置中断控制器并使能 USART 接收中断
    HAL_NVIC_SetPriority(DEBUG_USART_IRQ, 0, 1);
    HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ);
    __HAL_UART_ENABLE_IT(&UartHandle,UART_IT_RXNE);
    
    // 使能串口
    //__HAL_USART_ENABLE(&UartHandle);
}

// 发送字符串
void Usart_SendString(uint8_t *str)
{
    uint8_t i = 0;
    while(*(str+i)!='\0')
    {
        HAL_UART_Transmit(&UartHandle, (uint8_t *)(str + i), 1, 1000);
        i++;
    }
}

//重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口DEBUG_USART */
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 1000);	
	
	return (ch);
}

//重定向c库函数scanf到串口DEBUG_USART，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{		
	int ch;
	HAL_UART_Receive(&UartHandle, (uint8_t *)&ch, 1, 1000);	
	return (ch);
}
