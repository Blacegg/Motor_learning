#include "bsp_usart.h"

UART_HandleTypeDef UartHandle;

void USART_Config(void)
{
    // ʹ��RX �� TX ���� GPIO ʱ�Ӻ� USART ʱ��
    DEBUG_USART_CLK_ENABLE();
    DEBUG_USART_Tx_CLK_ENABLE();
    DEBUG_USART_Rx_CLK_ENABLE();
    
    // ��ʼ�� GPIO������ GPIO ���õ� USART ��
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pin = DEBUG_USART_Tx_Pin;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    //UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;//��Ӳ������
    HAL_GPIO_Init(DEBUG_USART_Tx_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.Mode = GPIO_MODE_AF_INPUT;
    GPIO_InitStructure.Pin = DEBUG_USART_Rx_Pin;
    HAL_GPIO_Init(DEBUG_USART_Rx_PORT, &GPIO_InitStructure);
    
    // ���� USART ����
    UartHandle.Instance = DEBUG_USART; //USART1 ���
    UartHandle.Init.BaudRate = DEBUG_USART_BAUDRATE;
    UartHandle.Init.Mode = USART_MODE_TX_RX;
    UartHandle.Init.Parity = USART_PARITY_NONE;
    UartHandle.Init.StopBits = USART_STOPBITS_1;
    UartHandle.Init.WordLength = USART_WORDLENGTH_8B;
    HAL_UART_Init(&UartHandle);
    
    // �����жϿ�������ʹ�� USART �����ж�
    HAL_NVIC_SetPriority(DEBUG_USART_IRQ, 0, 1);
    HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ);
    __HAL_UART_ENABLE_IT(&UartHandle,UART_IT_RXNE);
    
    // ʹ�ܴ���
    //__HAL_USART_ENABLE(&UartHandle);
}

// �����ַ���
void Usart_SendString(uint8_t *str)
{
    uint8_t i = 0;
    while(*(str+i)!='\0')
    {
        HAL_UART_Transmit(&UartHandle, (uint8_t *)(str + i), 1, 1000);
        i++;
    }
}

//�ض���c�⺯��printf������DEBUG_USART���ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
	/* ����һ���ֽ����ݵ�����DEBUG_USART */
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 1000);	
	
	return (ch);
}

//�ض���c�⺯��scanf������DEBUG_USART����д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{		
	int ch;
	HAL_UART_Receive(&UartHandle, (uint8_t *)&ch, 1, 1000);	
	return (ch);
}
