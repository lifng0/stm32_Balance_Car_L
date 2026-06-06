#include "bsp_usart2.h"


//USART2
void USART2_init(u32 baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	  
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure); 
 
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);        
	//USART_ClearFlag(USART2,USART_FLAG_TC);
	USART_Cmd(USART2, ENABLE);
	

}


//����һ���ַ� Send a character
void USART2_Send_U8(uint8_t ch)
{
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(USART2, ch);
}

//����һ���ַ���  Send a string
/**
 * @Brief: UsART2�������� UsART2 sends data
 * @Note:  
 * @Parm: BufferPtr:�����͵�����  Length:���ݳ���   BufferPtr: data to be sent Length: data length
 * @Retval: 
 */
void USART2_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
	while (Length--)
	{
		USART2_Send_U8(*BufferPtr);
		BufferPtr++;
	}
}

//�����жϷ����� Serial port interrupt service function
void USART2_IRQHandler(void)
{
	uint8_t Rx2_Temp;
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		Rx2_Temp = USART_ReceiveData(USART2);
		PI_Comm_OnByteReceived(Rx2_Temp);
	}
}

