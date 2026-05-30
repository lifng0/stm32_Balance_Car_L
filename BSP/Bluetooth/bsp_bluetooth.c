#include "bsp_bluetooth.h"

//蓝牙串口初始化 只能是9600波特率
// Initialize the Bluetooth serial port, the baud rate can only be 9600
void bluetooth_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD , ENABLE);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);    

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	  
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART5, &USART_InitStructure); 
//	USART_ITConfig(UART5, USART_IT_TXE, DISABLE);  

	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);   
	USART_Cmd(UART5, ENABLE);
	
	
	Init_PID();
	
	
}


//发送一个字符  Send a character
void UART5_Send_U8(uint8_t ch)
{
	while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(UART5, ch);
}

//发送一个字符串  Send a string
/**
 * @Brief: UART5发送数据 UART5 sends data
 * @Note: 
 * @Parm: BufferPtr:待发送的数据(Data to be sent)  Length:数据长度(Data length)
 * @Retval: 
 */
void UART5_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
	while (Length--)
	{
		UART5_Send_U8(*BufferPtr);
		BufferPtr++;
	}
}

/*串口5发送函数 Serial port 5 sending function*/
void USART5_Send_Byte(unsigned char byte)   //串口发送一个字节 The serial port sends a byte
{
//	while( USART_GetFlagStatus(UART5,USART_FLAG_TC)!= SET);  
	while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
	USART_SendData(UART5, byte);        //通过库函数  发送数据  Send data through library functions
        
}
void UART5_Send_Char(char *s)
{
	char *p;
	p=s;
	while(*p != '\0')
	{
		USART5_Send_Byte(*p);
		p++;
	}	
}

//串口中断服务函数  Serial port interrupt service function
void UART5_IRQHandler(void)
{
	uint8_t Rx5_Temp;
	if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{
		Rx5_Temp = USART_ReceiveData(UART5);
		deal_bluetooth(Rx5_Temp);
	}
}


