/*
* @par Copyright (C): 2018-2028, Shenzhen Yahboom Tech
* @file         // intsever.c
* @author       // lly
* @version      // V1.0
* @date         // 20240628
* @brief        // 小车控制平衡的主要中断服务  Main interruption service for trolley control balance
*/


#include "intsever.h"

void MPU6050_EXTI_Init(void)
{  
		GPIO_InitTypeDef GPIO_InitStructure;
		EXTI_InitTypeDef EXTI_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);   //外部中断，需要使能AFIO时钟 //External interrupt, AFIO clock needs to be enabled
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能GPIO端口时钟  //Enable GPIO port clock
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	          //端口配置   //Port configuration
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入   //Pull up input
		GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIO  //Initialize GPIO according to the set parameters
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource12);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line12;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发 //Falling edge trigger
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器  //Initialize the external EXTI register based on the parameters specified in EXTI_InitStruct
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//使能按键所在的外部中断通道  //Enable the external interrupt channel where the button is located
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2，    //Seize priority 2,
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级3 			 //Sub priority 3
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道 //Enable external interrupt channel
  	NVIC_Init(&NVIC_InitStructure); 
}






