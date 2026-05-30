#include "motor.h"
/**************************************************************************
Function: Initialize Motor Interface
Input   : none
Output  : none
函数功能：初始化电机接口
入口参数：无
返回  值：无
**************************************************************************/
void BalanceCar_Motor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能PC端口时钟 Enable PC port clock
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;	//端口配置 port configuration
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      //推挽输出 Push pull output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOC, &GPIO_InitStructure);					      //根据设定参数初始化GPIOC Initialize GPOC according to the set parameters
}



/**************************************************************************
Function: Initialize PWM to drive motor
Input   : arr：Auto reload value；psc：Prescaler coefficient
Output  : none
函数功能：初始化PWM，用于驱动电机 
入口参数：arr：自动重装值；psc：预分频系数
返回  值：无
**************************************************************************/
void BalanceCar_PWM_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	TIM_DeInit(TIM8);
	TIM_TimeBaseStructure.TIM_Period = arr - 1; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 Set the value of the automatic reload register cycle for the next update event loading activity
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  Set the prescaler value used as the divisor of the TIMx clock frequency. No division
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim Set clock division: TDTS=Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式 TIM Up Counting Mode
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位  Initialize the time base unit of TIMx based on the parameters specified in TIM_TimeBaseInitStruct

 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1  Select Timer Mode: TIM Pulse Width Modulation Mode 1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能  Compare output enablement
	TIM_OCInitStructure.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值  Set the pulse value to be loaded into the capture comparison register
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高  Output polarity: TIM output has a relatively high polarity
	
	
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx  Initialize peripheral TIMx based on the parameters specified in TIM_SICInitStruct
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx  Initialize peripheral TIMx based on the parameters specified in TIM_SICInitStruct
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx  Initialize peripheral TIMx based on the parameters specified in TIM_SICInitStruct
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx  Initialize peripheral TIMx based on the parameters specified in TIM_SICInitStruct


	TIM_CtrlPWMOutputs(TIM8,ENABLE);	//MOE 主输出使能  Main output enable

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1预装载使能	  CH1 pre load enable
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH2预装载使能    CH2 pre load enable
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH3预装载使能	  CH3 pre load enable
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH4预装载使能    CH4 pre load enable
	
	TIM_ARRPreloadConfig(TIM8, ENABLE); //使能TIMx在ARR上的预装载寄存器   Enable TIMx pre loaded registers on ARR
	
	
	/* TIM8 enable counter */
	TIM_Cmd(TIM8, ENABLE);                   //使能定时器8	Enable Timer 8

}
