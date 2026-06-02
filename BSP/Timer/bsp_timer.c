#include "bsp_timer.h"

static float battery_All;
static uint8_t battery_count=0,battery_flag=0;

static u16 stop_time = 0;//延迟时间  delay time

u16 led_flag = 0; //1:进入闪烁状态 0:等待闪烁 //1: Entering flashing state 0: waiting for flashing
u16 led_twinkle_count = 0;// 闪烁计数  //Flashing Count

u16 led_count = 0; //开始计数  //Start counting

u8 lower_power_flag = 0; //低电压标志  0:电压正常 1：低压  //Low Voltage Flag 0: Normal Voltage 1: Low Voltage


//定时器6做延迟 10ms的延迟 此方法比delay准确
//Timer 6 has a delay of 10ms. This method is more accurate than delay
void delay_time(u16 time)
{
	stop_time = time;
	while(stop_time);//死等 Wait
}

//延迟1s  Unit second
void my_delay(u16 s)//s
{
	for(int i = 0;i<s;i++)
	{
		delay_time(100);
	}
}


/**************************************************************************
Function function: TIM6 initialization, timed for 10 milliseconds
Entrance parameters: None
Return value: None
函数功能：TIM6初始化，定时10毫秒
入口参数：无
返回  值：无
**************************************************************************/
void TIM6_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //使能定时器的时钟  Enable the clock of the timer
	TIM_TimeBaseStructure.TIM_Prescaler = 7199;			 // 预分频器  Prescaler
	TIM_TimeBaseStructure.TIM_Period = 99;				 //设定计数器自动重装值  Set the automatic reset value of the counter
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);                //清除TIM的更新标志位 Clear the update flag of TIM
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

	//中断优先级NVIC设置  Interrupt priority NVIC setting
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;			  //TIM6中断 TIM6 interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4; //先占优先级4级 Preemption priority level 4
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  //从优先级2级 From priority level 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQ通道被使能 IRQ channel is enabled
	NVIC_Init(&NVIC_InitStructure);							  //初始化NVIC寄存器 Initialize NVIC registers

	TIM_Cmd(TIM6, ENABLE);
}


u8 bulettohflag = 0;

// TIM6中断 //TIM6 Interrupt service
void TIM6_IRQHandler(void)
{ 
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) //检查TIM更新中断发生与否  Check whether TIM update interruption occurs
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);    //清除TIMx更新中断标志  Clear TIMx update interrupt flag
		led_count++;  //led服务显示标志 LED service display logo
		battery_flag ++;		//电量显示标志	 Electricity display sign

		
		if(stop_time>0)
		{
			stop_time --;
		}

		PI_Comm_10ms_Task();
		
		if(mode == Normal || mode == Weight_M)
		{ 
				bulettohflag = 1;

//				//放在中断这里会影响平衡  Putting it here will affect the balance.
//				SendAutoUp();//蓝牙自动上报数据 Bluetooth automatically reports data 
		}
		
////////电压检测流程		 Voltage detection process
		if(battery_flag > 2)//20ms
		{		
			battery_flag = 0;
			battery_All += Get_Battery_Volotage();//获取电源电量 Obtain the power level of the power supply
			battery_count++;
			if(battery_count == 50)//1000ms
			{
				battery = battery_All/50; //平均值 average value
				battery_All = 0;  
				battery_count = 0;
				power_decect();//电压处理  Voltage processing
			}
			
		}
///////////
		
		cotrol_led();//灯服务  led service
		
				
		
	}
}


void power_decect(void)
{
	static u8 normal_power_flag = 1; //电压恢复标志 0：没恢复 1:恢复 //Voltage recovery flag 0: not restored 1: restored
	if(battery < 9.6) //小于9.6V报警 //Alarm below 9.6V
	{
		lower_power_flag = 1;
		normal_power_flag = 0;
	}
	else
	{
		if(normal_power_flag == 0)
		{
			lower_power_flag = 0;
			normal_power_flag = 1;
			BEEP_BEEP = 0;
		}
		
	}
}

void cotrol_led(void)
{
	//灯的效果和蜂鸣器的效果 低压报警 //The effect of the lamp and buzzer is low voltage alarm
		if(!led_flag)
		{
			if(led_count>300)//3S
			{
				led_count = 0;
				led_flag = 1;
			}
		}
		else
		{
			if(led_count>20)//200ms
			{
				led_count = 0;
				
				if(lower_power_flag == 0)
				{
					LED = !LED;//状态反转 //State reversal
				}
				else
				{
					BEEP_BEEP = !BEEP_BEEP;
					LED = 1;//低压蓝灯常亮 //Low voltage blue light is always on
				}
				
				led_twinkle_count++;
				if(led_twinkle_count == 6)
				{
					if(lower_power_flag == 0)
					{
						LED = 0;
					}
					else
					{
						BEEP_BEEP = 0;
					}
					
					led_twinkle_count = 0;
					led_flag = 0;
				}
				
			}
		}

}

