#include "bsp_beep.h"

u32 beep_time = 0;

void init_beep(void)
{
	//初始化蜂鸣器的引脚 Initialize the buzzer pin
  GPIO_InitTypeDef GPIO_InitStructure;
	/*开启外设时钟  Enable peripheral clock */
	RCC_APB2PeriphClockCmd(BEEP_RCC, ENABLE); 
	/*选择要控制的引脚  Select the pin to control */															   
	GPIO_InitStructure.GPIO_Pin = BEEP_PIN;	
	/*设置引脚模式为通用推挽输出 Set the pin mode to general push-pull output */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
	/*设置引脚速率为50MHz Set the pin rate to 50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
	/*调用库函数，初始化BEEP_PORT Call library function to initialize BEEP_PORT */
	GPIO_Init(BEEP_PORT, &GPIO_InitStructure);
	
	BEEP_BEEP = 0;

}

//蜂鸣器打开时长 Buzzer on time
//beep_time ：1ms为单位  1ms as unit
void open_beep(u32 beep_time) //10ms减一次  Subtract once every 10ms
{
	beep_time = beep_time/10;
}

