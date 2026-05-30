#include "bsp_LED.h"


void init_led_gpio(void)
{
		
		GPIO_InitTypeDef GPIO_InitStructure;
	
//		PWR_BackupAccessCmd(ENABLE); //允许修改RTC 和后备寄存器  //Allow modification of RTC and backup registers
//		RCC_LSEConfig(RCC_LSE_OFF);  //关闭外部低速外部时钟信号功能 后，PC13 PC14 PC15 才可以当普通IO用。 //After turning off the external low-speed external clock signal function, PC13 PC14 PC15 can be used as ordinary IO.
//		BKP_TamperPinCmd(DISABLE);   //关闭入侵检测功能，也就是 PC13，也可以当普通IO 使用  //Disable the intrusion detection function, that is, PC13, which can also be used as ordinary IO
//		PWR_BackupAccessCmd(DISABLE);//禁止修改后备寄存器 //Prohibit modification of backup registers
	
		//初始化LED的引脚 Initialize the LED pins
		RCC_APB2PeriphClockCmd(LED_RCC, ENABLE); 									   
  	GPIO_InitStructure.GPIO_Pin = LED_PIN;	
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
  	GPIO_Init(LED_PORT, &GPIO_InitStructure);
	
		LED = 0;

}


