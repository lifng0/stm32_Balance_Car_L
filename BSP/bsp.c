#include "bsp.h"
#include "intsever.h"

#define PI_COMM_BAUDRATE 921600U

void bsp_init(void)
{
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//库的设置中断分组  Interrupt grouping in library settings
	DIY_NVIC_PriorityGroupConfig(2);	  //设置中断分组   //Set interrupt grouping
	delay_init();	    	            //延时函数初始化	 //Delay function initialization
	JTAG_Set(JTAG_SWD_DISABLE);     //关闭JTAG接口    //Close JTAG interface
	JTAG_Set(SWD_ENABLE);           //打开SWD接口 可以利用主板的SWD接口调试 //Opening the SWD interface allows for debugging using the motherboard's SWD interface
	
	//led/beep
	init_led_gpio();								//板载LED  		//Onboard LED
	init_beep();										//板载蜂鸣器 //Onboard buzzer
	Key1_GPIO_Init();								//板载按键	 //Onboard buttons
	
	
	BalanceCar_Motor_Init();     	//电机GPIO初始化  //Motor GPIO initialization
	BalanceCar_PWM_Init(2880,0); 	//初始化PWM 25Khz 与电机硬件接口，用于驱动电机   Initialize PWM 25Khz and motor hardware interface for driving the motor
	Encoder_Init_TIM3();            //初始化编码器3  Initialize encoder 3
	Encoder_Init_TIM4();            //初始化编码器4  Initialize encoder 4
	
	uart_init(115200);	            //串口1初始化  Serial port 1 initialization
	PI_Comm_Init(PI_COMM_BAUDRATE);	 //树莓派通信串口初始化  Raspberry Pi communication UART

	delay_ms(300);
	
	IIC_MPU6050_Init();							//陀螺仪i2c初始化   Gyroscope I2C initialization
	MPU6050_initialize();						//陀螺仪量程初始化  Gyroscope range initialization
	DMP_Init();                     //DMP初始化    DMP initialization
	
	OLED_I2C_Init();							 //oled初始化  OLED initialization
	
	Battery_init();									//电池电量检测初始化 Initialization of battery level detection
	TIM6_Init();									//LED闪烁、电压检测、树莓派状态服务函数
}

//根据模式选择进行模块的初始化  Initialize modules based on mode selection
void bsp_mode_init(void)
{
	if(mode == Normal || mode == Weight_M)
	{
		bluetooth_init();								//蓝牙初始化   Bluetooth initialization
	}
}


void JTAG_Set(u8 mode)
{
	u32 temp;
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //开启辅助时钟	  Activate auxiliary clock  
	AFIO->MAPR&=0XF8FFFFFF; //清除MAPR的[26:24] Clear MAPR [26:24]
	AFIO->MAPR|=temp;       //设置jtag模式 Set jtag mode
} 


/**************************************************************************
Function: Set NVIC group
Input   : NVIC_Group
Output  : none
函数功能：设置中断分组
入口参数：NVIC_Group:NVIC分组 0~4 总共5组 	
返回  值：无
**************************************************************************/ 
void DIY_NVIC_PriorityGroupConfig(u8 NVIC_Group)	 
{ 
	u32 temp,temp1;	  
	temp1=(~NVIC_Group)&0x07;//取后三位 Take the last three
	temp1<<=8;
	temp=SCB->AIRCR;  //读取先前的设置  Read previous settings
	temp&=0X0000F8FF; //清空先前分组   Clear previous groups
	temp|=0X05FA0000; //写入钥匙  Write the key
	temp|=temp1;	   
	SCB->AIRCR=temp;  //设置分组	  Set grouping   	  				   
}

