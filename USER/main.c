/**
* @par Copyright (C): 2018-2028, Shenzhen Yahboom Tech
* @file         // main.c
* @author       // lly
* @version      // V1.0
* @date         // 240628
* @brief        // 程序入口 Program entry
* @details      
* @par History  // 修改历史记录列表，每条修改记录应包括修改日期、修改者及
*               // 修改内容简述  Modification history list, each modification record should include the modification date, modifier and a brief description of the modification content
*/ 

#include "AllHeader.h"
#include "intsever.h"
//注意:操作蜂鸣器的时候，要判断是否处于正常电压
//Attention: When operating the buzzer, check if it is at normal voltage

uint8_t GET_Angle_Way=2;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波  //Algorithm for obtaining angles, 1: Quaternion 2: Kalman 3: Complementary filtering
float Angle_Balance,Gyro_Balance,Gyro_Turn;     		//平衡倾角 平衡陀螺仪 转向陀螺仪 //Balance tilt angle balance gyroscope steering gyroscope
int Motor_Left,Motor_Right;                 	  		//电机PWM变量 //Motor PWM variable
int Temperature;                                		//温度变量 		//Temperature variable
float Acceleration_Z;                           		//Z轴加速度计  //Z-axis accelerometer
float Mid_Angle  = -10.3;                          						//机械中值  //Mechanical median
float Move_X,Move_Z; //Move_X:前进速度  Move_Z：转向速度  //Move_X: Forward speed Move_Z: Steering speed
u8 Stop_Flag = 1; //0:开始 1:停止  //0: Start 1: Stop


char showbuf[20]={'\0'};

extern u8 newLineReceived;//蓝牙接收 //Bluetooth reception
extern u8 bulettohflag;

int main(void)
{	
		
	bsp_init();//基本外设初始化 //Basic peripheral initialization
	
	Mode_select(); //按下按键结束模式选择 //Press the button to end mode selection
	
	bsp_mode_init();//根据模式初始化扩展外设 //Initialize and expand peripherals based on the pattern
	
	
	MPU6050_EXTI_Init();		//此中断服务函数放到最后  //This interrupt service function is placed last
	
	
	OLED_Draw_Line("put down key start!", 2, false, true); 
	
	while(!Key1_State(1) && Stop_Flag ==1 );
	Stop_Flag = 0; //开始控制  //Start controlling

	
	OLED_Draw_Line("start control!        ", 2, false, true); 
	


	while(1)
	{
		if(mode == Normal || mode == Weight_M)//正常模式、负重模式  //Normal mode, load mode
		{
			if (newLineReceived) //蓝牙遥控服务  //Bluetooth remote control service
			{
				ProtocolCpyData();
				Protocol();
			}
			if(bulettohflag == 1) 
			{
				bulettohflag = 0;
				SendAutoUp();//蓝牙自动上报数据 Bluetooth automatically reports data 
			}

			sprintf(showbuf,"bat =%2.2f V   ",battery);
			OLED_Draw_Line(showbuf, 3, false, true); 
		}

		else if(mode == K210_QR) //识别二维码模式  //Identify QR code patterns
		{
			Change_state_QR();//识别  //Identify
		}
		else if(mode == K210_SelfLearn) //自主学习模式 //Self directed learning mode
		{
			Change_state_self();
		}
		else if(mode == K210_mnist) //识别数字模式  //Identify numerical patterns
		{
			Change_state_minst();
		}
	}
}


