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
u8 Balance_Run_Enabled = 0; //0:未进入运行态 1:允许运行平衡环  //0: not armed 1: balance loop may run


char showbuf[20]={'\0'};

extern u8 newLineReceived;//蓝牙接收 //Bluetooth reception
extern u8 bulettohflag;

typedef enum
{
	SYS_WAIT_PI_READY = 0,
	SYS_MODE_SELECT,
	SYS_WAIT_START,
	SYS_RUNNING,
	SYS_FAULT_RECOVERY,
	SYS_SHUTDOWN_WAIT
} System_Run_State;

static void Show_Pi_Init_State(void)
{
	uint8_t host_state = PI_Comm_GetHostStateFlags();
	OLED_Draw_Line("Pi system init...", 1, true, false);
	OLED_Draw_Line((host_state & 0x01) ? "PI: READY         " : "PI: WAIT          ", 2, false, false);
	OLED_Draw_Line((host_state & 0x02) ? "LIDAR: READY      " : "LIDAR: WAIT       ", 3, false, false);
	OLED_Draw_Line((host_state & 0x04) ? "SYSTEM: READY     " : "SYSTEM: BOOTING   ", 4, false, true);
}

static void Show_Wait_Start_State(void)
{
	OLED_Draw_Line("press key to start", 2, false, false);
	OLED_Draw_Line("hold key to shutdn", 3, false, true);
}

static void Show_Shutdown_State(void)
{
	OLED_Draw_Line("shutdown request  ", 2, false, false);
	OLED_Draw_Line("wait pi poweroff  ", 3, false, false);
	OLED_Draw_Line("safe poweroff soon", 4, false, true);
}

static void Restore_Normal_Mode_Safe(void)
{
	Balance_Run_Enabled = 0;
	Stop_Flag = 1;
	Move_X = 0;
	Move_Z = 0;
	mode = Normal;
	Set_Mid_Angle();
	Set_angle();
	Set_control_speed();
	Set_PID();
	bsp_mode_init();
}

static void Show_Fault_Recovery_State(void)
{
	uint8_t host_state = PI_Comm_GetHostStateFlags();
	uint8_t timeout_fault = PI_Comm_HasHeartbeatTimeout();

	OLED_Draw_Line("link/component err", 1, true, false);
	if (timeout_fault || !(host_state & 0x01))
	{
		OLED_Draw_Line("ERR: PI LINK LOST ", 2, false, false);
	}
	else if (!(host_state & 0x02))
	{
		OLED_Draw_Line("ERR: LIDAR LOST   ", 2, false, false);
	}
	else if (!(host_state & 0x04))
	{
		OLED_Draw_Line("ERR: SYSTEM LOST  ", 2, false, false);
	}
	else
	{
		OLED_Draw_Line("ERR: UNKNOWN      ", 2, false, false);
	}
	OLED_Draw_Line("fallback: normal  ", 3, false, false);
	OLED_Draw_Line("wait recover...   ", 4, false, true);
}

int main(void)
{	
	System_Run_State system_state = SYS_WAIT_PI_READY;

	bsp_init();//基本外设初始化 //Basic peripheral initialization
	MPU6050_EXTI_Init();		//此中断服务函数放到最后  //This interrupt service function is placed last


	while(1)
	{
		if (Key1_Long_Press(2))
		{
			Balance_Run_Enabled = 0;
			Stop_Flag = 1;
			Move_X = 0;
			Move_Z = 0;
			PI_Comm_SendEventCode(0x14);
			system_state = SYS_SHUTDOWN_WAIT;
			Show_Shutdown_State();
		}

		if(system_state == SYS_WAIT_PI_READY)
		{
			Show_Pi_Init_State();
			if(PI_Comm_IsSystemReady())
			{
				system_state = SYS_MODE_SELECT;
			}
			continue;
		}

		if(system_state == SYS_MODE_SELECT)
		{
			Balance_Run_Enabled = 0;
			Stop_Flag = 1;
			Move_X = 0;
			Move_Z = 0;
			PI_Comm_SendEventCode(0x11);
			Mode_select();
			bsp_mode_init();
			Show_Wait_Start_State();
			system_state = SYS_WAIT_START;
			continue;
		}

		if(system_state == SYS_WAIT_START)
		{
			Balance_Run_Enabled = 0;
			Show_Wait_Start_State();
			if(!PI_Comm_IsSystemReady())
			{
				Restore_Normal_Mode_Safe();
				Show_Fault_Recovery_State();
				system_state = SYS_FAULT_RECOVERY;
				continue;
			}
			if(Key1_State(1))
			{
				Balance_Run_Enabled = 1;
				Stop_Flag = 0;
				PI_Comm_SendEventCode(0x10);
				OLED_Draw_Line("running...         ", 2, false, true);
				system_state = SYS_RUNNING;
			}
			continue;
		}

		if(system_state == SYS_FAULT_RECOVERY)
		{
			Balance_Run_Enabled = 0;
			Show_Fault_Recovery_State();
			if(PI_Comm_IsSystemReady())
			{
				Show_Wait_Start_State();
				system_state = SYS_WAIT_START;
			}
			continue;
		}

		if(system_state == SYS_SHUTDOWN_WAIT)
		{
			Balance_Run_Enabled = 0;
			Show_Shutdown_State();
			if(PI_Comm_GetHostStateFlags() & 0x08)
			{
				OLED_Draw_Line("pi shutdown ok     ", 2, false, false);
				OLED_Draw_Line("now cut main power ", 3, false, true);
			}
			continue;
		}

		if(Key1_State(1))
		{
			Balance_Run_Enabled = 0;
			Stop_Flag = 1;
			Move_X = 0;
			Move_Z = 0;
			system_state = SYS_MODE_SELECT;
			continue;
		}

		if(system_state == SYS_RUNNING)
		{
			if(PI_Comm_HasHeartbeatTimeout() || !PI_Comm_IsSystemReady())
			{
				Restore_Normal_Mode_Safe();
				Show_Fault_Recovery_State();
				system_state = SYS_FAULT_RECOVERY;
				continue;
			}
		}

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
			if(Stop_Flag)
			{
				OLED_Draw_Line("paused             ", 2, false, true);
			}
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


