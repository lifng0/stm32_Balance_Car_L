#include "app_mode.h"

uint8_t angle_max = 40;
Car_Mode mode = Normal;//Normal; //模式为正常

static uint8_t Mode_IsSelectable(Car_Mode candidate)
{
	return (uint8_t)(
		candidate == Normal ||
		candidate == Weight_M ||
		candidate == K210_Line ||
		candidate == K210_Follow ||
		candidate == Lidar_Follow
	);
}

static Car_Mode Mode_Next(Car_Mode current)
{
	switch (current)
	{
		case Normal:
			return Weight_M;
		case Weight_M:
			return K210_Line;
		case K210_Line:
			return K210_Follow;
		case K210_Follow:
			return Lidar_Follow;
		case Lidar_Follow:
		default:
			return Normal;
	}
}

static Car_Mode Mode_Previous(Car_Mode current)
{
	switch (current)
	{
		case Normal:
			return K210_Follow;
		case Weight_M:
			return Normal;
		case K210_Line:
			return Weight_M;
		case K210_Follow:
			return K210_Line;
		case Lidar_Follow:
		default:
			return K210_Follow;
	}
}


//模式选择 用手拧轮子来进行模式切换
//Mode selection: Use the hand to twist the wheel to switch modes
void Mode_select(void)
{
	int16_t mode_cnt = 0;
	OLED_Draw_Line("1.Standard Mode", 1, true, true); 

	while(!Key1_State(1)) 
	{
		mode_cnt +=Read_Encoder(MOTOR_ID_ML);
		mode_cnt +=-Read_Encoder(MOTOR_ID_MR);
		car_mode(mode_cnt);//模式选择 Mode selection
		show_mode_oled();//oled显示模式 oled display mode
		
	}
	
	Set_Mid_Angle();//模式设置好后,设置机械中值 After setting the mode, set the mechanical median value
	Set_angle();//设置跌倒倾角 Set the inclination angle for falls
	Set_control_speed();//设置遥控的速度 Set the speed of the remote control


	Set_PID();//某些模式的需要特殊设置一下平衡pid Some modes require special settings for balancing PID

}


void car_mode(int16_t cnt)
{
	static int16_t cnt_old ;
	
	if(myabs(myabs(cnt)-myabs(cnt_old))>250)
	{
		if(cnt < cnt_old)
		{
			mode = Mode_Previous(mode);
		}
		else
		{
			mode = Mode_Next(mode);
		}
		
		cnt_old = cnt; //赋值  Assignment
//		printf("%d\r\n",mode);
	}
	
}


//根据模式设置机械中值 Set the mechanical median according to the mode
void Set_Mid_Angle(void)
{
	switch ((uint8_t)mode)
	{
		case Normal:   	
		case Weight_M:  		 	
			Mid_Angle = 0;
			break;
		
		case K210_Line:  	 		
		case K210_Follow:
		case Lidar_Follow:
			Mid_Angle = -1;
			break;

		default:
			Mid_Angle = 0;
			break;
	}

}

//遥控的速度初始化
//Speed initialization of remote control
void Set_control_speed()
{
	if(mode == Normal || mode == Weight_M) //正常和负重模式都需要初始化该值  Both normal and load modes need to initialize this value
	{
		Car_Target_Velocity=30;
		Car_Turn_Amplitude_speed=36;
	}
	else
	{
		Car_Target_Velocity=0;
		Car_Turn_Amplitude_speed=0;
	}
}

//设置跌倒的倾角
//Set the inclination angle for falls
void Set_angle(void)
{
	if((mode == K210_Line)||(mode == K210_Follow)||(mode == Lidar_Follow))
	{
		angle_max = 30;
	}
	else
	{
		angle_max = 40;
	}

}


extern float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki,Turn_Kp,Turn_Kd; //引入立直环、速度环、转向环 //Introduce vertical rings, speed rings, and steering rings
void Set_PID(void)
{
	if(mode == Weight_M || mode == K210_Follow || mode == Lidar_Follow) //负重 / 自动追踪 Load bearing / autonomous follow
	{
		
		Balance_Kp =9600;
		Balance_Kd =75 ; 
		Velocity_Kp=7000; 
	  Velocity_Ki=35;  
	  Turn_Kp=1400; 
		Turn_Kd=20;

	}
	else if(mode == K210_Line)
	{
		Balance_Kp =12000;
		Balance_Kd =72 ;

		Velocity_Kp=8000; 
		Velocity_Ki=40;  

		Turn_Kp=2500; 
		Turn_Kd=20;

	}
	else
	{
		Balance_Kp =9600;
		Balance_Kd =48 ; 

		Velocity_Kp =6200; 
		Velocity_Ki =31;  

		Turn_Kp =1700; 
		Turn_Kd =20;
	}

}

