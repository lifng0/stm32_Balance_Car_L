#include "app_mode.h"

uint8_t angle_max = 40;
Car_Mode mode = Normal;//Normal; //模式为正常


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
			mode = (Car_Mode)((mode - 1) %Mode_Max); //大到小 枚举(u8) -1即为255  Large to small   enumeration (u8) -1 is 255
			if(mode > Mode_Max)
			{
				mode = (Car_Mode)(Mode_Max -1);
			}
		}
		else
		{
			mode = (Car_Mode)((mode + 1) %Mode_Max); //小到大  Small to Large
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
		
		case K210_QR:   			
		case K210_Line:  	 		
		case K210_Follow:
		case K210_SelfLearn:
		case K210_mnist:
			Mid_Angle = -1;
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
}

//设置跌倒的倾角
//Set the inclination angle for falls
void Set_angle(void)
{
	if((mode == K210_QR)||(mode == K210_Line)||(mode == K210_Follow)||(mode == K210_SelfLearn)||(mode == K210_mnist))
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
	if(mode == Weight_M || mode == K210_Follow) //负重 Load bearing
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
	else if(mode == Normal)
	{
		Balance_Kp =9600;
		Balance_Kd =48 ; 

		Velocity_Kp =6200; 
		Velocity_Ki =31;  

		Turn_Kp =1700; 
		Turn_Kd =20;
	}

}

