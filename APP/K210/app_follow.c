#include "app_follow.h"
#include "app_k210.h"

//前进方向 Forward direction
#define GO_PID_KP       (0.001) 
#define GO_PID_KD       (0.0005) //0.0008

//单独的转弯   Separate turns
//#define Trun_PID_KP       (3.3)//3.3
//#define Trun_PID_KI       (0.001)
//#define Trun_PID_KD       (0.7)//0.4

#define Trun_PID_KP       (2.15)//3.3
#define Trun_PID_KI       (0.0001)
#define Trun_PID_KD       (0.2)//0.4


#define K210_Minddle_area    (6500) //物体面积的大小，根据自己的物体修改 The size of the object area can be modified according to one's own object
#define K210_Minddle_X       (160)
#define K210_Minddle_Y       (120)

int16_t g_error_x,g_error_y,g_error_area;

float my_abs(float num)
{
	if(num < 0)
		num = -num;
	return num;
}


//前进方向的位置pid The position pid in the forward direction
static float GO_PID(void)
{
	static int16_t error,Last_error;
	static float k210go;
	error= g_error_area;                      	//计算偏差 物体面积 Calculate the area of the deviated object
	
	k210go=GO_PID_KP*error+GO_PID_KD*(error-Last_error);//位置式PID控制器  Position PID controller
	
	Last_error=error;                                       		 			//保存上一次偏差 Save last deviation
	return k210go;       

}

//转弯方向的位置pid Position PID of turning direction
static float Turn_PID(void)
{
	static int16_t error,Last_error;
	static float k210Turn,Integral_error;
	
	error=g_error_x;                         				 //计算偏差 Calculate deviation
	
	if(Integral_error>700) Integral_error=700;
	else if(Integral_error<-700) Integral_error=-700;
	k210Turn=-Trun_PID_KP*error-Trun_PID_KI*Integral_error-Trun_PID_KD*(error-Last_error);	//位置式PID控制器 Position based PID controller
	Last_error=error;                                       					 				//保存上一次偏差 Save the previous deviation
	
	if(Turn_Off(Angle_Balance,battery)== 1)								//电机关闭，此时积分清零  The motor is turned off, and the integral is reset to zero at this time
		Integral_error = 0;
                                        					 
	
	return k210Turn;  //输出 output

}


//函数功能:结合k210做颜色跟随 
//Function function: Combining with K210 for color following
void APP_K210X_Y_Follow_PID(void)
{
	g_error_x = K210_Minddle_X-K210_data.k210_X;   
//	g_error_y = K210_Minddle_Y-K210_data.k210_Y; 
	
	
  if(K210_data.k210_area > K210_Minddle_area*2) K210_data.k210_area = K210_Minddle_area*2; //限制一下面积大小，防止后退才快 Limit the area size to prevent rapid retreat
	
	g_error_area  =  K210_Minddle_area-K210_data.k210_area;
	

	
	if(K210_data.k210_area == 0)//但面积为0的时候,说明识别不出物体 But when the area is 0, it means that the object cannot be recognized
	{
		Move_X = 0;
		Move_Z = 0;
		return;
	}
	
	
	
	if((g_error_area > 1500) || (g_error_area < -4000)) //面积死区  不是对称的不能用myabs 中心面积越大越要注意  The dead zone of the area is not symmetrical and cannot be used with myabs. The larger the center area, the more attention should be paid
	{			
		Move_X = GO_PID();
	}
	else
	{
		Move_X = 0;

	}
	
	
	
////	if((my_abs(g_error_x )> 40))//不在死区内 Not in dead zone
//	{
		Move_Z = Turn_PID();
//	}
////	else
////	{
////		Move_Z = 0;
////	}
//	

	
	if(Move_X > 30)Move_X = 30;
	else if(Move_X < -30)Move_X = -30;
	
	
	
}






