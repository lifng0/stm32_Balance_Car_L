#include "app_line.h"

#define Track_K210Speed 15 //15 3 0.1   // 20 5.5 0.3 

void Set_K210track_speed(void)
{
	Move_X = Track_K210Speed;       
}





#define K210_Trun_KP (3) //3 0-11
#define K210_Trun_KD (0.1) //0.1


#define K210_Minddle  160  //k210屏幕的x轴是320 所以中间值为160  The x-axis of the K210 screen is 320, so the median value is 160

//位置式pid  Positional pid
int Turn_K210_PD(float gyro)
{	
	int k210Turn = 0; 
	float K210x_median_err = 0;	  
	
	K210x_median_err=K210_data.k210_X-K210_Minddle;
	
	k210Turn=K210x_median_err*K210_Trun_KP+gyro*K210_Trun_KD;
	
	return k210Turn;

}

