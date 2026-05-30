#ifndef __APP_MOTOR_H_
#define __APP_MOTOR_H_

#include "ALLHeader.h" 

#define PI 3.14159265							//PI圆周率  PI π
#define Control_Frequency  200.0	//编码器读取频率  Encoder reading frequency
#define Diameter_67  67.0 				//轮子直径67mm   Wheel diameter 67mm
#define EncoderMultiples   4.0 		//编码器倍频数  Encoder multiples
#define Encoder_precision  11.0 	//编码器精度 11线  Encoder precision 11 lines
#define Reduction_Ratio  30.0			//减速比30  Reduction ratio 30
#define Perimeter  210.4867 			//周长，单位mm Perimeter, unit mm


void Set_Pwm(int motor_left,int motor_right);
int PWM_Limit(int IN,int max,int min);

void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);
uint8_t Turn_Off(float angle, float voltage);
	
int PWM_Ignore(int pulse);

#endif

