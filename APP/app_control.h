#ifndef __APP_CONTROL_H_
#define __APP_CONTROL_H_

#include "ALLHeader.h" 

#define MPU6050_INT PAin(12)   //PA12连接到MPU6050的中断引脚  PA12 is connected to the interrupt pin of MPU6050

void Get_Angle(u8 way);
u16 get_time_int(void);
void delay_time_int(u16 time);
void set_time_int(u16 time);

int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
int Put_Down(float Angle,int encoder_left,int encoder_right);

#endif

