#ifndef __APP_MODE_H_
#define __APP_MODE_H_

#include "AllHeader.h"


void Mode_select(void);
void car_mode(int16_t cnt);
void Set_Mid_Angle(void);
void Set_angle(void);
void Set_PID(void);
void Set_control_speed(void);

extern uint8_t angle_max;

#endif


