#ifndef __BSP_PI_COMM_H__
#define __BSP_PI_COMM_H__

#include "stm32f10x.h"

void PI_Comm_Init(u32 baudrate);
void PI_Comm_10ms_Task(void);

#endif
