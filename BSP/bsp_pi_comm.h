#ifndef __BSP_PI_COMM_H__
#define __BSP_PI_COMM_H__

#include "stm32f10x.h"

void PI_Comm_Init(u32 baudrate);
void PI_Comm_10ms_Task(void);
uint8_t PI_Comm_GetHostStateFlags(void);
uint8_t PI_Comm_IsSystemReady(void);
void PI_Comm_SendEventCode(uint8_t event_code);

#endif
