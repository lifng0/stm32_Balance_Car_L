#ifndef __BSP_USART2_H_
#define __BSP_USART2_H_

#include "AllHeader.h"

void USART2_init(u32 baudrate);
void USART2_Send_U8(uint8_t ch);
void USART2_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);

#endif

