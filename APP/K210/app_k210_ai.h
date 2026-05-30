#ifndef __APP_K210_AI_H_
#define __APP_K210_AI_H_

#include "AllHeader.h"

typedef struct K210_Data
{
	uint16_t k210_X ; //识别框x轴的中心点 Identify the center point of the x-axis of the box
	uint16_t k210_Y ; //识别框y轴的中心点 Identify the center point of the y-axis of the box
	uint16_t k210_W ;//宽度  width
	uint16_t k210_H ;//高度  height
	uint16_t k210_area ;//面积  area
}K210_Data_t;

extern K210_Data_t K210_data;

void Deal_K210_AI(uint8_t recv_msg);
void Get_K210_Data(void);
float APP_K210Y_PID_Calc(float actual_value);

#endif

