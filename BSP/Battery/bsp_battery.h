#ifndef __BSP_BATTERY_H_
#define __BSP_BATTERY_H_

#include "AllHeader.h"


#define BAT_GPIO_CLK    RCC_APB2Periph_GPIOA
#define BAT_GPIO_PORT   GPIOA
#define BAT_GPIO_PIN    GPIO_Pin_5


#define BAT_ADC         ADC1
#define BAT_ADC_CH      ADC_Channel_5
#define BAT_ADC_CLK     RCC_APB2Periph_ADC1

void Battery_init(void);
float Get_Measure_Volotage(void);
float Get_Battery_Volotage(void);

uint16_t Battery_Get_Average(uint8_t ch, uint8_t times);

#endif

