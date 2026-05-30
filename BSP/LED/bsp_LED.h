#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "AllHeader.h"

#define LED_RCC   RCC_APB2Periph_GPIOB
#define LED_PORT  GPIOB
#define LED_PIN   GPIO_Pin_3
#define LED_ON    GPIO_SetBits(LED_PORT,LED_PIN) 
#define LED_OFF   GPIO_ResetBits(LED_PORT,LED_PIN)



#define LED  PBout(3) 
 

void init_led_gpio(void);

void init_gpio(void);
#endif
