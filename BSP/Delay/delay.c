#include "delay.h"


static uint8_t fac_us = 0;  //us延时倍乘数 us delay multiplier
static uint16_t fac_ms = 0; //ms延时倍乘数 ms delay multiplier

void delay_init(void)
{
	uint8_t SYSCLK = SystemCoreClock / 1000000;
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); // 选择外部时钟  HCLK/8  Select external clock HCLK/8
	fac_us = SYSCLK / 8;
	fac_ms = (uint16_t)fac_us * 1000;
}


/**********************************************************
** Function name: delay_ms
** Function description: Delay nms
** Input parameter: nms
** Output parameter: None
** Note: SysTick->LOAD is a 24-bit register, so the maximum delay is:
	 nms<=0xffffff*8*1000/SYSCLK
	 SYSCLK unit is Hz, nms unit is ms
	 Under 72M conditions, nms<=1864
** 函数名: delay_ms
** 功能描述: 延时nms
** 输入参数: nms
** 输出参数: 无
** 说明：SysTick->LOAD为24位寄存器,所以,最大延时为:
		nms<=0xffffff*8*1000/SYSCLK
		SYSCLK单位为Hz,nms单位为ms
		对72M条件下,nms<=1864 
***********************************************************/
void delay_ms(uint16_t nms)
{
	uint32_t temp;
	SysTick->LOAD = (uint32_t)nms * fac_ms; //时间加载(SysTick->LOAD为24bit) Time loading (SysTick->LOAD is 24bit)
	SysTick->VAL = 0x00;			   //清空计数器 Clear counter
	SysTick->CTRL = 0x01;			   //开始倒数  Start countdown
	do
	{
		temp = SysTick->CTRL;
	} while (temp & 0x01 && !(temp & (1 << 16))); //等待时间到达 Waiting time to arrive
	SysTick->CTRL = 0x00;						  //关闭计数器  Close Counter
	SysTick->VAL = 0X00;						  //清空计数器  Clear counter
}

/**********************************************************
** Function name: delay_us
** Function description: Delay nus, nus is the number of us to delay.
** Input parameter: nus
** Output parameter: None
** 函数名: delay_us
** 功能描述: 延时nus，nus为要延时的us数.
** 输入参数: nus
** 输出参数: 无
***********************************************************/
void delay_us(uint32_t nus)
{
	uint32_t temp;
	SysTick->LOAD = nus * fac_us; //时间加载  Time loading
	SysTick->VAL = 0x00;		  //清空计数器 Clear counter
	SysTick->CTRL = 0x01;		  //开始倒数 Start countdown
	do
	{
		temp = SysTick->CTRL;
	} while (temp & 0x01 && !(temp & (1 << 16))); //等待时间到达 Waiting time to arrive
	SysTick->CTRL = 0x00;						  //关闭计数器 Close Counter
	SysTick->VAL = 0X00;						  //清空计数器 Clear counter
}




