#include "oled_show.h"


void show_mode_oled(void)
{
	static uint8_t mode_old = 0;
	
	if(mode == mode_old) return; 
	
	//不相等赋值  Unequal assignment
	mode_old = (uint8_t)mode;
		
	switch (mode_old)
	{
		case Normal: OLED_Draw_Line("1.Standard Mode", 1, true, true);  				 			break;
		case Weight_M: OLED_Draw_Line("2.Load Movement", 1, true, true);   		 			break;
		case K210_QR: OLED_Draw_Line("3.K210 QR Rec", 1, true, true);  			 			break;
		case K210_Line: OLED_Draw_Line("4.K210 Track", 1, true, true);  	 		  break;
		case K210_Follow: OLED_Draw_Line("5.K210 Follow", 1, true, true);     break;
		case K210_SelfLearn: OLED_Draw_Line("6.K210 Self Learn", 1, true, true);    break;
		case K210_mnist: OLED_Draw_Line("7.K210 Num Rec", 1, true, true);     	break;
	}
	
}

