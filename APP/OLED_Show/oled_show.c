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
		case K210_Line: OLED_Draw_Line("3.Vision Line", 1, true, true);  	 		  break;
		case K210_Follow: OLED_Draw_Line("4.Vision Follow", 1, true, true);     break;
		case Lidar_Follow: OLED_Draw_Line("5.Lidar Follow", 1, true, true);     break;
	}
	
}

