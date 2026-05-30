#include "app_k210.h"


char buf_msg[20] = {'\0'};
uint8_t g_new_flag = 0;
uint8_t g_index = 0; 
uint8_t g_new_data = 0; //1:数据接收完成 1: Data reception completed

//Function function: Retain the information of k210
//Pass in function: recv-msg: Information sent from serial port
// 函数功能:保留k210的信息
// 传入函数:recv_msg:串口发来的信息
void Deal_K210_QR(uint8_t recv_msg)
{
	if (recv_msg == '$' && g_new_flag == 0)
	{
		g_new_flag = 1;
		memset(buf_msg, 0, sizeof(buf_msg)); // Clear old data 清除旧数据
		return;
	}
	if(g_new_flag == 1)
	{
		if (recv_msg == '#')
		{
			g_new_flag = 0;
			g_index = 0;
			g_new_data = 1;
		}

		if (g_new_flag == 1 && recv_msg != '$')
		{
			buf_msg[g_index++] = recv_msg;

			if(g_index > 20) //数组溢出 Array overflow
			{
				g_index = 0;
				g_new_flag = 0;
				g_new_data = 0;
				memset(buf_msg, 0, sizeof(buf_msg)); // Clear old data 清除旧数据
			}

		}
	}
}



#define Trun_speed 400  //此值和pid参数的大小有一定的关系  This value has a certain relationship with the size of the pid parameter
#define Go_speed 15

/*
 * 函数功能：根据k210发来的不同指令进行不同的动作
 *
 *Function: perform different actions according to different instructions sent by k210
 * 
*/
void Change_state_QR(void)
{
	if(g_new_data == 1)
	{
		g_new_data = 0;  
		if (strcmp("goback", buf_msg) == 0 )
		{
			//蜂鸣器响  Buzzer sounds
			BEEP_BEEP = 1;
			delay_time(20); //200ms
			BEEP_BEEP = 0;
			//小车后退两秒后停止 The car moves back for two seconds and then stops
			Move_X = -Go_speed;
			my_delay(2);
			Move_X = 0;
		}
		else if (strcmp("goahead", buf_msg) == 0 )
		{
			//蜂鸣器响  Buzzer sounds
			BEEP_BEEP = 1;
			delay_time(20); //200ms
			BEEP_BEEP = 0;
			//小车后退两秒后停止  The car moves back for two seconds and then stops
			Move_X = Go_speed;
			my_delay(2);
			Move_X = 0;
		}
		else if (strcmp("turnleft", buf_msg) == 0)
		{
			//蜂鸣器响 Buzzer sounds
			BEEP_BEEP = 1;
			delay_time(20); //200ms
			BEEP_BEEP = 0;
			//小车左转1s  The car turns left for 1s
			Move_Z = -Trun_speed;
			my_delay(1);
			Move_Z = 0;
			
		}
		else if (strcmp("turnright", buf_msg) == 0 )
		{
			//蜂鸣器响  Buzzer sounds
			BEEP_BEEP = 1;
			delay_time(20); //200ms
			BEEP_BEEP = 0;
			//小车右转1s The car turns right for 1s
			Move_Z = Trun_speed;
			my_delay(1);
			Move_Z = 0;
			
		}
		else if (strcmp("buzzer", buf_msg) == 0 )
		{
			//蜂鸣器响3次  The buzzer sounds 3 times
			for (u8 i =0;i<3;i++)
			{
				BEEP_BEEP = 1;
				delay_time(20); //200ms
				BEEP_BEEP = 0;
				delay_time(20); //200ms
			}
			
		}
		
	}

}


//自主学习
//Self directed learning
void Deal_K210_self(uint8_t recv_msg)
{
	if (recv_msg == '$' && g_new_flag == 0)
	{
		g_new_flag = 1;
		memset(buf_msg, 0, sizeof(buf_msg)); // Clear old data 清除旧数据
		return;
	}
	if(g_new_flag == 1)
	{
		if (recv_msg == '#')
		{
			g_new_flag = 0;
			g_index = 0;
			g_new_data = 1;
		}

		if (g_new_flag == 1 && recv_msg != '$')
		{
			buf_msg[g_index++] = recv_msg;

			if(g_index > 20) //数组溢出 Array overflow
			{
				g_index = 0;
				g_new_flag = 0;
				g_new_data = 0;
				memset(buf_msg, 0, sizeof(buf_msg)); // Clear old data 清除旧数据
			}

		}
	}
}



#define Trun_speed_self 400  //此值和pid参数的大小有一定的关系 This value has a certain relationship with the size of the pid parameter
#define Go_speed_self 15
/*
 * 函数功能：根据k210发来的不同指令进行不同的动作
 *
 *Function: perform different actions according to different instructions sent by k210
 * 
*/
void Change_state_self(void)
{
	if(g_new_data == 1)
	{
		g_new_data = 0;  
		if (strcmp("1", buf_msg) == 0 )
		{
			//小车前进两秒后停止  The car moves forward for two seconds and then stops
			Move_X = Trun_speed_self;
			my_delay(2);
			Move_X = 0;
		}
		else if (strcmp("2", buf_msg) == 0)
		{
			//小车左转1s然后前进1秒后停止 The car turns left for 1 second and then moves forward for 1 second before stopping
			Move_Z = -Trun_speed_self;
			my_delay(1);
			
			Move_Z = 0;
			Move_X = Go_speed_self;
			
			my_delay(1);
			Move_X = 0;
		}
		else if (strcmp("3", buf_msg) == 0 )
		{
			//小车右转1s然后前进1秒后停止  The car turns right for 1 second and then moves forward for 1 second before stopping
			Move_Z = Trun_speed_self;
			my_delay(1);
			
			Move_Z = 0;
			Move_X = Go_speed_self;
		
			my_delay(1);
			Move_X = 0;
		}
		
	}

}


void Deal_K210_minst(uint8_t recv_msg)
{
	if (recv_msg == '$' && g_new_flag == 0)
	{
		g_new_flag = 1;
		memset(buf_msg, 0, sizeof(buf_msg)); // Clear old data 清除旧数据
		return;
	}
	if(g_new_flag == 1)
	{
		if (recv_msg == '#')
		{
			g_new_flag = 0;
			g_index = 0;
			g_new_data = 1;
		}

		if (g_new_flag == 1 && recv_msg != '$')
		{
			buf_msg[g_index++] = recv_msg;

			if(g_index > 20) //数组溢出 Array overflow
			{
				g_index = 0;
				g_new_flag = 0;
				g_new_data = 0;
				memset(buf_msg, 0, sizeof(buf_msg)); // Clear old data 清除旧数据
			}

		}
	}
}


#define Trun_speed_minst 400  //此值和pid参数的大小有一定的关系 This value has a certain relationship with the size of the pid parameter

void Change_state_minst(void)
{
	if(g_new_data == 1)
	{
		g_new_data = 0;  
		if (strcmp("6", buf_msg) == 0 )
		{
			OLED_Draw_Line("num:6!  ", 3, false, true);
			//蜂鸣器响1s Buzzer sounds for 1s
			BEEP_BEEP = 1;
			my_delay(1);
			BEEP_BEEP = 0;
			
		}
		else if (strcmp("2", buf_msg) == 0)
		{
			OLED_Draw_Line("num:2!  ", 3, false, true);
			//小车左转2s然后停止  The car turns left for 2 seconds and then stops
			Move_Z = -Trun_speed_minst;
			my_delay(1);
			my_delay(1);
			Move_Z = 0;
			
		}
		else if (strcmp("3", buf_msg) == 0 )
		{
			OLED_Draw_Line("num:3!  ", 3, false, true);
			//小车右转2s然后停止 The car turns right for 2 seconds and then stops
			Move_Z = Trun_speed_minst;
			my_delay(1);
			my_delay(1);
			Move_Z = 0;
			
		}
		
	}

}






