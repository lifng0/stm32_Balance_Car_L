#include "app_k210.h"


char buf_msg[20] = {'\0'};
uint8_t g_new_flag = 0;
uint8_t g_index = 0; 
uint8_t g_new_data = 0; //1:���ݽ������ 1: Data reception completed
static char g_last_k210_text[21] = {'\0'};
static uint8_t g_last_k210_text_valid = 0;

static void K210_SaveLastText(void)
{
	memcpy(g_last_k210_text, buf_msg, sizeof(buf_msg));
	g_last_k210_text[sizeof(g_last_k210_text) - 1] = '\0';
	g_last_k210_text_valid = (uint8_t)(g_last_k210_text[0] != '\0');
}

//Function function: Retain the information of k210
//Pass in function: recv-msg: Information sent from serial port
// ��������:����k210����Ϣ
// ���뺯��:recv_msg:���ڷ�������Ϣ
void Deal_K210_QR(uint8_t recv_msg)
{
	if (recv_msg == '$' && g_new_flag == 0)
	{
		g_new_flag = 1;
		memset(buf_msg, 0, sizeof(buf_msg)); // Clear old data ���������
		return;
	}
	if(g_new_flag == 1)
	{
		if (recv_msg == '#')
		{
			K210_SaveLastText();
			g_new_flag = 0;
			g_index = 0;
			g_new_data = 1;
		}

		if (g_new_flag == 1 && recv_msg != '$')
		{
			buf_msg[g_index++] = recv_msg;

			if(g_index > 20) //������� Array overflow
			{
				g_index = 0;
				g_new_flag = 0;
				g_new_data = 0;
				memset(buf_msg, 0, sizeof(buf_msg)); // Clear old data ���������
			}

		}
	}
}



#define Trun_speed 400  //��ֵ��pid�����Ĵ�С��һ���Ĺ�ϵ  This value has a certain relationship with the size of the pid parameter
#define Go_speed 15

/*
 * �������ܣ�����k210�����Ĳ�ָͬ����в�ͬ�Ķ���
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
			//��������  Buzzer sounds
			BEEP_BEEP = 1;
			delay_time(20); //200ms
			BEEP_BEEP = 0;
			//С�����������ֹͣ The car moves back for two seconds and then stops
			Move_X = -Go_speed;
			my_delay(2);
			Move_X = 0;
		}
		else if (strcmp("goahead", buf_msg) == 0 )
		{
			//��������  Buzzer sounds
			BEEP_BEEP = 1;
			delay_time(20); //200ms
			BEEP_BEEP = 0;
			//С�����������ֹͣ  The car moves back for two seconds and then stops
			Move_X = Go_speed;
			my_delay(2);
			Move_X = 0;
		}
		else if (strcmp("turnleft", buf_msg) == 0)
		{
			//�������� Buzzer sounds
			BEEP_BEEP = 1;
			delay_time(20); //200ms
			BEEP_BEEP = 0;
			//С����ת1s  The car turns left for 1s
			Move_Z = -Trun_speed;
			my_delay(1);
			Move_Z = 0;
			
		}
		else if (strcmp("turnright", buf_msg) == 0 )
		{
			//��������  Buzzer sounds
			BEEP_BEEP = 1;
			delay_time(20); //200ms
			BEEP_BEEP = 0;
			//С����ת1s The car turns right for 1s
			Move_Z = Trun_speed;
			my_delay(1);
			Move_Z = 0;
			
		}
		else if (strcmp("buzzer", buf_msg) == 0 )
		{
			//��������3��  The buzzer sounds 3 times
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


//����ѧϰ
//Self directed learning
void Deal_K210_self(uint8_t recv_msg)
{
	if (recv_msg == '$' && g_new_flag == 0)
	{
		g_new_flag = 1;
		memset(buf_msg, 0, sizeof(buf_msg)); // Clear old data ���������
		return;
	}
	if(g_new_flag == 1)
	{
		if (recv_msg == '#')
		{
			K210_SaveLastText();
			g_new_flag = 0;
			g_index = 0;
			g_new_data = 1;
		}

		if (g_new_flag == 1 && recv_msg != '$')
		{
			buf_msg[g_index++] = recv_msg;

			if(g_index > 20) //������� Array overflow
			{
				g_index = 0;
				g_new_flag = 0;
				g_new_data = 0;
				memset(buf_msg, 0, sizeof(buf_msg)); // Clear old data ���������
			}

		}
	}
}



#define Trun_speed_self 400  //��ֵ��pid�����Ĵ�С��һ���Ĺ�ϵ This value has a certain relationship with the size of the pid parameter
#define Go_speed_self 15
/*
 * �������ܣ�����k210�����Ĳ�ָͬ����в�ͬ�Ķ���
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
			//С��ǰ�������ֹͣ  The car moves forward for two seconds and then stops
			Move_X = Trun_speed_self;
			my_delay(2);
			Move_X = 0;
		}
		else if (strcmp("2", buf_msg) == 0)
		{
			//С����ת1sȻ��ǰ��1���ֹͣ The car turns left for 1 second and then moves forward for 1 second before stopping
			Move_Z = -Trun_speed_self;
			my_delay(1);
			
			Move_Z = 0;
			Move_X = Go_speed_self;
			
			my_delay(1);
			Move_X = 0;
		}
		else if (strcmp("3", buf_msg) == 0 )
		{
			//С����ת1sȻ��ǰ��1���ֹͣ  The car turns right for 1 second and then moves forward for 1 second before stopping
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
		memset(buf_msg, 0, sizeof(buf_msg)); // Clear old data ���������
		return;
	}
	if(g_new_flag == 1)
	{
		if (recv_msg == '#')
		{
			K210_SaveLastText();
			g_new_flag = 0;
			g_index = 0;
			g_new_data = 1;
		}

		if (g_new_flag == 1 && recv_msg != '$')
		{
			buf_msg[g_index++] = recv_msg;

			if(g_index > 20) //������� Array overflow
			{
				g_index = 0;
				g_new_flag = 0;
				g_new_data = 0;
				memset(buf_msg, 0, sizeof(buf_msg)); // Clear old data ���������
			}

		}
	}
}


#define Trun_speed_minst 400  //��ֵ��pid�����Ĵ�С��һ���Ĺ�ϵ This value has a certain relationship with the size of the pid parameter

void Change_state_minst(void)
{
	if(g_new_data == 1)
	{
		g_new_data = 0;  
		if (strcmp("6", buf_msg) == 0 )
		{
			OLED_Draw_Line("num:6!  ", 3, false, true);
			//��������1s Buzzer sounds for 1s
			BEEP_BEEP = 1;
			my_delay(1);
			BEEP_BEEP = 0;
			
		}
		else if (strcmp("2", buf_msg) == 0)
		{
			OLED_Draw_Line("num:2!  ", 3, false, true);
			//С����ת2sȻ��ֹͣ  The car turns left for 2 seconds and then stops
			Move_Z = -Trun_speed_minst;
			my_delay(1);
			my_delay(1);
			Move_Z = 0;
			
		}
		else if (strcmp("3", buf_msg) == 0 )
		{
			OLED_Draw_Line("num:3!  ", 3, false, true);
			//С����ת2sȻ��ֹͣ The car turns right for 2 seconds and then stops
			Move_Z = Trun_speed_minst;
			my_delay(1);
			my_delay(1);
			Move_Z = 0;
			
		}
		
	}

}

uint8_t K210_GetLastText(char *buffer, uint8_t buffer_len)
{
	uint8_t copy_len;

	if (buffer == 0 || buffer_len == 0 || g_last_k210_text_valid == 0)
	{
		return 0;
	}

	copy_len = (uint8_t)strlen(g_last_k210_text);
	if (copy_len >= buffer_len)
	{
		copy_len = buffer_len - 1;
	}
	memcpy(buffer, g_last_k210_text, copy_len);
	buffer[copy_len] = '\0';
	return copy_len;
}

uint8_t K210_HasLastText(void)
{
	return g_last_k210_text_valid;
}







