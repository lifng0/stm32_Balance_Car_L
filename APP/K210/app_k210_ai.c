#include "app_k210_ai.h"

uint8_t buf_msg_AI[20] = {'\0'};
uint8_t g_new_flag_AI = 0;
uint8_t g_index_AI = 0;
static uint8_t g_k210_ai_valid = 0;


K210_Data_t K210_data={160,120,0,0,0};//��ʼ������ֵ Initialize to median value

// ��������:����k210����Ϣ
// ���뺯��:recv_msg:���ڷ�������Ϣ
// Function function: Preserve information of k210
// Incoming function: recv_ Msg: Information sent from serial port
void Deal_K210_AI(uint8_t recv_msg)
{
	if (recv_msg == '$' && g_new_flag_AI == 0)
	{
		g_new_flag_AI = 1;
		memset(buf_msg_AI, 0, sizeof(buf_msg_AI)); // Clear old data ���������
		return;
	}

	if(g_new_flag_AI == 1)
	{
		if (recv_msg == '#')
		{
			g_new_flag_AI = 0;
			g_index_AI = 0;
			Get_K210_Data(); // New data received completed �����ݽ������
			g_k210_ai_valid = 1;
			memset(buf_msg_AI, 0, sizeof(buf_msg_AI)); // Clear old data ���������
		}

		else if (g_new_flag_AI == 1 && recv_msg != '$')
		{
			buf_msg_AI[g_index_AI++] = recv_msg;

			if(g_index_AI > 20) //������� Array overflow
			{
				g_index_AI = 0;
				g_new_flag_AI = 0;
				memset(buf_msg_AI, 0, sizeof(buf_msg_AI)); // Clear old data ���������
			}
		}
	}

}

uint8_t K210_HasAISnapshot(void)
{
	return g_k210_ai_valid;
}

void K210_GetAISnapshot(K210_Data_t *snapshot)
{
	if (snapshot == 0)
	{
		return;
	}
	*snapshot = K210_data;
}

//��������:��ȡʶ��ͼ������ĵ�X\Y����
//�������:��
//Function function: Obtain the X  Y coordinates of the center point of the recognition image
//Incoming parameter: None
void Get_K210_Data(void)
{
	K210_data.k210_X = (buf_msg_AI[0] -'0') *100 + (buf_msg_AI[1] -'0') *10 + (buf_msg_AI[2] -'0');

	K210_data.k210_Y = (buf_msg_AI[3] -'0') *100 + (buf_msg_AI[4] -'0') *10 + (buf_msg_AI[5] -'0');
	
	
	if(mode == K210_Follow)//����ֻ�и������  Only following can
	{
			K210_data.k210_W = (buf_msg_AI[6] -'0') *100 + (buf_msg_AI[7] -'0') *10 + (buf_msg_AI[8] -'0');
			K210_data.k210_H = (buf_msg_AI[9] -'0') *100 + (buf_msg_AI[10] -'0') *10 + (buf_msg_AI[11] -'0');
			K210_data.k210_area = K210_data.k210_W*K210_data.k210_H;
	}

}





