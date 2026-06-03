#include "AllHeader.h"

#define PI_COMM_SOF1                 0xAA
#define PI_COMM_SOF2                 0x55
#define PI_COMM_VERSION              0x01
#define PI_COMM_MAX_PAYLOAD          32
#define PI_COMM_HEARTBEAT_TIMEOUT    50

#define PI_COMM_CMD_PING             0x01
#define PI_COMM_CMD_SET_ENABLE       0x02
#define PI_COMM_CMD_SET_MODE         0x03
#define PI_COMM_CMD_SET_MOVE         0x04
#define PI_COMM_CMD_QUERY_STATUS     0x05
#define PI_COMM_CMD_HEARTBEAT        0x06
#define PI_COMM_CMD_EMERGENCY_STOP   0x07
#define PI_COMM_CMD_SET_HOST_STATE   0x08
#define PI_COMM_CMD_QUERY_VISION     0x09
#define PI_COMM_CMD_K210_TEXT        0x0A

#define PI_COMM_CMD_ACK              0x81
#define PI_COMM_CMD_NACK             0x82
#define PI_COMM_CMD_STATUS           0x83
#define PI_COMM_CMD_EVENT            0x84
#define PI_COMM_CMD_HEARTBEAT_ACK    0x85
#define PI_COMM_CMD_VISION_STATUS    0x86

#define PI_VISION_NONE               0x00
#define PI_VISION_TEXT               0x01
#define PI_VISION_AI                 0x02

#define PI_COMM_ERR_CHECKSUM         0x01
#define PI_COMM_ERR_LENGTH           0x02
#define PI_COMM_ERR_ILLEGAL_CMD      0x03
#define PI_COMM_ERR_ILLEGAL_PARAM    0x04
#define PI_COMM_ERR_BUSY_STATE       0x05

#define PI_COMM_EVENT_LOW_POWER      0x01
#define PI_COMM_EVENT_POWER_RECOVER  0x02
#define PI_COMM_EVENT_TIMEOUT_STOP   0x04
#define PI_COMM_EVENT_START_REQUEST  0x10
#define PI_COMM_EVENT_MODE_SELECT    0x11
#define PI_COMM_EVENT_STOP_ASSERT    0x12
#define PI_COMM_EVENT_STOP_CLEAR     0x13
#define PI_COMM_EVENT_SHUTDOWN_REQ   0x14

#define PI_HOST_STATE_PI_READY       0x01
#define PI_HOST_STATE_LIDAR_READY    0x02
#define PI_HOST_STATE_SYSTEM_READY   0x04
#define PI_HOST_STATE_SHUTDOWN_ACK   0x08

typedef enum
{
	PI_RX_WAIT_SOF1 = 0,
	PI_RX_WAIT_SOF2,
	PI_RX_WAIT_VER,
	PI_RX_WAIT_CMD,
	PI_RX_WAIT_SEQ,
	PI_RX_WAIT_LEN,
	PI_RX_WAIT_PAYLOAD,
	PI_RX_WAIT_CHK
} PI_RxState;

static volatile PI_RxState pi_rx_state = PI_RX_WAIT_SOF1;
static volatile uint8_t pi_rx_cmd = 0;
static volatile uint8_t pi_rx_seq = 0;
static volatile uint8_t pi_rx_len = 0;
static volatile uint8_t pi_rx_index = 0;
static volatile uint8_t pi_rx_payload[PI_COMM_MAX_PAYLOAD];

static volatile uint16_t pi_timeout_ticks = 0;
static volatile uint8_t pi_supervision_active = 0;
static volatile uint8_t pi_timeout_latched = 0;
static uint8_t pi_last_power_flag = 0;
static uint8_t pi_last_stop_flag = 1;
static volatile uint8_t pi_host_state_flags = 0;
static uint8_t pi_k210_uart_ready = 0;

static void PI_Comm_ResetParser(void)
{
	pi_rx_state = PI_RX_WAIT_SOF1;
	pi_rx_cmd = 0;
	pi_rx_seq = 0;
	pi_rx_len = 0;
	pi_rx_index = 0;
}

static void PI_Comm_SendByte(uint8_t data)
{
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
	{
	}
	USART_SendData(USART3, data);
}

static void PI_Comm_SendArray(const uint8_t *buffer, uint8_t length)
{
	uint8_t i;
	for (i = 0; i < length; i++)
	{
		PI_Comm_SendByte(buffer[i]);
	}
}

static uint8_t PI_Comm_Checksum(uint8_t cmd, uint8_t seq, uint8_t len, const uint8_t *payload)
{
	uint8_t checksum = PI_COMM_VERSION ^ cmd ^ seq ^ len;
	uint8_t i;

	for (i = 0; i < len; i++)
	{
		checksum ^= payload[i];
	}

	return checksum;
}

static int16_t PI_Comm_BytesToInt16(const uint8_t *payload)
{
	return (int16_t)((payload[1] << 8) | payload[0]);
}

static void PI_Comm_PutInt16(uint8_t *buffer, uint8_t offset, int16_t value)
{
	buffer[offset] = (uint8_t)(value & 0xFF);
	buffer[offset + 1] = (uint8_t)((value >> 8) & 0xFF);
}

static float PI_Comm_ClampMove(float value)
{
	if (value > 30.0f)
	{
		return 30.0f;
	}
	if (value < -30.0f)
	{
		return -30.0f;
	}
	return value;
}

static void PI_Comm_RefreshWatchdog(void)
{
	pi_supervision_active = 1;
	pi_timeout_ticks = 0;
	pi_timeout_latched = 0;
}

static void PI_Comm_SendFrame(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint8_t len)
{
	uint8_t frame[PI_COMM_MAX_PAYLOAD + 7];
	uint8_t i;

	frame[0] = PI_COMM_SOF1;
	frame[1] = PI_COMM_SOF2;
	frame[2] = PI_COMM_VERSION;
	frame[3] = cmd;
	frame[4] = seq;
	frame[5] = len;

	for (i = 0; i < len; i++)
	{
		frame[6 + i] = payload[i];
	}

	frame[6 + len] = PI_Comm_Checksum(cmd, seq, len, payload);
	PI_Comm_SendArray(frame, (uint8_t)(7 + len));
}

static void PI_Comm_SendAck(uint8_t seq, uint8_t ack_cmd)
{
	uint8_t payload[2];
	payload[0] = ack_cmd;
	payload[1] = seq;
	PI_Comm_SendFrame(PI_COMM_CMD_ACK, seq, payload, 2);
}

static void PI_Comm_SendNack(uint8_t seq, uint8_t nack_cmd, uint8_t err_code)
{
	uint8_t payload[3];
	payload[0] = nack_cmd;
	payload[1] = seq;
	payload[2] = err_code;
	PI_Comm_SendFrame(PI_COMM_CMD_NACK, seq, payload, 3);
}

static void PI_Comm_SendStatus(uint8_t seq)
{
	uint8_t payload[11];
	int16_t move_x = (int16_t)(Move_X * 10.0f);
	int16_t move_z = (int16_t)(Move_Z * 10.0f);
	uint16_t battery_mv = (uint16_t)(battery * 100.0f);
	int16_t angle = (int16_t)(Angle_Balance * 10.0f);

	payload[0] = (uint8_t)mode;
	payload[1] = Stop_Flag;
	payload[2] = lower_power_flag;
	PI_Comm_PutInt16(payload, 3, move_x);
	PI_Comm_PutInt16(payload, 5, move_z);
	payload[7] = (uint8_t)(battery_mv & 0xFF);
	payload[8] = (uint8_t)((battery_mv >> 8) & 0xFF);
	PI_Comm_PutInt16(payload, 9, angle);
	PI_Comm_SendFrame(PI_COMM_CMD_STATUS, seq, payload, 11);
}

static void PI_Comm_SendVisionStatus(uint8_t seq)
{
	uint8_t payload[PI_COMM_MAX_PAYLOAD];
	uint8_t payload_len = 0;
	char text_buffer[21] = {'\0'};
	uint8_t text_len = 0;
	K210_Data_t ai_snapshot;

	memset(payload, 0, sizeof(payload));
	payload[1] = (uint8_t)mode;

	if ((mode == K210_QR) || (mode == K210_SelfLearn) || (mode == K210_mnist))
	{
		text_len = K210_GetLastText(text_buffer, sizeof(text_buffer));
		payload[0] = PI_VISION_TEXT;
		payload[2] = (uint8_t)(text_len > 0 ? 1 : 0);
		payload[3] = text_len;
		payload_len = 4;
		if (text_len > 0)
		{
			memcpy(&payload[4], text_buffer, text_len);
			payload_len = (uint8_t)(4 + text_len);
		}
	}
	else if ((mode == K210_Line) || (mode == K210_Follow))
	{
		payload[0] = PI_VISION_AI;
		payload[2] = K210_HasAISnapshot();
		K210_GetAISnapshot(&ai_snapshot);
		PI_Comm_PutInt16(payload, 3, (int16_t)ai_snapshot.k210_X);
		PI_Comm_PutInt16(payload, 5, (int16_t)ai_snapshot.k210_Y);
		PI_Comm_PutInt16(payload, 7, (int16_t)ai_snapshot.k210_W);
		PI_Comm_PutInt16(payload, 9, (int16_t)ai_snapshot.k210_H);
		PI_Comm_PutInt16(payload, 11, (int16_t)ai_snapshot.k210_area);
		payload_len = 13;
	}
	else
	{
		payload[0] = PI_VISION_NONE;
		payload[2] = 0;
		payload_len = 3;
	}

	PI_Comm_SendFrame(PI_COMM_CMD_VISION_STATUS, seq, payload, payload_len);
}

static void PI_Comm_SendEvent(uint8_t event_code)
{
	uint8_t payload[1];
	payload[0] = event_code;
	PI_Comm_SendFrame(PI_COMM_CMD_EVENT, 0, payload, 1);
}

static void PI_Comm_EnsureK210UartReady(void)
{
	if (pi_k210_uart_ready == 0)
	{
		USART2_init(115200);
		pi_k210_uart_ready = 1;
	}
}

static void PI_Comm_InitModePeripheral(Car_Mode target_mode)
{
	if (target_mode == Normal || target_mode == Weight_M)
	{
		bluetooth_init();
	}
	else if ((target_mode == K210_QR) || (target_mode == K210_Line) || (target_mode == K210_Follow) ||
	         (target_mode == K210_SelfLearn) || (target_mode == K210_mnist))
	{
		PI_Comm_EnsureK210UartReady();
	}
}

static void PI_Comm_HandleFrame(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint8_t len)
{
	float move_x_value;
	float move_z_value;

	switch (cmd)
	{
	case PI_COMM_CMD_PING:
		if (len != 0)
		{
			PI_Comm_SendNack(seq, cmd, PI_COMM_ERR_LENGTH);
			return;
		}
		PI_Comm_SendAck(seq, cmd);
		break;

	case PI_COMM_CMD_SET_ENABLE:
		if (len != 1)
		{
			PI_Comm_SendNack(seq, cmd, PI_COMM_ERR_LENGTH);
			return;
		}
		if (payload[0] > 1)
		{
			PI_Comm_SendNack(seq, cmd, PI_COMM_ERR_ILLEGAL_PARAM);
			return;
		}
		if (payload[0] == 0)
		{
			Stop_Flag = 1;
			Move_X = 0;
			Move_Z = 0;
		}
		else
		{
			Stop_Flag = 0;
		}
		PI_Comm_RefreshWatchdog();
		PI_Comm_SendAck(seq, cmd);
		break;

	case PI_COMM_CMD_SET_MODE:
		if (len != 1)
		{
			PI_Comm_SendNack(seq, cmd, PI_COMM_ERR_LENGTH);
			return;
		}
		if (payload[0] >= Mode_Max)
		{
			PI_Comm_SendNack(seq, cmd, PI_COMM_ERR_ILLEGAL_PARAM);
			return;
		}
		mode = (Car_Mode)payload[0];
		Move_X = 0;
		Move_Z = 0;
		Stop_Flag = 1;
		PI_Comm_InitModePeripheral(mode);
		PI_Comm_RefreshWatchdog();
		PI_Comm_SendAck(seq, cmd);
		break;

	case PI_COMM_CMD_SET_MOVE:
		if (len != 4)
		{
			PI_Comm_SendNack(seq, cmd, PI_COMM_ERR_LENGTH);
			return;
		}
		if (!(mode == Normal || mode == Weight_M))
		{
			PI_Comm_SendNack(seq, cmd, PI_COMM_ERR_BUSY_STATE);
			return;
		}
		move_x_value = (float)PI_Comm_BytesToInt16(payload) / 10.0f;
		move_z_value = (float)PI_Comm_BytesToInt16(payload + 2) / 10.0f;
		Move_X = PI_Comm_ClampMove(move_x_value);
		Move_Z = PI_Comm_ClampMove(move_z_value);
		PI_Comm_RefreshWatchdog();
		PI_Comm_SendAck(seq, cmd);
		break;

	case PI_COMM_CMD_QUERY_STATUS:
		if (len != 0)
		{
			PI_Comm_SendNack(seq, cmd, PI_COMM_ERR_LENGTH);
			return;
		}
		PI_Comm_SendStatus(seq);
		break;

	case PI_COMM_CMD_HEARTBEAT:
		if (len != 0)
		{
			PI_Comm_SendNack(seq, cmd, PI_COMM_ERR_LENGTH);
			return;
		}
		PI_Comm_RefreshWatchdog();
		PI_Comm_SendFrame(PI_COMM_CMD_HEARTBEAT_ACK, seq, 0, 0);
		break;

	case PI_COMM_CMD_QUERY_VISION:
		if (len != 0)
		{
			PI_Comm_SendNack(seq, cmd, PI_COMM_ERR_LENGTH);
			return;
		}
		PI_Comm_SendVisionStatus(seq);
		break;

	case PI_COMM_CMD_K210_TEXT:
		if (len == 0 || len > PI_COMM_MAX_PAYLOAD)
		{
			PI_Comm_SendNack(seq, cmd, PI_COMM_ERR_LENGTH);
			return;
		}
		PI_Comm_EnsureK210UartReady();
		USART2_Send_ArrayU8((uint8_t *)payload, len);
		PI_Comm_SendAck(seq, cmd);
		break;

	case PI_COMM_CMD_EMERGENCY_STOP:
		if (len != 0)
		{
			PI_Comm_SendNack(seq, cmd, PI_COMM_ERR_LENGTH);
			return;
		}
		Move_X = 0;
		Move_Z = 0;
		Stop_Flag = 1;
		PI_Comm_RefreshWatchdog();
		PI_Comm_SendAck(seq, cmd);
		break;

	case PI_COMM_CMD_SET_HOST_STATE:
		if (len != 1)
		{
			PI_Comm_SendNack(seq, cmd, PI_COMM_ERR_LENGTH);
			return;
		}
		pi_host_state_flags = payload[0];
		PI_Comm_SendAck(seq, cmd);
		break;

	default:
		PI_Comm_SendNack(seq, cmd, PI_COMM_ERR_ILLEGAL_CMD);
		break;
	}
}

void PI_Comm_Init(u32 baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);

	PI_Comm_ResetParser();
}

void PI_Comm_10ms_Task(void)
{
	if (pi_supervision_active)
	{
		if (pi_timeout_ticks < 0xFFFF)
		{
			pi_timeout_ticks++;
		}

		if (pi_timeout_ticks >= PI_COMM_HEARTBEAT_TIMEOUT && pi_timeout_latched == 0)
		{
			Move_X = 0;
			Move_Z = 0;
			Stop_Flag = 1;
			pi_timeout_latched = 1;
			PI_Comm_SendEvent(PI_COMM_EVENT_TIMEOUT_STOP);
		}
	}

	if (pi_last_power_flag != lower_power_flag)
	{
		pi_last_power_flag = lower_power_flag;
		if (lower_power_flag)
		{
			PI_Comm_SendEvent(PI_COMM_EVENT_LOW_POWER);
		}
		else
		{
			PI_Comm_SendEvent(PI_COMM_EVENT_POWER_RECOVER);
		}
	}

	if (pi_last_stop_flag != Stop_Flag)
	{
		pi_last_stop_flag = Stop_Flag;
		if (Stop_Flag)
		{
			PI_Comm_SendEvent(PI_COMM_EVENT_STOP_ASSERT);
		}
		else
		{
			PI_Comm_SendEvent(PI_COMM_EVENT_STOP_CLEAR);
		}
	}
}

uint8_t PI_Comm_GetHostStateFlags(void)
{
	return pi_host_state_flags;
}

uint8_t PI_Comm_IsSystemReady(void)
{
	return (uint8_t)((pi_host_state_flags & PI_HOST_STATE_SYSTEM_READY) ? 1 : 0);
}

void PI_Comm_SendEventCode(uint8_t event_code)
{
	PI_Comm_SendEvent(event_code);
}

void USART3_IRQHandler(void)
{
	uint8_t rx_data;
	uint8_t checksum;

	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		rx_data = (uint8_t)USART_ReceiveData(USART3);

		switch (pi_rx_state)
		{
		case PI_RX_WAIT_SOF1:
			if (rx_data == PI_COMM_SOF1)
			{
				pi_rx_state = PI_RX_WAIT_SOF2;
			}
			break;

		case PI_RX_WAIT_SOF2:
			if (rx_data == PI_COMM_SOF2)
			{
				pi_rx_state = PI_RX_WAIT_VER;
			}
			else
			{
				PI_Comm_ResetParser();
			}
			break;

		case PI_RX_WAIT_VER:
			if (rx_data == PI_COMM_VERSION)
			{
				pi_rx_state = PI_RX_WAIT_CMD;
			}
			else
			{
				PI_Comm_ResetParser();
			}
			break;

		case PI_RX_WAIT_CMD:
			pi_rx_cmd = rx_data;
			pi_rx_state = PI_RX_WAIT_SEQ;
			break;

		case PI_RX_WAIT_SEQ:
			pi_rx_seq = rx_data;
			pi_rx_state = PI_RX_WAIT_LEN;
			break;

		case PI_RX_WAIT_LEN:
			pi_rx_len = rx_data;
			pi_rx_index = 0;
			if (pi_rx_len > PI_COMM_MAX_PAYLOAD)
			{
				PI_Comm_SendNack(pi_rx_seq, pi_rx_cmd, PI_COMM_ERR_LENGTH);
				PI_Comm_ResetParser();
			}
			else if (pi_rx_len == 0)
			{
				pi_rx_state = PI_RX_WAIT_CHK;
			}
			else
			{
				pi_rx_state = PI_RX_WAIT_PAYLOAD;
			}
			break;

		case PI_RX_WAIT_PAYLOAD:
			pi_rx_payload[pi_rx_index++] = rx_data;
			if (pi_rx_index >= pi_rx_len)
			{
				pi_rx_state = PI_RX_WAIT_CHK;
			}
			break;

		case PI_RX_WAIT_CHK:
			checksum = PI_Comm_Checksum(pi_rx_cmd, pi_rx_seq, pi_rx_len, (const uint8_t *)pi_rx_payload);
			if (checksum == rx_data)
			{
				PI_Comm_HandleFrame(pi_rx_cmd, pi_rx_seq, (const uint8_t *)pi_rx_payload, pi_rx_len);
			}
			else
			{
				PI_Comm_SendNack(pi_rx_seq, pi_rx_cmd, PI_COMM_ERR_CHECKSUM);
			}
			PI_Comm_ResetParser();
			break;

		default:
			PI_Comm_ResetParser();
			break;
		}
	}
}
