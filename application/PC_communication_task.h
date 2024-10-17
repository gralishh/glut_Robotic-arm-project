#ifndef PC_COMMUNICATION_TASK_H
#define PC_COMMUNICATION_TASK_H
#include "main.h"
#include "referee.h"
typedef __packed struct
{
	uint8_t identify_target_flag;
	uint8_t position1[4];
	uint8_t position2[4];
	uint8_t position3[4];
	uint8_t position4[4];
	uint8_t position5[4];
	uint8_t position6[4];
	uint8_t position7[4];
	uint8_t kong[11];

}visuals_tx_data_t; //视觉发送数据数据结构体

//帧头 0xaa  数据data30  crc16校验两位(只校验数据) 帧尾 0x70
typedef __packed struct
{
	uint8_t header;
	visuals_tx_data_t data;
	uint16_t CRC_16;
	uint8_t tailer;
}visuals_tx_t; //视觉发送结构体

typedef __packed struct
{
	uint8_t header;
	uint8_t vx[4];
	uint8_t vy[4];
	uint8_t position1[4];
	uint8_t position2[4];
	uint8_t position3[4];
	uint8_t position4[4];
	uint8_t position5[4];
	uint8_t position6[4];
	uint8_t position7[4];
	uint8_t kong[4];
	uint16_t crc_16;
	uint8_t tailer;

}visuals_rx_t; //视觉接收原始数据数据结构体

typedef __packed struct
{
	float vx;
	float vy;
	float position1;
	float position2;
	float position3;
	float position4;
	float position5;
	float position6;
	float position7;

}visuals_rx_data_t; //视觉接收原始数据数据结构体
const visuals_rx_t *get_visuals_rx_point(void);
const visuals_rx_data_t *get_visuals_rx_data_classis_point(void);
void visual_receive_task(void const *pvParameters);
void visual_send_task(void const *parmas);

#endif


