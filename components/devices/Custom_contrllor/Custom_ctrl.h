#ifndef __CUSTOM_CTRL_H__
#define __CUSTOM_CTRL_H__

#include "main.h" 
#include "struct_typedef.h"

typedef __packed struct
{
	uint8_t data[30];
} robot_interactive_data_t;

typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;

typedef __packed struct
{
  uint8_t  up : 1;
  uint8_t  down : 1;
  uint8_t  left : 1;
	uint8_t  right : 1;
	uint8_t  k1 : 1;
	uint8_t  k2 : 1;
	uint8_t  k3 : 1;
	uint8_t  k4 : 1;
	uint8_t  k5 : 1;
	uint8_t  k6 : 1;
	uint8_t  k7 : 1;
	uint8_t  k8 : 1;
	uint8_t  k9 : 1;
	uint8_t  k10 : 1;
	uint8_t  k11 : 1;
	uint8_t  k12 : 1;
} key_t;

typedef __packed struct
{
	frame_header_t	header;
	uint16_t cmd_id;
	key_t   key;
	float adc_val[5];
	uint16_t crc16;
} data_t; 

typedef data_t CUSTOM_CTRL_RX_PACK;

typedef struct{
  CUSTOM_CTRL_RX_PACK rx_pack;
  fp32 joint_angle[5];
} CUSTOM_CTRL_T;

CUSTOM_CTRL_T* Custom_Ctrl_get_ptr(void);
data_t* Custom_Ctrl_get_rx_pack_ptr(void);
void Custom_Ctrl_unpack(void);
void Custom_Ctrl_Task(void* para);

#endif
