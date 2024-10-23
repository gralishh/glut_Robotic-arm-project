#ifndef MCU_COMMUNICATION_TASK_H
#define MCU_COMMUNICATION_TASK_H
#include "main.h"
#include "referee.h"
#include "PC_communication_task.h"
#define ChassisSendime  1


typedef __packed struct
{
	uint8_t header;
	float motor_position1;
	float last_position1;
	
	float motor_position2;
	float last_position2;

	float motor_position3;
	float last_position3;

	float motor_position4;
	float last_position4;


	float motor_position5;
	float last_position5;


	float motor_position6;
	float last_position6;

	uint8_t key1;
	uint8_t key2;
	uint8_t key3;
	uint8_t tailer;
}elf_measure_t;//自定义控制器原始数据解算后发送云台

typedef __packed struct
{
	float motor_position1;
	float motor_position2;

}gimbal_status_t;//云台需要数据

typedef __packed struct
{
	float motor_position3;
	float motor_position4;
	float motor_position5;
	float motor_position6;
	float motor_position7;

}hand_status_t;//手臂需要数据

typedef __packed struct
{
	uint8_t header;
	uint8_t gimbal_status[8];
	uint8_t hand_status[20];
	uint8_t kong[10];//画UI需要状态 ，提前留空
	uint8_t tailer;
}gimbal_to_chassis_t; //获得云台需要数据发送给底盘 UI和视觉


typedef __packed struct
{
	uint8_t kong[10];
}UI_need_data_t; //获得UI需要数据

typedef __packed struct
{
	uint8_t motor_position1[4];
	uint8_t motor_position2[4];
	uint8_t motor_position3[4];
	uint8_t motor_position4[4];
	uint8_t motor_position5[4];
	uint8_t motor_position6[4];
	uint8_t motor_position7[4];

}visuals_need_data_t; //获得视觉需要数据


void chassis_send_task(void const *parmas);
void gimbal_receive_task(void const *parmas);
void gimbal_send_task(void const *parmas);

const UI_need_data_t *get_UI_need_data_point(void);
const visuals_need_data_t *get_Visual_need_data_point(void);
void self_control_measure(const ext_self_control_t self_control , elf_measure_t*self_measure  );
static uint16_t merge_uint8_to_uint16(uint8_t a, uint8_t b) ;
static void split_uint8_to_uint8(uint8_t input, uint8_t *output1, uint8_t *output2, uint8_t *output3);
static void gimbal_data_allocate_to_UI_or_Visual(const gimbal_to_chassis_t chassis_rx , UI_need_data_t* UI_need , visuals_need_data_t* Visual_need );
void Transmission_Visual_measure_visuals_rx(const visuals_rx_t Transmission_Visual , visuals_rx_data_t* visuals_rx);
const  visuals_rx_data_t *get_visuals_rx_data_point(void);
const elf_measure_t *get_self_measure_point(void);
void chassis_receive_task(void const *parmas);

#endif 

