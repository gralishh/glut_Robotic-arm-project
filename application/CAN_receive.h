#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "pid.h"
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/*C610¡¢C620*/
#define SEND_ID201_204 0x200
#define SEND_ID205_208 0X1FF
/*6020*/
#define _6020_SEND_ID201_204 s
/*5-7*/
#define _6020_SEND_ID205_208 0X2FF

#define M2006_REDUCTIONRTION 36
#define M3510_REDUCTIONRTION  27

/*C610¡¢C620*/
#define SEND_ID201_204 0x200
#define SEND_ID205_208 0X1FF
/*6020*/
// #define _6020_SEND_ID201_204 0X1FF
// /*5-7*/
// #define _6020_SEND_ID205_208 0X2FF

#define M2006_REDUCTIONRTION 36
#define M3510_REDUCTIONRTION  27

/*??????*/
#define hcan1 hfdcan1
#define hcan2 hfdcan2
#define hcan3 hfdcan3

typedef enum
{
	CAN_2006_roll_1_ID  				  = 0x201,
	CAN_2006_HAND_pitch_2_ID 			= 0x202,  
 	CAN_2006_HAND_x_4_ID          = 0x203,  	
	CAN_encoder_HAND_x_4_ID       = 0x02,
  CAN_HAND_INS                  = 0x88,
    	

  CAN_3508_M1_ID                = 0x201,
  CAN_3508_M2_ID                = 0x202,
  CAN_3508_M3_ID                = 0x203,
  CAN_3508_M4_ID                = 0x204,


} can_msg_id_e;

typedef enum
{
	CAN_EXTER_ENCODER_HEIGHT_L 		= 0x02,
	CAN_EXTER_ENCODER_HEIGHT_R		= 0x04,
	CAN_EXTER_ENCODER_LEFTRIGHT   = 0x06,

  CAN_3508_RISE1_height_L_ID    = 0x201,
  CAN_3508_RISE1_height_R_ID    = 0x202,
	CAN_3508_LeftRight_ID         = 0x203,
	
  CAN_CHASSIS_CONTROLLER        = 0x10,
  CAN_GIMBAL_CONTROLLER         = 0x08,

}can2_msg_id_e;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
	  int64_t crc;
} motor_measure_t;

typedef struct
{
	  
    uint8_t data_lenth;
    uint8_t encoder_ID;
    uint8_t func_cmd;
	  uint8_t crc;
	  uint32_t last_ecd ;
    uint32_t ecd;
} external_encoder_t;

const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
/**************************************************************/
const motor_measure_t *get_3508_RISE1_height_L_Measure_Point(void);
const motor_measure_t *get_3508_RISE2_height_R_Measure_Point(void);
const motor_measure_t *get_3508_LeftRight_Measure_Point(void);
/**************************************************************/
const motor_measure_t *get__roll_1_Measure_Point(void);
const motor_measure_t *get_2006_HAND_pitch_2_Measure_Point(void);
const motor_measure_t *get_2006_HAND_roll_3_Measure_Point(void);
const motor_measure_t *get_2006_HAND_x_4_Measure_Point(void);
const external_encoder_t *get_encoder_HAND_x_4_Measure_Point(void);

const motor_measure_t *get_2006_HAND_yaw_5_Measure_Point(void);
/*************************************************************/
const motor_measure_t *get_2006_VIEW_pitch_Measure_Point(void);
const motor_measure_t *get_2006_HOOK_pitch_L_Measure_Point(void);
const motor_measure_t *get_2006_HOOK_pitch_R_Measure_Point(void);
/**************************************************************/
const external_encoder_t *get_gimbal_encoder_height_L_Point(void);
const external_encoder_t *get_gimbal_encoder_height_R_Point(void);
const external_encoder_t *get_gimbal_encoder_leftRight_Point(void);
void CAN_RX_hook(FDCAN_HandleTypeDef* CANx, FDCAN_RxHeaderTypeDef* rx_header,uint8_t* rx_message); 
void CanSendMess(FDCAN_HandleTypeDef* CANx,uint32_t SendID,int16_t *message);
void CanSendMoreMess(FDCAN_HandleTypeDef* CANx, uint32_t SendID, uint8_t *message, uint8_t messageLength);

#endif
