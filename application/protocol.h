#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "struct_typedef.h"

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
    GAME_STATE_CMD_ID                 = 0x0001, //比赛状态数据
    GAME_RESULT_CMD_ID                = 0x0002, //比赛结果数据
    GAME_ROBOT_HP_CMD_ID              = 0x0003, //机器人血量数据
    Rart_launch_Satus_e               = 0x0004, //接收飞镖的数据
    FIELD_EVENTS_CMD_ID               = 0x0101, //场地事件数据
    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102, //场地补给站数据标识
    SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103, //裁判系统没有
    REFEREE_WARNING_CMD_ID            = 0x0104, //裁判警告信息
  	Rart_Remaining_Time_t             = 0x0105, //飞镖发射口倒计时
    ROBOT_STATE_CMD_ID                = 0x0201, //机器人状态数据
    POWER_HEAT_DATA_CMD_ID            = 0x0202, //实时功率热量数据
    ROBOT_POS_CMD_ID                  = 0x0203, //机器人位置数据
    BUFF_MUSK_CMD_ID                  = 0x0204, //机器人增益数据
    AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205, //空中机器人能量状态数据
    ROBOT_HURT_CMD_ID                 = 0x0206, //伤害状态数据
    SHOOT_DATA_CMD_ID                 = 0x0207, //实时射击数据
    BULLET_REMAINING_CMD_ID           = 0x0208, //子弹剩余发射数
    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301, //机器人RFID状态
    Dart_Client_Cmd_t                 = 0x020A,	//飞镖机器人客户端指令数据:0x020A
    Robot_Interaction                 = 0x0301,	//机器人间交互数据
    IDCustomData,
}referee_cmd_id_t;
//enum{
//	
//	
//		//0x0005为人工智能挑战赛加成数据
//	//0x0105 为飞镖发射口倒计时
//	//0x020A fei飞镖机器人客户端指令数据
//	//0x0209 机器人RFID状态
//	//0x0208 空中机器人的弹丸发射数
//	//0x0104 裁判警告信息
//	Competition_Satus_e  = 0x0001,//比赛状态数据
//	Competition_Result_e = 0x0002,//比赛结果数据
//	Robot_Survive_Data_e = 0x0003,//机器人血量数据
//	Rart_launch_Satus_e  = 0x0004,//接收飞镖的数据
//	Site_Event_Data_e    = 0x0101,//场地事件数据
//	Supply_Station_Data_e  = 0x0102,//场地补给站数据标识
//	Request_Bullet_Data_e  = 0x0103,  //裁判系统没有
//	Referee_Warning_Message_t = 0x0104,  //裁判警告信息
//	Rart_Remaining_Time_t = 0x0105,   //飞镖发射口倒计时
//	Robot_Status_Data_e    = 0x0201,//机器人状态数据
//	Power_Heat_Data_e      = 0x0202,//实时功率热量数据
//	Robot_Position         = 0x0203,//机器人位置数据
//	Robot_Gain_Data_e      = 0x0204,//机器人增益数据
//  Air_Robot_Power_Data_e = 0x0205,//空中机器人能量状态数据
//  Hurt_data_e            = 0x0206,//伤害状态数据
//  Shoot_Data_e           = 0x0207,//实时射击数据
//  Bullet_Remaining_t     = 0x0208,// 子弹剩余发射数
//	Robot_Rfid_Status_t    = 0x0209,//机器人 RFID 状态
//	Dart_Client_Cmd_t      = 0x020A,	// 飞镖机器人客户端指令数据：0x020A
//  Robot_Interaction      = 0x0301,	//机器人间交互数据

//};
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
