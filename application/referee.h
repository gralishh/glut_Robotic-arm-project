#ifndef __REFEREE__H
#define __REFEREE__H

#include "stdio.h"
#include "main.h"
#define AHRS_RC_LEN  50 	//定义接收字节?? 512

enum{
	
	
		//0x0005为人工智能挑战赛加成数据
	//0x0105 为???镖发射口倒???时
	//0x020A fei飞镖机器人???户??指令数据
	//0x0209 机器人RFID状??
	//0x0208 空中机器人的弹丸发射??
	//0x0104 裁判警告信息
	Competition_Satus_e  = 0x0001,//比赛状态数??
	Competition_Result_e = 0x0002,//比赛结果数据
	Robot_Survive_Data_e = 0x0003,//机器人???量数??
	Rart_launch_Satus_e  = 0x0004,//接收飞镖的数??
	Site_Event_Data_e    = 0x0101,//场地事件数据
	Supply_Station_Data_e  = 0x0102,//场地补给站数??标识
	Request_Bullet_Data_e  = 0x0103,  //裁判系统没有
	Referee_Warning_Message_t = 0x0104,  //裁判警告信息
	Rart_Remaining_Time_t = 0x0105,   //飞镖发射口倒???时
	Robot_Status_Data_e    = 0x0201,//机器人状态数??
	Power_Heat_Data_e      = 0x0202,//实时功率??量数??
	Robot_Position         = 0x0203,//机器人位??数据
	Robot_Gain_Data_e      = 0x0204,//机器人???益数据
  Air_Robot_Power_Data_e = 0x0205,//空中机器人能量状态数??
  Hurt_data_e            = 0x0206,//伤???状态数??
  Shoot_Data_e           = 0x0207,//实时射击数据
  Bullet_Remaining_t     = 0x0208,// 子弹剩余发射??
	Robot_Rfid_Status_t    = 0x0209,//机器?? RFID 状??
	Dart_Client_Cmd_t      = 0x020A,	// 飞镖机器人???户??指令数据??0x020A
  Robot_Interaction      = 0x0301,	//机器人间交互数据
  	Self_Controler	 = 0x0302,	//??定义控制??
	Robot_Command_t				 = 0x0304,  // 图传遥控信息
	Custom_Client_Data_e	=		0x0306,	//??定义控制器模拟键鼠操作选手??

};
//2020??
typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;

typedef __packed struct
{
uint8_t game_type : 4;
uint8_t game_progress : 4;
uint16_t stage_remain_time;
}ext_game_state_t;

typedef __packed struct
{
uint8_t winner;
} ext_game_result_t;

//机器人???量数????0x0003。发送???率??1Hz，发送范围：所有机器人??
typedef __packed struct
{
 uint16_t red_1_robot_HP;
 uint16_t red_2_robot_HP; 
 uint16_t red_3_robot_HP; 
 uint16_t red_4_robot_HP; 
 uint16_t red_5_robot_HP; 
 uint16_t red_7_robot_HP; 
 uint16_t red_outpost_HP;
 uint16_t red_base_HP;
	
 uint16_t blue_1_robot_HP; 
 uint16_t blue_2_robot_HP; 
 uint16_t blue_3_robot_HP; 
 uint16_t blue_4_robot_HP; 
 uint16_t blue_5_robot_HP; 
 uint16_t blue_7_robot_HP; 
 uint16_t blue_outpost_HP;
 uint16_t blue_base_HP;
	
} ext_game_robot_HP_t;



//飞镖发射状态：0x0004。发送???率：???镖发射后发送，发送范围：所有机器人
typedef __packed struct
{
 uint8_t dart_belong; 
 uint16_t stage_remaining_time; 
} ext_dart_status_t;

//人工智能挑战赛加成与惩罚区状态：0x0005。发送???率??1Hz 周期发送，发送范围：所有机器人
typedef __packed struct
{
 uint8_t F1_zone_status:1;
 uint8_t F1_zone_buff_debuff_status:3;
 uint8_t F2_zone_status:1;
 uint8_t F2_zone_buff_debuff_status:3; 
 uint8_t F3_zone_status:1;
 uint8_t F3_zone_buff_debuff_status:3; 
 uint8_t F4_zone_status:1;
 uint8_t F4_zone_buff_debuff_status:3; 
 uint8_t F5_zone_status:1;
 uint8_t F5_zone_buff_debuff_status:3; 
 uint8_t F6_zone_status:1;
 uint8_t F6_zone_buff_debuff_status:3;
	
 uint16_t red1_bullet_left;
 uint16_t red2_bullet_left;
 uint16_t blue1_bullet_left;
 uint16_t blue2_bullet_left;
	
} ext_ICRA_buff_debuff_zone_status_t;

typedef __packed struct
{
 uint32_t event_type;
} ext_event_data_t;

typedef __packed struct
{
 uint8_t supply_projectile_id; 
 uint8_t supply_robot_id; 
 uint8_t supply_projectile_step; 
uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef __packed struct
{
 uint8_t level;
 uint8_t foul_robot_id; 
} ext_referee_warning_t;

typedef __packed struct
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

typedef __packed struct
{
uint8_t robot_id;
uint8_t robot_level;
uint16_t remain_HP;
uint16_t max_HP;
uint16_t shooter_id1_17mm_cooling_rate;
uint16_t shooter_id1_17mm_cooling_limit;
uint16_t shooter_id1_17mm_speed_limit;	 //需要除??256
uint16_t shooter_id2_17mm_cooling_rate;
uint16_t shooter_id2_17mm_cooling_limit;
uint16_t shooter_id2_17mm_speed_limit;	
uint16_t shooter_id1_42mm_cooling_rate;
uint16_t shooter_id1_42mm_cooling_limit;  
uint16_t shooter_id1_42mm_speed_limit;    //需要除??256 
uint16_t chassis_power_limit;            //需要除??256   
uint8_t mains_power_gimbal_output : 1;
uint8_t mains_power_chassis_output : 1;
uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

typedef __packed struct
{
uint16_t chassis_volt;
uint16_t chassis_current;
float chassis_power;
uint16_t chassis_power_buffer;
uint16_t shooter_id1_17mm_cooling_heat;
uint16_t shooter_id2_17mm_cooling_heat;
uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef __packed struct
{
 float x;
 float y;
 float z;
 float yaw;
} ext_game_robot_pos_t;


///////////////////////////



typedef __packed struct
{
 uint16_t energy_point;
 uint8_t attack_time;
} ext_aerial_robot_energy_t;


typedef __packed struct
{
uint8_t power_rune_buff;
}ext_buff_t;

typedef __packed struct
{
uint8_t attack_time;
} aerial_robot_energy_t;


//////////////////////////////

typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t shooter_id;
 uint8_t bullet_freq;
 float bullet_speed;
} ext_shoot_data_t;

typedef __packed struct
{
uint16_t bullet_remaining_num_17mm;
uint16_t bullet_remaining_num_42mm;
uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

typedef __packed struct
{
 uint32_t rfid_status;
} ext_rfid_status_t;

typedef __packed struct
{
 uint8_t dart_launch_opening_status;
 uint8_t dart_attack_target;
 uint16_t target_change_time;
 uint8_t first_dart_speed;
 uint8_t second_dart_speed;
 uint8_t third_dart_speed;
 uint8_t fourth_dart_speed;
 uint16_t last_dart_launch_time;
 uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;



//??定义UI




typedef __packed struct
{

	uint8_t operate_tpye;
	uint8_t graphic_tpye;
	uint8_t graphic_name[5];
	uint8_t layer;
	uint8_t color;
	uint8_t width;
	uint16_t start_x;
	uint16_t start_y;
	uint16_t radius;
	uint16_t end_x;
	uint16_t end_y;
	int16_t start_angle;
	int16_t end_angle;
	uint8_t text_lenght;
	uint8_t text[30];



}graphic_data;


//交互发接信息
typedef __packed struct
{
uint16_t data_cmd_id;
uint16_t sender_ID;
uint16_t receiver_ID;	
}ext_student_interactive_header_data_t;

//删除图形
typedef __packed struct
{
uint8_t operate_tpye; 
uint8_t layer; 
} ext_client_custom_graphic_delete_t;

typedef __packed struct
{ 
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4; 
	uint32_t color:4; 
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10; 
	uint32_t start_x:11; 
	uint32_t start_y:11; 
	uint32_t radius:10; 
	uint32_t end_x:11; 
	uint32_t end_y:11; 
} graphic_data_struct_t;
//
//画一??图形
typedef __packed struct
{
 graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

//画两??图形
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[2];
	
} ext_client_custom_graphic_double_t;
//画五??图形
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;
//画七??图形
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

//绘制字???
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct;
char  data[30];
} ext_client_custom_character_t;
//定义UI(画图)
typedef __packed struct
{
	frame_header_t head;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  //发送ID、接收ID等等
	graphic_data_struct_t grapic_data_struct[7];
	uint16_t crc16;
}ext_draw_ui;

//??定义UI(画字??)
typedef __packed struct
{
  frame_header_t head;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  //发送ID、接收ID等等
	graphic_data_struct_t grapic_data_struct;
	uint8_t   data[30];
	uint16_t crc16;
}ext_draw_ui_character;



typedef __packed struct
{
uint16_t robot_legion;
} ext_game_robot_survivors_t;




typedef __packed struct
{
uint8_t supply_projectile_id;
uint8_t supply_robot_id;
uint8_t supply_num;
} ext_supply_projectile_booking_t;



/*********************self_control**********************/
typedef __packed struct
{
	uint8_t header;
	uint8_t motor_position1[4];
	uint8_t motor_position2[4];
	uint8_t motor_position3[4];
	uint8_t motor_position4[4];
	uint8_t motor_position5[4];
	uint8_t motor_position6[4];
	uint8_t key;
	uint8_t kong1[3]; 
	uint8_t tailer;
}elf_control_t;
typedef __packed struct
{
elf_control_t self_control_point;
} ext_self_control_t;

typedef __packed struct
{
 uint16_t key_value;
 uint16_t x_position:12;
 uint16_t mouse_left:4;
 uint16_t y_position:12;
 uint16_t mouse_right:4;
 uint16_t reserved;
}custom_client_data_t;
typedef __packed struct
{
	frame_header_t head;
	uint16_t cmd_id;
	custom_client_data_t data_struct;
	uint16_t crc16;
}ext_custom_data_all_ui;
/*********************self_control**********************/

/*********************说明**********************
Referee.c已经舍弃，用usart6.c代替
其它.c还有在引用Referee.h的结构体
*********************说明**********************/

typedef struct __Referee_Date__
{		
	frame_header_t    frame_header;
  int16_t CmdID;	  
	ext_game_state_t  game_state;
	ext_game_result_t game_result;
	ext_game_robot_HP_t   game_robot_survivors;
//	ext_dart_status_t
//	ext_ICRA_buff_debuff_zone_status_t
//  ext_aerial_robot_energy_t;
	
	ext_event_data_t      event_data;
	ext_supply_projectile_action_t   supply_projectile_action;
	ext_referee_warning_t            referee_warning_message;
	ext_supply_projectile_booking_t  supply_projectile_booking;
	ext_dart_remaining_time_t        dart_remaining_time;
	ext_rfid_status_t                rfid_status;
	ext_dart_client_cmd_t            dart_client_cmd;
	
	
	ext_game_robot_status_t  game_robot_state;
	ext_power_heat_data_t   power_heat_data;
	ext_game_robot_pos_t    game_robot_pos;
	ext_dart_status_t       dart_status;
	ext_buff_t  buff_musk;
	aerial_robot_energy_t aerial_robot_energy;
	ext_robot_hurt_t   robot_hurt;
	ext_shoot_data_t   shoot_data;
	ext_bullet_remaining_t   bullet_remaining;
	ext_student_interactive_header_data_t student_interactive_header_data;
//	ext_client_custom_graphic_delete_t
//  graphic_data_struct_t
	ext_self_control_t		self_control;
}Referee_Date;



typedef struct _power
{
	volatile  int16_t SetPower;
	float curr_power;
	int16_t Limit;
	float Kp;	
	float Ki;
	float Kd;
  volatile  float average ;	
	volatile  int16_t output;
	float pout;
	float iout;
	float dout;
	float error[3];
}Power;

typedef __packed struct _Client
{
	frame_header_t head;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t student_interactive_header_data;
	float customize_data1;
	float customize_data2;
	float customize_data3;
	uint8_t Indicator_light : 6;
	uint8_t reverse:2;
	uint16_t crc16;
}clientData;




extern  Referee_Date Referee_date1;
extern  uint16_t Usart1_RX_Cou  ;
extern uint8_t Usart1_RX_Flag  ;
extern char  g_Referee_flag ;
extern Power power ; 

//void Referee_init(void);

#endif
