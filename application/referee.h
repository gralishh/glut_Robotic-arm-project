#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"
// Referee_Date *GetRefereeDataPoint(void);

enum
{

    // 0x0005为人工智能挑战赛加成数据
    // 0x0105 为飞镖发射口倒计时
    // 0x020A fei飞镖机器人客户端指令数据
    // 0x0209 机器人RFID状态
    // 0x0208 空中机器人的弹丸发射数
    // 0x0104 裁判警告信息
    Competition_Satus_e = 0x0001,  // 比赛状态数据
    Competition_Result_e = 0x0002, // 比赛结果数据
    Robot_Survive_Data_e = 0x0003, // 机器人血量数据
    //	Rart_launch_Satus_e  = 0x0004,//接收飞镖的数据
    Site_Event_Data_e = 0x0101,         // 场地事件数据
    Supply_Station_Data_e = 0x0102,     // 场地补给站数据标识
    Request_Bullet_Data_e = 0x0103,     // 裁判系统没有
    Referee_Warning_Message_t = 0x0104, // 裁判警告信息
    //	Rart_Remaining_Time_t = 0x0105,   //飞镖发射口倒计时
    Robot_Status_Data_e = 0x0201,    // 机器人状态数据
    Power_Heat_Data_e = 0x0202,      // 实时功率热量数据
    Robot_Position = 0x0203,         // 机器人位置数据
    Robot_Gain_Data_e = 0x0204,      // 机器人增益数据
    Air_Robot_Power_Data_e = 0x0205, // 空中机器人能量状态数据
    Hurt_data_e = 0x0206,            // 伤害状态数据
    Shoot_Data_e = 0x0207,           // 实时射击数据
    Bullet_Remaining_t = 0x0208,     // 子弹剩余发射数
    Robot_Rfid_Status_t = 0x0209,    // 机器人 RFID 状态
    //	Dart_Client_Cmd_t      = 0x020A,	// 飞镖机器人客户端指令数据：0x020A
    //  Robot_Interaction      = 0x0301,	//机器人间交互数据

};
typedef enum
{
    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_STANDARD_1 = 3,
    RED_STANDARD_2 = 4,
    RED_STANDARD_3 = 5,
    RED_AERIAL = 6,
    RED_SENTRY = 7,
    BLUE_HERO = 11,
    BLUE_ENGINEER = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL = 16,
    BLUE_SENTRY = 17,
} robot_id_t;
typedef __packed struct
{
    uint8_t sof;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
} frame_header_t;
typedef enum
{
    PROGRESS_UNSTART = 0,
    PROGRESS_PREPARE = 1,
    PROGRESS_SELFCHECK = 2,
    PROGRESS_5sCOUNTDOWN = 3,
    PROGRESS_BATTLE = 4,
    PROGRESS_CALCULATING = 5,
} game_progress_t;
typedef __packed struct // 0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
} ext_game_state_t;

typedef __packed struct // 0002
{
    uint8_t winner;
} ext_game_result_t;
typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;
typedef __packed struct // 0101
{
    uint32_t event_type;
} ext_event_data_t;

typedef __packed struct // 0x0102
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef __packed struct // 0x0103
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef __packed struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;
typedef __packed struct // 0x0201
{

    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_heat0_cooling_rate;  // 17mm机器人 1 号 17mm 枪口每秒冷却值
    uint16_t shooter_heat0_cooling_limit; // 机器人 1 号 17mm 枪口热量上限
    uint16_t chassis_power_limit;         // 机器人功率上线

    uint16_t shooter_heat1_cooling_rate; // 17mm
    uint16_t shooter_heat1_cooling_limit;
    uint16_t shooter_heat1_speed_limit;

    uint16_t shooter_heat2_cooling_rate; // 42mm
    uint16_t shooter_heat2_cooling_limit;
    uint16_t shooter_heat2_speed_limit;

    //    uint16_t chassis_power_limit;   //机器人底盘功率限制上限

    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t; // 无误2023.3.23

typedef __packed struct // 0x0202
{
    uint16_t chassis_volt;         // 底盘输出电压 单位 毫伏
    uint16_t chassis_current;      // 底盘输出电流 单位 毫安
    float chassis_power;           // 底盘输出功率 单位 W 瓦
    uint16_t chassis_power_buffer; // 底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
    uint16_t shooter_heat0;        // 1 号 17mm 枪口热量
    uint16_t shooter_heat1;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t; // 无误2023.3.23

typedef __packed struct // 0x0203
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t; // 无误2023.3.23

typedef __packed struct // 0x0204
{
    uint8_t power_rune_buff;
} ext_buff_musk_t; // 空中机器人//无误2023.3.23

typedef __packed struct // 0x0205
{
    uint8_t energy_point;
    uint8_t attack_time;
} aerial_robot_energy_t;

typedef __packed struct // 0x0206
{
    uint8_t armor_type : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct // 0x0207
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed; // 射术，要添加
} ext_shoot_data_t;
typedef __packed struct
{
    uint8_t bullet_remaining_num; // 剩余子弹（要添加）
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;
typedef __packed struct // 0x0301
{
    uint16_t data_cmd_id; // 子内容 ID
    uint16_t sender_id;   // 发送者 ID
    uint16_t receiver_id; // 接收者 ID
    uint8_t user_data[4]; // 内容数据段，最大为 113
} ext_student_interactive_data_t;

typedef __packed struct // 0x0101
{
    frame_header_t head;
    uint16_t cmd_id;
    ext_student_interactive_data_t cmd_data;
    uint16_t crc16;
} cmd_test_t;
typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data_t;

typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;

typedef __packed struct // 0x0303
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint8_t cmd_source;
} map_command_t;

// UI设计
typedef __packed struct
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
} student_interactive_header_data_t;

typedef __packed struct // 图形
{
    uint8_t figure_name[3];    // 图形名
    uint32_t operate_tpye : 3; // 图形操作
    uint32_t figure_tpye : 3;  // 图形类型
    uint32_t layer : 4;        // 图层数（0~9）
    uint32_t color : 4;        // 颜色
    uint32_t start_angle : 9;  // 圆弧起始角度
    uint32_t end_angle : 9;    // 圆弧终止角度
    uint32_t width : 10;       // 线宽，建议字体大小与线宽比例为 10：1
    uint32_t start_x : 11;     // 起点/圆心 x 坐标
    uint32_t start_y : 11;     // 起点/圆心 y 坐标
    uint32_t radius : 10;      // 半径
    /*
                X                  Y
        直线：终点x坐标         终点y坐标
        矩形：对角顶点x坐标     对角顶点y坐标
        椭圆：x半轴长度         y半轴长度
        圆弧：x半轴长度         y半轴长度
    */
    uint32_t x : 11;
    uint32_t y : 11;
} UI_figure_t;

typedef __packed struct // 字符
{
    uint8_t figure_name[3];    // 图形名
    uint32_t operate_tpye : 3; // 图形操作
    uint32_t figure_tpye : 3;  // 图形类型
    uint32_t layer : 4;        // 图层数（0~9）
    uint32_t color : 4;        // 颜色
    uint32_t size : 9;         // 字体大小
    uint32_t length : 9;       // 字符长度
    uint32_t width : 10;       // 线宽，建议字体大小与线宽比例为 10：1
    uint32_t start_x : 11;     // 起点/圆心 x 坐标
    uint32_t start_y : 11;     // 起点/圆心 y 坐标
    uint32_t details_c : 10;   // 保留
    uint32_t details_d : 11;
    uint32_t details_e : 11;
} UI_char_t;

typedef __packed struct // 整数
{
    uint8_t figure_name[3];    // 图形名
    uint32_t operate_tpye : 3; // 图形操作
    uint32_t figure_tpye : 3;  // 图形类型
    uint32_t layer : 4;        // 图层数（0~9）
    uint32_t color : 4;        // 颜色
    uint32_t size : 9;         // 字体大小
    uint32_t details_b : 9;    // 保留
    uint32_t width : 10;       // 线宽，建议字体大小与线宽比例为 10：1
    uint32_t start_x : 11;     // 起点/圆心 x 坐标
    uint32_t start_y : 11;     // 起点/圆心 y 坐标
    int32_t int_value;         // 值
} UI_int_t;

typedef __packed struct // float
{
    uint8_t figure_name[3];    // 图形名
    uint32_t operate_tpye : 3; // 图形操作
    uint32_t figure_tpye : 3;  // 图形类型
    uint32_t layer : 4;        // 图层数（0~9）
    uint32_t color : 4;        // 颜色
    uint32_t size : 9;         // 字体大小
    uint32_t details_b : 9;    // 保留
    uint32_t width : 10;       // 线宽，建议字体大小与线宽比例为 10：1
    uint32_t start_x : 11;     // 起点/圆心 x 坐标
    uint32_t start_y : 11;     // 起点/圆心 y 坐标
    int32_t float_value_1000;  // float值, 该值除以 1000 即实际显示值
} UI_float_t;

// 删除图形
typedef __packed struct // 0x0100
{
    uint8_t delete_type;
    uint8_t layer;
} interaction_layer_delete_t;

// 画一个图形
typedef __packed struct // 0x0101
{
    frame_header_t head;
    uint16_t cmd_id;
    student_interactive_header_data_t Client_Custom_ID; // 发送ID、接收ID等等
    UI_figure_t UI_figure;
    uint16_t crc16;
} draw_ui_figure_1;
// 画俩个图形
typedef __packed struct // 0x0102
{
    frame_header_t head;
    uint16_t cmd_id;
    student_interactive_header_data_t Client_Custom_ID; // 发送ID、接收ID等等
    UI_figure_t UI_figure[2];
    uint16_t crc16;
} draw_ui_figure_2;
// 画五个图形
typedef __packed struct // 0x0103
{
    frame_header_t head;
    uint16_t cmd_id;
    student_interactive_header_data_t Client_Custom_ID; // 发送ID、接收ID等等
    UI_figure_t UI_figure[5];
    uint16_t crc16;
} draw_ui_figure_5;
// 画七个图形
typedef __packed struct // 0x0104
{
    frame_header_t head;
    uint16_t cmd_id;
    student_interactive_header_data_t Client_Custom_ID; // 发送ID、接收ID等等
    UI_figure_t UI_figure[7];
    uint16_t crc16;
} draw_ui_figure_7;

// 画字符
typedef __packed struct // 0x0110
{
    frame_header_t head;
    uint16_t cmd_id;
    student_interactive_header_data_t Client_Custom_ID; // 发送ID、接收ID等等
    UI_char_t UI_char;
    uint8_t data[30];
    uint16_t crc16;
} draw_ui_char;

//  画一个int
typedef __packed struct // 0x0101
{
    frame_header_t head;
    uint16_t cmd_id;
    student_interactive_header_data_t Client_Custom_ID; // 发送ID、接收ID等等
    UI_int_t UI_int;
    uint16_t crc16;
} draw_ui_int_1;
//  画两个int
typedef __packed struct // 0x0102
{
    frame_header_t head;
    uint16_t cmd_id;
    student_interactive_header_data_t Client_Custom_ID; // 发送ID、接收ID等等
    UI_int_t UI_int[2];
    uint16_t crc16;
} draw_ui_int_2;
//  画5个int
typedef __packed struct // 0x0103
{
    frame_header_t head;
    uint16_t cmd_id;
    student_interactive_header_data_t Client_Custom_ID; // 发送ID、接收ID等等
    UI_int_t UI_int[5];
    uint16_t crc16;
} draw_ui_int_5;
//  画7个int
typedef __packed struct // 0x0104
{
    frame_header_t head;
    uint16_t cmd_id;
    student_interactive_header_data_t Client_Custom_ID; // 发送ID、接收ID等等
    UI_int_t UI_int[7];
    uint16_t crc16;
} draw_ui_int_7;

// 画1个float
typedef __packed struct // 0x0101
{
    frame_header_t head;
    uint16_t cmd_id;
    student_interactive_header_data_t Client_Custom_ID; // 发送ID、接收ID等等
    UI_float_t UI_float;
    uint16_t crc16;
} draw_ui_float_1;
// 画2个float
typedef __packed struct // 0x0102
{
    frame_header_t head;
    uint16_t cmd_id;
    student_interactive_header_data_t Client_Custom_ID; // 发送ID、接收ID等等
    UI_float_t UI_float[2];
    uint16_t crc16;
} draw_ui_float_2;
// 画5个float
typedef __packed struct // 0x0103
{
    frame_header_t head;
    uint16_t cmd_id;
    student_interactive_header_data_t Client_Custom_ID; // 发送ID、接收ID等等
    UI_float_t UI_float[5];
    uint16_t crc16;
} draw_ui_float_5;
// 画7个float
typedef __packed struct // 0x0104
{
    frame_header_t head;
    uint16_t cmd_id;
    student_interactive_header_data_t Client_Custom_ID; // 发送ID、接收ID等等
    UI_float_t UI_float[7];
    uint16_t crc16;
} draw_ui_float_7;

extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);
extern uint16_t Get_chassis_power_limit(void);
extern uint8_t GetEenmyColor(void);
extern int16_t Get_shooter_42mm_speed_limit(void);
extern uint8_t GetRobotID(void);
extern uint8_t GetHp(void);
extern uint8_t GetMatchReady(void);
extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);
extern float GetPowerBuffer(void); // 得到底盘实时缓存功率
extern float GetRealPower(void);   // 得到底盘实时功率
extern float GetPowerHeat(void);   // 得到枪口热度
extern int16_t GetBullet(void);    // 获取发射弹量
extern float bullet_speed(void);
#endif
