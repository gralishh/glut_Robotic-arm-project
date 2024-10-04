#ifndef CHASSISTASK_H
#define CHASSISTASK_H
#include "main.h"
#include "remote_control.h"
#include "pid.h"
#include "user_lib.h"
#include "CAN_receive.h"
//任务开始空闲一段时??
#define CHASSIS_TASK_INIT_TIME 357

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 3
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 2
//在特殊模式下，可以通过遥控器控制旋??
#define CHASSIS_WZ_CHANNEL 4
/**************************************************************************/
//选择底盘状?? 开关通道??
#define MODE_CHANNEL 1
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.0053f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.0035f
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）???加到车体???度的比??
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//不跟随云台的时?? 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比??
#define CHASSIS_WZ_RC_SEN 0.01f
#define CHASSIS_WZ_MOUSE_SEN 0.1f

//底盘??向和纵向的加速度滤波器阶??
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.1666666667f
/*0.3333333333f*/

#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002
//底盘任务控制频率，尚??使用这个??
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//底盘3508最??can发送电流??
#define MAX_MOTOR_CAN_CURRENT 5000.0f

//底盘360旋转按键
#define REVOLVE_START_KEY  KEY_PRESSED_OFFSET_SHIFT
#define REVOLVE_STOP_KEY    KEY_PRESSED_OFFSET_CTRL

//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//m3508??化成底盘速度(m/s)的比例，做两???? ??因为??能换电机需要更换比??
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

#define CHASSIS_WZ_SET_SCALE 0.03f

//底盘四个??子分开PID调试
#define MOTOR_1_SPEED_PID_KP  16000.0f//16000
#define MOTOR_1_SPEED_PID_KI  0.0f
#define MOTOR_1_SPEED_PID_KD  0.0f
#define MOTOR_1_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define MOTOR_1_SPEED_PID_MAX_IOUT 2000.0f

#define MOTOR_2_SPEED_PID_KP 16000.0f//16000
#define MOTOR_2_SPEED_PID_KI 0.0f
#define MOTOR_2_SPEED_PID_KD 0.0f
#define MOTOR_2_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define MOTOR_2_SPEED_PID_MAX_IOUT 2000.0f

#define MOTOR_3_SPEED_PID_KP 16000.0f//16000
#define MOTOR_3_SPEED_PID_KI 0.0f
#define MOTOR_3_SPEED_PID_KD 0.0f
#define MOTOR_3_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define MOTOR_3_SPEED_PID_MAX_IOUT 2000.0f

#define MOTOR_4_SPEED_PID_KP 16000.0f//16000
#define MOTOR_4_SPEED_PID_KI 0.0f
#define MOTOR_4_SPEED_PID_KD 0.0f
#define MOTOR_4_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define MOTOR_4_SPEED_PID_MAX_IOUT 2000.0f

//具体而言，CHASSIS_VECTOR_RAW 控制模式使用的是底盘的初始坐标系，通过向机器人发送满足???坐标系下约束的线性加速度和???加速度指令来控制底盘向指定方向运动。这种模式的好???是??以实现更灵活的控制，能???在某些场景下做到更精确地控制底盘???走轨迹??

//而???于 CHASSIS_VECTOR_NO_FOLLOW_YAW 模式，底盘的控制仅沿着当前方向平移，不考虑底盘??否朝向前进方向。这种模式下机器人将??进???直线运??，且操作相???简单，适用于一些较为简单的场景??
typedef enum
{
  CHASSIS_VECTOR_NO_FOLLOW_YAW,
  CHASSIS_VECTOR_RAW,
} chassis_mode_e;

//chassis_motor_measure：电机???数器测量值指针；
//accel：电机加速度??
//speed：电机当前速度??
//speed_set：电机目标速度??
//give_current：电机输出给定电??
typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} Chassis_Motor_t;

typedef struct
{
  const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针
  chassis_mode_e chassis_mode;               //底盘控制状态机
  chassis_mode_e last_chassis_mode;          //底盘上???控制状态机
  Chassis_Motor_t motor_chassis[4];          //底盘电机数据
  PidTypeDef motor_speed_pid[4];             //底盘电机速度pid     
  PidTypeDef chassis_angle_pid;              //底盘跟随角度pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;
  first_order_filter_type_t chassis_cmd_slow_set_vy;

  fp32 vx;                         //底盘速度 前进方向 前为正，单位 m/s
  fp32 vy;                         //底盘速度 左右方向 左为??  单位 m/s
  fp32 wz;                         //底盘旋转角速度，逆时针为?? 单位 rad/s
  fp32 vx_set;                     //底盘设定速度 前进方向 前为正，单位 m/s
  fp32 vy_set;                     //底盘设定速度 左右方向 左为正，单位 m/s
  fp32 wz_set;                     //底盘设定旋转角速度，逆时针为?? 单位 rad/s

  fp32 vx_front_speed;  //前进方向最大速度 单位m/s
  fp32 vx_back_speed;  //后退方向最小速度 单位m/s
  fp32 vy_left_speed;  //左方向最大速度 单位m/s
  fp32 vy_right_speed;  //右方向最小速度 单位m/s
} chassis_move_t;

void chassis_task(void const *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);
extern float  GetChassisMaxOutput(void);
extern Chassis_Motor_t *getChassisGive_current(void );
#endif
