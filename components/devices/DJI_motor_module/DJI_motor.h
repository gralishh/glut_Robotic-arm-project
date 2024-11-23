/**
 * @brief DJI电机控制
 * @date 24/11/9
 * @note DJI_Motor_Bus_t仅针对CAN端口
 * 上层代码仅对电机句柄操作
 * @todo 添加电机输出转轴位置环控制
 */
#ifndef __DJI_MOTOR_MODULE__
#define __DJI_MOTOR_MODULE__

#include "main.h"
#include "DJI_motor_canbus.h"
#include "struct_typedef.h"
#include "pid.h"

typedef enum{
  M2006=0,
  M3508,
  ANY_MOTOR,
} Motor_Type_e;

// 电机控制模式,由电机控制函数设置
typedef enum{
  NON_FORCE=0,
  SPEED_LOOP,
  POS_LOOP,
  GIVING_CURRENT,
  LOCK
} Motor_Ctrl_mode_e;

// 表示电机初始化状态，
//限制条件不足时电机的控制
typedef enum{
  MOTOR_SPEED_PID_INIT  =0x01<<0,
  MOTOR_POS_PID_INIT    =0x01<<1,
  EXTERN_ENCODER_INIT   =0x01<<2,
} Motor_Ctrl_init_state_e;

typedef struct __DJI_Motor_Bus_t DJI_Motor_Bus_t;

/*反馈报文*/
#pragma pack(1)
typedef struct{
  uint16_t ecd;
  int16_t speed_rpm;
  uint16_t torque;
  uint8_t temp;
  uint8_t REMAIN;
} dji_recv_pack;
#pragma pack()

typedef struct{

} DJI_Motor_Config_t;

typedef struct __DJI_Motor_Ctrl_t{
/*电机结构体*/
  DJI_Motor_Bus_t* mounted_bus;
  PidTypeDef pid_speed_loop;
  PidTypeDef pid_pos_loop;
  dji_recv_pack recv_pack;

/*电机属性*/
  Motor_Type_e type;
  uint16_t id;

/*标志*/
  Motor_Ctrl_mode_e mode;
  Motor_Ctrl_init_state_e init_state;
  //电机反转标志,会使发送的电流置为负值
  uint8_t reverse_flag;
  //电机转子计数标志,启用圈速计数反馈
  uint8_t circle_count_flag;

/*目标值*/
  fp32 set_speed;//目标速度 range:0~2PI 下同
  fp32 set_angle;//目标角度
  int16_t set_current;

/*角度反馈*/
  fp32 *ref_ptr;// 反馈变量指针,可以为该结构体的angle成员
  fp32 ecd_angle;// 转子角度反馈
  fp32 last_ecd_angle;// 上一转子角度反馈
  fp32 circle_count;//转子圈数

} DJI_Motor_Ctrl_t;


/**********DJI_Motor*********/
/*DJI_Motor初始化设置*/
void DJI_Motor_init(DJI_Motor_Ctrl_t* motor,DJI_Motor_Bus_t* bus,Motor_Type_e motor_type,uint16_t id);
void DJI_Motor_set_angle_feedback(DJI_Motor_Ctrl_t* motor,fp32* feedback_angle);
void DJI_Motor_Pos_PID_init(DJI_Motor_Ctrl_t* motor,enum PID_MODE pid_mod,
  fp32 Kp,fp32 Ki,fp32 Kd,
  fp32 max_out,fp32 max_iout);
void DJI_Motor_Speed_PID_init(DJI_Motor_Ctrl_t* motor,enum PID_MODE pid_mod,
  fp32 Kp,fp32 Ki,fp32 Kd,
  fp32 max_out,fp32 max_iout);
void DJI_Motor_PID_set_deadband(DJI_Motor_Ctrl_t* motor,fp32 deadband);

/*电机控制接口*/
void DJI_Motor_set_angle(DJI_Motor_Ctrl_t* motor, fp32 angle);//
void DJI_Motor_set_speed(DJI_Motor_Ctrl_t* motor, uint16_t speed_rpm);//速度设置
void DJI_Motor_set_current(DJI_Motor_Ctrl_t* motor, int16_t current);
void DJI_Motor_set_nonforce(DJI_Motor_Ctrl_t* motor);
void DJI_Motor_lockup(DJI_Motor_Ctrl_t* motor);//电机自锁

/*电机反馈接口*/
void DJI_Motor_get_feedback(DJI_Motor_Ctrl_t* motor,fp32 torque,fp32 speed,fp32 angle);

/*DJI_Motor循环控制接口*/
void __DJI_Motor_speed_ctrl_loop(DJI_Motor_Ctrl_t* motor);
void __DJI_Motor_pos_ctrl_loop(DJI_Motor_Ctrl_t* motor);
void __DJI_Motor_current_ctrl_loop(DJI_Motor_Ctrl_t* motor);
void __DJI_Motor_get_feedback(DJI_Motor_Ctrl_t* motor,uint8_t* rx_msg);

/**********PRIVATE**********/
/*预设pid值*/
static void __DJI_Motor_preset_pid_m2006(DJI_Motor_Ctrl_t* motor);
static void __DJI_Motor_preset_pid_m3508(DJI_Motor_Ctrl_t* motor);

static fp32 __DJI_Motor_speed_loop_calc(DJI_Motor_Ctrl_t* motor);
static fp32 __DJI_Motor_angle_loop_calc(DJI_Motor_Ctrl_t* motor);

//static void __DJI_Motor_circle_

#endif
