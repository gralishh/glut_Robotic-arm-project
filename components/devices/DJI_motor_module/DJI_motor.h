/**
 * @brief DJI电机控制
 * @date 24/11/9
 * @note DJI_Motor_Bus_t仅针对CAN端口
 * 上层代码仅对电机句柄操作
 * @todo 添加电机输出转轴速度控制
 */
#ifndef __DJI_MOTOR_MODULE__
#define __DJI_MOTOR_MODULE__

#include "main.h"
#include "struct_typedef.h"
#include "pid.h"

#define MAX_MOTOR_MOUNTED 6
#define IS_OUTPUT_ID_200H(id) ((id>=0x201)&&(id<=0x204))
#define IS_OUTPUT_ID_1FFH(id) ((id>=0x205)&&(id<=0x208))

typedef enum{
  M2006=0,
  M3508,
  ANY_MOTOR,
} Motor_Type_t;


typedef struct{

} DJI_Motor_Config_t;


/**
 * @brief dji电机总线句柄结构体
 * @date 24/11/8
 */
typedef struct{
  FDCAN_HandleTypeDef* can;
  DJI_Motor_Ctrl_t* mounted_motor[MAX_MOTOR_MOUNTED];

  uint16_t output_current200H[4];
  uint16_t output_current1FFH[4];
} DJI_Motor_Bus_t;

typedef struct{
  DJI_Motor_Bus_t* mounted_bus;
  PidTypeDef pid_speed_loop;
  dji_recv_pack recv_pack;

  Motor_Type_t type;
  uint16_t id;

  uint8_t enable_flag;//电机启动标志

  fp32 set_speed;
  fp32 set_angle;//

  fp32 circle_count;//转子圈数
} DJI_Motor_Ctrl_t;

/*反馈报文*/
typedef __packed struct{
  uint16_t ecd;
  uint16_t speed_rpm;
  uint16_t torque;
  uint8_t temp;
  uint8_t REMAIN;
} dji_recv_pack;

/*DJI_CANBus*/
void DJI_CANBus_init(DJI_Motor_Bus_t* bus,FDCAN_HandleTypeDef* can);
void DJI_CANBus_config_init(DJI_Motor_Bus_t* bus, DJI_Motor_Config_t* config);
void DJI_CANBus_add_motor(DJI_Motor_Bus_t* bus);

void DJI_CANBus_ctrl_loop(DJI_Motor_Bus_t* bus);
void DJI_CANBus_feedback_update(DJI_Motor_Bus_t* bus, uint16_t rx_id);


/*DJI_Motor*/
void DJI_Motor_init(DJI_Motor_Ctrl_t* motor,DJI_Motor_Bus_t* bus,Motor_Type_t motor_type,uint16_t id);
void DJI_Motor_set_speed(DJI_Motor_Ctrl_t* motor, uint16_t speed_rpm);
void DJI_Motor_set_angle_feedback(DJI_Motor_Ctrl_t* motor,fp32* feedback_angle);
void DJI_Motor_PID_init(DJI_Motor_Ctrl_t* motor,enum PID_MODE pid_mod,
  fp32 Kp,fp32 Ki,fp32 Kd,
  fp32 max_out,fp32 max_iout);
void DJI_Motor_PID_set_deadband(DJI_Motor_Ctrl_t* motor,fp32 deadband);

void DJI_Motor_speed_ctrl_loop(DJI_Motor_Ctrl_t* motor);
void DJI_Motor_get_feedback(DJI_Motor_Ctrl_t* motor,uint8_t* rx_msg);

/*****PRIVATE*****/
/*预设pid值*/
static void __DJI_Motor_preset_pid_m2006(DJI_Motor_Ctrl_t* motor);
static void __DJI_Motor_preset_pid_m3508(DJI_Motor_Ctrl_t* motor);

static void __DJI_Motor_speed_loop_calc(DJI_Motor_Ctrl_t* motor);
static void __DJI_Motor_angle_loop_calc(DJI_Motor_Ctrl_t* motor,);

static void __DJI_Motor_circle_

#endif
