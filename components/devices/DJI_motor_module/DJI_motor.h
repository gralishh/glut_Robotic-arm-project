#ifndef __DJI_MOTOR_MODULE__
#define __DJI_MOTOR_MODULE__

#include "main.h"
#include "struct_typedef.h"
#include "pid.h"

#define MAX_MOTOR_MOUNTED 6

/**
 * @brief dji电机总线结构体
 * @date 2024/11/8
 */
typedef struct{
  FDCAN_HandleTypeDef* can;
  DJI_Motor_Ctrl_t* mounted_motor[MAX_MOTOR_MOUNTED];
} DJI_Motor_Bus_t;

typedef struct{
  DJI_Motor_Bus_t* mounted_bus;
  PidTypeDef pid;

  fp32 set_speed;
  fp32 set_angle;//
} DJI_Motor_Ctrl_t;



/*****PRIVATE*****/
/*预设pid值*/

#endif
