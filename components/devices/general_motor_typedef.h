#ifndef __GERNERAL_MOTOR_TYPEDEF__
#define __GERNERAL_MOTOR_TYPEDEF__

#include "main.h"  

// 电机控制模式,由电机控制函数设置
typedef enum{
  NON_FORCE=0,
  SPEED_LOOP,
  POS_LOOP,
  GIVING_CURRENT,
  LOCK
} Motor_Ctrl_mode_e;

typedef enum{
  NON_MOTOR=0x00,
  DJI_MOTOR,
  M8010_MOTOR,
  M4310_MOTOR,
} Motor_Type_e;

#endif
