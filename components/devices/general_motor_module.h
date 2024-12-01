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

#define GENERAL_MOTOR_GET_FEEDBACK(instance_ptr,motor_type,current_ptr,speed_ptr,angle_ptr) \
{ \
  switch(motor_type) \
  { \
    case DJI_MOTOR: \
      DJI_Motor_get_feedback( \
        (DJI_Motor_Ctrl_t*)instance_ptr, \
        (current_ptr), \
        (speed_ptr), \
        (angle_ptr) \
      ); \
      break; \
    default: \
  } \
} \

#define GENERAL_MOTOR_SET_OUTPUT(instance_ptr,motor_type,ctrl_state,current,speed,angle) \
{ \
  switch(motor_type) \
  { \
    case DJI_MOTOR: \
      switch(ctrl_state) \
      { \
        case SPEED_LOOP: \
          DJI_Motor_set_speed(instance_ptr,speed); \
          break; \
        case POS_LOOP: \
          DJI_Motor_set_angle(instance_ptr,angle); \
          break; \
        case GIVING_CURRENT: \
          DJI_Motor_set_current(instacne_ptr,current); \
          break; \
        case LOCK: \
          DJI_Motor_set_current(instance_ptr); \
          break; \
        case NON_FORCE: \
        default: \
          DJI_Motor_set_nonforce(instance_ptr); \
          break; \
      } \
      break; \
    default: \
  } \
} \

#endif
