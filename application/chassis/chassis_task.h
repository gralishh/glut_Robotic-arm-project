#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include "main.h"
#include <stdint.h>
#include "general_motor_module.h"
#include "FreeRTOS.h"
#include "task.h"
//test
typedef enum{
  CHASSIS_JOINT_COUNT,
}CHASSIS_JOINT_INDEX;

typedef enum{
  DJI_LF=0x00,
  DJI_RF,
  DJI_RB,
  DJI_LB,
  CHASSIS_MOTOR_COUNT,
}CHASSIS_MOTOR_INDEX;

typedef enum{
  CHASSIS_MODE_NONFORCE=0x00,
  CHASSIS_MODE_IDLE,
  CHASSIS_MODE_RC_CTRL,
  CHASSIS_MODE_UNION_CTRL,
  CHASSIS_MODE_SLOW_CTRL,
  CHASSIS_MODE_COUNT,
}CHASSIS_CTRL_MODE;

typedef struct{
  /*instance array*/
  void* motor_instance[CHASSIS_MOTOR_COUNT];

  /*state value*/
  uint8_t ctrl_mode;
  uint8_t mode_switch; //??????1
  Motor_Type_e motor_type[CHASSIS_MOTOR_COUNT];
  Motor_Ctrl_mode_e motor_ctrl_mode[CHASSIS_MOTOR_COUNT];
  uint8_t motor_offline_flag[CHASSIS_MOTOR_COUNT];

  /*feedback value*/
  fp32 feedback_motor_angle[CHASSIS_MOTOR_COUNT];
  // int32_t feedback_motor_speed[CHASSIS_MOTOR_COUNT];
  // int32_t feedback_motor_current[CHASSIS_MOTOR_COUNT];
  fp32 feedback_motor_speed[CHASSIS_MOTOR_COUNT];
  fp32 feedback_motor_current[CHASSIS_MOTOR_COUNT];

  /*output value*/
  fp32 motor_angle[CHASSIS_MOTOR_COUNT];
  // int32_t motor_speed[CHASSIS_MOTOR_COUNT];
  // int32_t motor_current[CHASSIS_MOTOR_COUNT];
  fp32 motor_speed[CHASSIS_MOTOR_COUNT];
  fp32 motor_current[CHASSIS_MOTOR_COUNT];

  /*for arm ctrl*/
  fp32 feedback_joint_angle[CHASSIS_JOINT_COUNT];
  fp32 joint_angle[CHASSIS_JOINT_COUNT];
  fp32 max_joint_angle[CHASSIS_JOINT_COUNT];
  fp32 min_joint_angle[CHASSIS_JOINT_COUNT];

  uint8_t tick_count_halt;
  int64_t tick;

  /*µ×ÅÌÔË¶¯×´Ì¬*/
  fp32 vx;
  fp32 vy;
  fp32 wz;
} CHASSIS_TASK_HANDLER_TYPE; 
extern CHASSIS_TASK_HANDLER_TYPE chassis_task_handler;/*unique structure*/
extern CHASSIS_TASK_HANDLER_TYPE* chassis_task_handler_ptr;

void chassis_task(void *argument);

#endif
