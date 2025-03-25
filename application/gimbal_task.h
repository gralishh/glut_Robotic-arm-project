#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "main.h"
#include <stdint.h>
#include "general_motor_module.h"
#include "FreeRTOS.h"
#include "task.h"

typedef enum{
  GIMBAL_UPLIFT=0x00,
  GIMBAL_JOINT_COUNT,
}GIMBAL_JOINT_INDEX;

typedef enum{
  DJI_UL=0x00,
  GIMBAL_MOTOR_COUNT,
}GIMBAL_MOTOR_INDEX;

typedef enum{
  GIMBAL_MODE_NONFORCE=0x00,
  GIMBAL_MODE_IDLE,
  GIMBAL_MODE_RC_CTRL,
  GIMBAL_MODE_UPLIFT_RC_CTRL,
  GIMBAL_MODE_CUSTOM_CTRL,
  GIMBAL_MODE_COUNT,
}GIMBAL_CTRL_MODE;

typedef struct{
  /*instance array*/
  void* motor_instance[GIMBAL_MOTOR_COUNT];

  /*state value*/
  uint8_t ctrl_mode;
  uint8_t mode_switch; //模式切换时置1
  Motor_Type_e motor_type[GIMBAL_MOTOR_COUNT];
  Motor_Ctrl_mode_e motor_ctrl_mode[GIMBAL_MOTOR_COUNT];

  /*feedback value*/
  fp32 feedback_motor_angle[GIMBAL_MOTOR_COUNT];
  int32_t feedback_motor_speed[GIMBAL_MOTOR_COUNT];
  int32_t feedback_motor_current[GIMBAL_MOTOR_COUNT];

  /*output value*/
  fp32 motor_angle[GIMBAL_MOTOR_COUNT];
  int32_t motor_speed[GIMBAL_MOTOR_COUNT];
  int32_t motor_current[GIMBAL_MOTOR_COUNT];

  /*for arm ctrl*/
  fp32 feedback_joint_angle[GIMBAL_JOINT_COUNT];
  fp32 joint_angle[GIMBAL_JOINT_COUNT];
  fp32 max_joint_angle[GIMBAL_JOINT_COUNT];
  fp32 min_joint_angle[GIMBAL_JOINT_COUNT];

  uint8_t tick_count_halt;
  int64_t tick;
  int64_t tick_stack[5];// 具体使用取决于任务
} GIMBAL_TASK_HANDLER_TYPE; 
extern GIMBAL_TASK_HANDLER_TYPE gimbal_task_handler;/*unique structure*/
extern GIMBAL_TASK_HANDLER_TYPE* gimbal_task_handler_ptr;

void gimbal_task(void *argument);

#endif



