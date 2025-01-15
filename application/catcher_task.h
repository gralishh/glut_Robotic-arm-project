#ifndef _CATCHER_TASK_H
#define _CATCHER_TASK_H

#include "main.h"
#include <stdint.h>
#include "general_motor_module.h"
#include "FreeRTOS.h"
#include "task.h"

typedef enum{
  CAT_UPLIFT,
  CAT_PUSHOUT,
  CAT_PICKUP,
  CATCHER_JOINT_COUNT,
}CATCHER_JOINT_INDEX;

typedef enum{
  DJI_CAT_UL=0x00,
  DJI_CAT_PUSH,
  DJI_CAT_PICK,
  CATCHER_MOTOR_COUNT,
}CATCHER_MOTOR_INDEX;

typedef enum{
  CATCHER_MODE_NONFORCE=0x00,
  CATCHER_MODE_IDLE,
  CATCHER_MODE_RC_CTRL,
  CATCHER_MODE_POSE_CTRL,
  CATCHER_MODE_COUNT,
}CATCHER_CTRL_MODE;

typedef struct{
  /*instance array*/
  void* motor_instance[CATCHER_MOTOR_COUNT];

  /*state value*/
  uint8_t ctrl_mode;
  uint8_t mode_switch; //模式切换时置1
  Motor_Type_e motor_type[CATCHER_MOTOR_COUNT];
  Motor_Ctrl_mode_e motor_ctrl_mode[CATCHER_MOTOR_COUNT];

  /*feedback value*/
  fp32 feedback_motor_angle[CATCHER_MOTOR_COUNT];
  int32_t feedback_motor_speed[CATCHER_MOTOR_COUNT];
  int32_t feedback_motor_current[CATCHER_MOTOR_COUNT];

  /*output value*/
  fp32 motor_angle[CATCHER_MOTOR_COUNT];
  int32_t motor_speed[CATCHER_MOTOR_COUNT];
  int32_t motor_current[CATCHER_MOTOR_COUNT];

  /*for arm ctrl*/
  fp32 feedback_joint_angle[CATCHER_JOINT_COUNT];
  fp32 joint_angle[CATCHER_JOINT_COUNT];
  fp32 max_joint_angle[CATCHER_JOINT_COUNT];
  fp32 min_joint_angle[CATCHER_JOINT_COUNT];

  uint8_t tick_count_halt;
  int64_t tick;
  int64_t tick_stack[5];// 具体使用取决于任务
} CATCHER_TASK_HANDLER_TYPE; 
extern CATCHER_TASK_HANDLER_TYPE hand_task_handler;/*unique structure*/
extern CATCHER_TASK_HANDLER_TYPE* hand_task_handler_ptr;

void catcher_task(void *argument);

#endif
