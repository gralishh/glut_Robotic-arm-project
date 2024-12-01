#ifndef _HAND_TASK_H
#define _HAND_TASK_H

#include "main.h"
#include <stdint.h>
#include "general_motor_module.h"
#include "task.h"

typedef enum{
  first_joint=0x00,
  JOINT_COUNT,
}HAND_JOINT_INDEX;

typedef enum{
  first_motor,
  MOTOR_COUNT,
}HAND_MOTOR_INDEX;

typedef enum{
  first_mode=0x00,
  MODE_COUNT,
}HAND_CTRL_MODE;

typedef struct{
  /*instance array*/
  void* motor_instance[MOTOR_COUNT];

  /*state value*/
  uint8_t ctrl_mode;
  Motor_Type_e motor_type[MOTOR_COUNT];
  Motor_Ctrl_mode_e motor_ctrl_mode[MOTOR_COUNT];

  /*feedback value*/
  fp32 feedback_motor_angle[MOTOR_COUNT];
  int32_t feedback_motor_speed[MOTOR_COUNT];
  int32_t feedback_motor_current[MOTOR_COUNT];

  /*output value*/
  fp32 motor_angle[MOTOR_COUNT];
  int32_t motor_speed[MOTOR_COUNT];
  int32_t motor_current[MOTOR_COUNT];

  /*for arm ctrl*/
  fp32 feedback_joint_angle[JOINT_COUNT];
  fp32 joint_angle[JOINT_COUNT];
} HAND_TASK_HANDLER_TYPE; 
HAND_TASK_HANDLER_TYPE hand_task_handler;/*unique structure*/
HAND_TASK_HANDLER_TYPE* hand_task_handler_ptr=&hand_task_handler;

void hand_task(void *argument);

#endif
