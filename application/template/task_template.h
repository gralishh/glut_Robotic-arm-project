/**
 * feedback(flash feedback value)->mode_select->mode_ctrl(set target state)->output
 *                                                   |
 *                                                   V
 *                                      rc_ctrl,auto,lock,non_force
 */
#ifndef __TASK_TEMPLATE__
#define __TASK_TEMPLATE__

#include "main.h"
#include <stdint.h>
#include "task.h"

#define MOTOR_COUNT 

typedef enum{
  first_joint=0x00,
  JOINT_COUNT,
}JOINT_INDEX;

typedef enum{
  first_mode=0x00,
  MODE_COUNT,
}CTRL_MODE;

typedef enum{
  first_motor_type=0x00,
}MOTOR_TYPE_E;

typedef struct{
  /*instance array*/
  void* motor_instance[MOTOR_COUNT];

  /*state value*/
  uint8_t ctrl_mode;
  int16_t motor_type[MOTOR_COUNT];
  int16_t motor_ctrl_mode[MOTOR_COUNT];

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
} TASK_HANDLER_TYPE; 
TASK_HANDLER_TYPE task_handler;/*unique structure*/
TASK_HANDLER_TYPE* task_handler_ptr=&task_handler;

void template_task(void *argument);

#undef MOTOR_COUNT

#endif
