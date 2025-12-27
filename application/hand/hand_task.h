#ifndef _HAND_TASK_H
#define _HAND_TASK_H

#include "main.h"
#include <stdint.h>
#include "general_motor_module.h"
#include "FreeRTOS.h"
#include "task.h"

typedef enum
{
  HAND_J1 = 0x00,
  HAND_J2,
  HAND_J3,
  HAND_G,
  // HAND_PITCH,
  // HAND_ROLL,
  HAND_JOINT_COUNT,
} HAND_JOINT_INDEX;

typedef enum
{
  //  M8010_J1,
  AK_J1,
  DM_J2,
  DJI_J3,
  dm_gripper,
  // DJI_HE_L,
  // DJI_HE_R,
  HAND_MOTOR_COUNT,
} HAND_MOTOR_INDEX;

typedef enum
{
  HAND_MODE_NONFORCE = 0x00,
  HAND_MODE_IDLE,
  HAND_MODE_RC_CTRL,
  HAND_MODE_POSE_CTRL,
  HAND_MODE_CUSTOM_CTRL,
  HAND_MODE_GOLE_CATCH_CTRL,
  HAND_MODE_SM_CTRL,
  HAND_MODE_RESET_CTRL,
  HAND_MODE_COUNT,
} HAND_CTRL_MODE;

typedef struct
{
  /*instance array*/
  void *motor_instance[HAND_MOTOR_COUNT];

  /*state value*/
  uint8_t ctrl_mode;
  uint8_t mode_switch; // 模式切换时置1
  Motor_Type_e motor_type[HAND_MOTOR_COUNT];
  Motor_Ctrl_mode_e motor_ctrl_mode[HAND_MOTOR_COUNT];
  uint8_t motor_offline_flag[HAND_MOTOR_COUNT];

  /*feedback value*/
  fp32 feedback_motor_angle[HAND_MOTOR_COUNT];
  fp32 feedback_motor_speed[HAND_MOTOR_COUNT];
  int32_t feedback_motor_current[HAND_MOTOR_COUNT];

  /*output value*/
  fp32 motor_angle[HAND_MOTOR_COUNT];
  int32_t motor_speed[HAND_MOTOR_COUNT];
  int32_t motor_current[HAND_MOTOR_COUNT];

  /*for arm ctrl*/
  fp32 feedback_joint_angle[HAND_JOINT_COUNT];
  fp32 joint_angle[HAND_JOINT_COUNT];
  fp32 max_joint_angle[HAND_JOINT_COUNT];
  fp32 min_joint_angle[HAND_JOINT_COUNT];

  uint8_t tick_count_halt;
  int64_t tick;
  int64_t tick_stack[5]; // 具体使用取决于任务
} HAND_TASK_HANDLER_TYPE;
extern HAND_TASK_HANDLER_TYPE hand_task_handler; /*unique structure*/
extern HAND_TASK_HANDLER_TYPE *hand_task_handler_ptr;

void hand_task(void *argument);

#endif
