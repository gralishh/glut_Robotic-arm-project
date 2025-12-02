#include "chassis_task_interface.h"
#include "chassis_task.h"
#include "general_motor_module.h"
#include "DJI_motor_canbus.h"
#include "remote_control.h"
#include "angle_process.h"
#include "cmsis_os2.h"
#include "detect_task.h"
#include "Custom_ctrl.h"
#include "gimbal_task.h"
#include "cmsis_armcc.h"
#include "referee.h"

#include "main.h"
#include "Vofa.h"

fp32 vofa_power_limit = 0;      // vofac查看输出功率
//int16_t motor_current[4] = {0}; // 测试使用

#define HANDLER chassis_task_handler
#define HANDLER_PTR chassis_task_handler_ptr
#define RC_CTRL_PTR (get_remote_control_point())

/*extern*/
extern FDCAN_HandleTypeDef hfdcan1;
extern GIMBAL_TASK_HANDLER_TYPE *gimbal_task_handler_ptr;
#define UL_HEIGHT (gimbal_task_handler_ptr->joint_angle[0])
#define UL_MAX (gimbal_task_handler_ptr->max_joint_angle[0])
#define UL_MIN (gimbal_task_handler_ptr->min_joint_angle[0])

/*global macro variable*/
// joint mapping parameter

// controller sensity(degree per loop)
#define VX_CTRL_SEN 4.23f
#define VY_CTRL_SEN 4.23f
#define WZ_CTRL_SEN 9.0f
// chassis para
#define CHASSIS_WZ_SET_SCALE 0.03f
#define MOTOR_DISTANCE_TO_CENTER 0.3f

/*global motor handler*/
DJI_Motor_Ctrl_t DJI_Motor_LeftFront;
DJI_Motor_Ctrl_t DJI_Motor_RightFront;
DJI_Motor_Ctrl_t DJI_Motor_LeftBack;
DJI_Motor_Ctrl_t DJI_Motor_RightBack;

static void __chassis_nonforce(void);
static void __chassis_idle_ctrl(void);
static void __chassis_rc_ctrl(void);
static void __chassis_rc_slow_ctrl(void);
static void __chassis_union_rc_ctrl(void);
get_botton_dji_motor_current();
// void chassis_reset(void);

/*general handler method*/
/**
 * macro name format:
 *  __<GET/SET>_<MOTOR/JOINT>_<ITEM>(index[,value])
 */
/*获取电机状态*/
#define __GET_MOTOR_INSTANCE(index) (HANDLER_PTR->motor_instance[index])
#define __SET_MOTOR_INSTANCE(index, instance_ptr) (HANDLER_PTR->motor_instance[index] = ((void *)instance_ptr))
#define __GET_STRUCT_MODE() (HANDLER_PTR->ctrl_mode)
#define __SET_STRUCT_MODE(value) (HANDLER_PTR->ctrl_mode = value)

#define __GET_MOTOR_TYPE(index) (HANDLER_PTR->motor_type[index])
#define __SET_MOTOR_TYPE(index, type) (HANDLER_PTR->motor_type[index] = (type))
#define __GET_MOTOR_CTRL_MODE(index) (HANDLER_PTR->motor_ctrl_mode[index])

#define __SET_MOTOR_OFFLINE(index) (HANDLER_PTR->motor_offline_flag[index] = 1)
#define __CLEAR_MOTOR_OFFLINE(index) (HANDLER_PTR->motor_offline_flag[index] = 0)
#define __IS_MOTOR_OFFLINE(index) (HANDLER_PTR->motor_offline_flag[index])

/*获取电机设定目标值*/
#define __GET_SET_MOTOR_ANGLE(index) (HANDLER_PTR->motor_angle[index])
#define __GET_SET_MOTOR_SPEED(index) (HANDLER_PTR->motor_speed[index])
#define __GET_SET_MOTOR_CURRENT(index) (HANDLER_PTR->motor_current[index])
/*获取电机反馈*/
#define __GET_MOTOR_ANGLE(index) (HANDLER_PTR->feedback_motor_angle[index])
#define __GET_MOTOR_SPEED(index) (HANDLER_PTR->feedback_motor_speed[index])
#define __GET_MOTOR_CURRENT(index) (HANDLER_PTR->feedback_motor_current[index])

/*电机输出控制*/
#define __SET_MOTOR_ANGLE(index, value)               \
  {                                                   \
    (HANDLER_PTR->motor_angle[index] = value);        \
    (HANDLER_PTR->motor_ctrl_mode[index] = POS_LOOP); \
  }
#define __ADD_MOTOR_ANGLE(index, value)               \
  {                                                   \
    (HANDLER_PTR->motor_angle[index] += value);       \
    (HANDLER_PTR->motor_ctrl_mode[index] = POS_LOOP); \
  }
#define __SET_MOTOR_SPEED(index, value)                 \
  {                                                     \
    (HANDLER_PTR->motor_speed[index] = value);          \
    (HANDLER_PTR->motor_ctrl_mode[index] = SPEED_LOOP); \
  }
#define __SET_MOTOR_CURRENT(index, value)                   \
  {                                                         \
    (HANDLER_PTR->motor_current[index] = value);            \
    (HANDLER_PTR->motor_ctrl_mode[index] = GIVING_CURRENT); \
  }
#define __SET_MOTOR_LOCKUP(index) (HANDLER_PTR->motor_ctrl_mode[index] = LOCK)
#define __SET_MOTOR_NONFORCE(index) (HANDLER_PTR->motor_ctrl_mode[index] = NON_FORCE)

/*关节控制*/
#define __GET_JOINT_ANGLE(index) (HANDLER_PTR->feedback_joint_angle[index])
#define __SET_JOINT_LIMIT(index, min, max)       \
  {                                              \
    (HANDLER_PTR->max_joint_angle[index] = max); \
    (HANDLER_PTR->min_joint_angle[index] = min); \
  }
#define __JOINT_LIMIT(index, value) angle_limit(value, HANDLER_PTR->max_joint_angle[index], HANDLER_PTR->min_joint_angle[index])
#define __SET_JOINT_ANGLE(index, value)                              \
  {                                                                  \
    (HANDLER_PTR->joint_angle[index] = __JOINT_LIMIT(index, value)); \
    (HANDLER_PTR->motor_ctrl_mode[index] = POS_LOOP);                \
  }
#define __ADD_JOINT_ANGLE(index, value)                                                                  \
  {                                                                                                      \
    (HANDLER_PTR->joint_angle[index] = __JOINT_LIMIT(index, HANDLER_PTR->joint_angle[index] + (value))); \
    (HANDLER_PTR->motor_ctrl_mode[index] = POS_LOOP);                                                    \
  }

/*时间控制*/
#define __RESET_TICKS() (HANDLER_PTR->tick = 0)
#define __HALT_TICKS_COUNTING() (HANDLER_PTR->tick_count_halt = 1)
#define __HOLD_TICKS_COUNTING() (HANDLER_PTR->tick_count_halt = 0)
#define __GET_TICKS() (HANDLER_PTR->tick)
// unit:seconds
#define __GET_TICKS_TIME() (HANDLER_PTR->tick * 1)
#define __GET_PROCESS_PERCENTAGE(PROCESS_TIME) (__GET_TICKS_TIME() / PROCESS_TIME)
#define __IS_MODE_SWITCHED() (1 == HANDLER_PTR->mode_switch)

void chassis_task_init()
{
  /*基础初始化*/
  __HALT_TICKS_COUNTING();
  __RESET_TICKS();

  /*电机初始化*/
  __SET_MOTOR_INSTANCE(DJI_LF, &DJI_Motor_LeftFront);
  __SET_MOTOR_TYPE(DJI_LF, DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_LeftFront, &DJI_CAN1_Bus_ctrl, M3508, 0x201);
  DJI_Motor_Speed_PID_init(&DJI_Motor_LeftFront, PID_POSITION, 18, 0.000, 0.05, 9000, 1000);
  DJI_Motor_Pos_PID_init(&DJI_Motor_LeftFront, PID_POSITION, 15, 0, 0, 1000, 0);

  __SET_MOTOR_INSTANCE(DJI_RF, &DJI_Motor_RightFront);
  __SET_MOTOR_TYPE(DJI_RF, DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_RightFront, &DJI_CAN1_Bus_ctrl, M3508, 0x202);
  DJI_Motor_Speed_PID_init(&DJI_Motor_RightFront, PID_POSITION, 18, 0.000, 0.05, 9000, 1000);
  DJI_Motor_Pos_PID_init(&DJI_Motor_RightFront, PID_POSITION, 15, 0, 0, 1000, 0);

  __SET_MOTOR_INSTANCE(DJI_LB, &DJI_Motor_LeftBack);
  __SET_MOTOR_TYPE(DJI_LB, DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_LeftBack, &DJI_CAN1_Bus_ctrl, M3508, 0x204);
  DJI_Motor_Speed_PID_init(&DJI_Motor_LeftBack, PID_POSITION, 22, 0.000, 0.05, 9000, 1000);
  DJI_Motor_Pos_PID_init(&DJI_Motor_LeftBack, PID_POSITION, 15, 0, 0, 1000, 0);

  __SET_MOTOR_INSTANCE(DJI_RB, &DJI_Motor_RightBack);
  __SET_MOTOR_TYPE(DJI_RB, DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_RightBack, &DJI_CAN1_Bus_ctrl, M3508, 0x203);
  DJI_Motor_Speed_PID_init(&DJI_Motor_RightBack, PID_POSITION, 18, 0.000, 0.05, 9000, 1000);
  DJI_Motor_Pos_PID_init(&DJI_Motor_RightBack, PID_POSITION, 15, 0, 0, 1000, 0);

  __chassis_idle_ctrl();
  DJI_CANBus_enable_bus(&DJI_CAN1_Bus_ctrl);

  // while (
  //     toe_is_error(TOE_3508_M1_ID) ||
  //     toe_is_error(TOE_3508_M2_ID) ||
  //     toe_is_error(TOE_3508_M3_ID) ||
  //     toe_is_error(TOE_3508_M4_ID) ||
  //       )
  //   ;
  // __CLEAR_MOTOR_OFFLINE(TOE_3508_M1_ID);
  // __CLEAR_MOTOR_OFFLINE(TOE_3508_M2_ID);
  // __CLEAR_MOTOR_OFFLINE(TOE_3508_M3_ID);
  // __CLEAR_MOTOR_OFFLINE(TOE_3508_M4_ID);
  // osDelay(20);
}

/**
 * @brief 刷新句柄的反馈值
 * @details 遍历电机控制句柄，针对点击类型刷新反馈
 * @code
 */
void chassis_task_get_feedback()
{
  uint16_t index;
  /*motor feedback*/
  for (index = 0; index < CHASSIS_MOTOR_COUNT; index++)
  {
    GENERAL_MOTOR_GET_FEEDBACK(__GET_MOTOR_INSTANCE(index),
                               __GET_MOTOR_TYPE(index),
                               &__GET_MOTOR_CURRENT(index),
                               &__GET_MOTOR_SPEED(index),
                               &__GET_MOTOR_ANGLE(index))
  }

  /*joint angle map*/
  /*
  __GET_JOINT_ANGLE(index,
    ...
  )
  ...
  */
}

/**
 * @brief 模式状态刷新
 * @details 根据控制器拨杆刷新模式(二级模式会与UI耦合)
 */
void chassis_task_mode_flush()
{
  /**/
  static uint8_t last_mode = CHASSIS_MODE_NONFORCE;
  last_mode = HANDLER_PTR->ctrl_mode;
  //  switch(GET_SWITCH())
  //  {
  //    case 1:
  //      __SET_STRUCT_MODE(CHASSIS_MODE_RC_CTRL);
  //      break;
  //    case 2:
  //      __SET_STRUCT_MODE(CHASSIS_MODE_RC_CTRL);
  //      break;
  //    case 0:
  //    default:
  //      __SET_STRUCT_MODE(CHASSIS_MODE_NONFORCE);
  //  }
  if (switch_is_down(get_remote_control_point()->rc.s[1]))
  {
    if (switch_is_mid(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(CHASSIS_MODE_RC_CTRL);
    else if (switch_is_up(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(CHASSIS_MODE_RC_CTRL);
  }
  else if (switch_is_mid(get_remote_control_point()->rc.s[1]))
  {
    if (switch_is_mid(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(CHASSIS_MODE_UNION_CTRL);
    else
      __SET_STRUCT_MODE(CHASSIS_MODE_IDLE);
  }
  else if (switch_is_up(get_remote_control_point()->rc.s[1]))
  {
    if (switch_is_mid(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(CHASSIS_MODE_IDLE);
    else
      __SET_STRUCT_MODE(CHASSIS_MODE_IDLE);
  }
  else
  {
    __SET_STRUCT_MODE(CHASSIS_MODE_IDLE);
  }

  if (switch_is_down(get_remote_control_point()->rc.s[1]) && switch_is_down(get_remote_control_point()->rc.s[0]))
  {
    __SET_STRUCT_MODE(CHASSIS_MODE_NONFORCE);
  }

  if (__GET_STRUCT_MODE() == CHASSIS_MODE_NONFORCE)
  {
    if ((GET_KEY(KEY_SHIFT) && GET_KEY(KEY_CTRL) && GET_KEY(KEY_B)))
    {
      __set_FAULTMASK(1); // 鹿乇諎霉訍謵露蠉
      NVIC_SystemReset(); // 陆酶袗拳录镁赂麓位
    }
  }

  {
    static uint16_t count = 0;
    if (count < 1000 && remote_data.pause)
      count++;
    if (count == 1000)
    {
      __set_FAULTMASK(1); // 鹿乇諎霉訍謵露蠉
      NVIC_SystemReset(); // 陆酶袗拳录镁赂麓位
    }
    if (!remote_data.pause)
      count = 0;
  }

  //  if(toe_is_error(TOE_HE_L) &&
  //    toe_is_error(TOE_HE_R) &&
  //    toe_is_error(TOE_J1) &&
  //    toe_is_error(TOE_J2) &&
  //    toe_is_error(TOE_J3) &&
  //    toe_is_error(TOE_UPLIFT) &&
  //    toe_is_error(TOE_3508_M1_ID) &&
  //    toe_is_error(TOE_3508_M2_ID) &&
  //    toe_is_error(TOE_3508_M3_ID) &&
  //    toe_is_error(TOE_3508_M4_ID)
  //  )
  //  {
  //        __set_FAULTMASK(1); //鹿乇諎霉訍謵露蠉
  //    NVIC_SystemReset(); //陆酶袗拳录镁赂麓位
  //  }

  // if(GetMatchReady())
  //{
  //   __SET_STRUCT_MODE(CHASSIS_MODE_NONFORCE);
  // }

  if (toe_is_error(DBUSTOE) && toe_is_error(CAMERA_TOE))
  {
    __SET_STRUCT_MODE(CHASSIS_MODE_NONFORCE);
  }

  if (HANDLER_PTR->ctrl_mode == last_mode)
    HANDLER_PTR->mode_switch = 0;
  else
    HANDLER_PTR->mode_switch = 1;
}

/**
 * @brief 设置输出量(电流|速度|位置|力矩)
 * @details 模式控制
 */
void chassis_task_set_output()
{
  switch (__GET_STRUCT_MODE())
  {
  case CHASSIS_MODE_IDLE:
    __chassis_idle_ctrl();
    break;
  case CHASSIS_MODE_RC_CTRL:
    __chassis_rc_ctrl();
    break;
  case CHASSIS_MODE_UNION_CTRL:
    __chassis_union_rc_ctrl();
    break;
  case CHASSIS_MODE_SLOW_CTRL:
    __chassis_rc_slow_ctrl();
    break;
  case CHASSIS_MODE_NONFORCE:
  default:
    __chassis_nonforce();
  }
}

/**
 * @brief 控制输出
 * @details 根据电机种类与控制状态设定输出
 */
void chassis_task_output()
{
  uint16_t index;
  /*joint map to motor state*/

  /*motor output*/
  for (index = 0; index < CHASSIS_MOTOR_COUNT; index++)
  {
    GENERAL_MOTOR_SET_OUTPUT(__GET_MOTOR_INSTANCE(index),
                             __GET_MOTOR_TYPE(index),
                             __GET_MOTOR_CTRL_MODE(index),
                             HANDLER_PTR->motor_current[index],
                             HANDLER_PTR->motor_speed[index],
                             HANDLER_PTR->motor_angle[index])
  }

  // /*掉电检测*/
  // if (toe_is_error(TOE_3508_M1_ID) || __IS_MOTOR_OFFLINE(TOE_3508_M1_ID))
  // {
  //   __SET_MOTOR_CTRL_MODE(TOE_3508_M1_ID, OFFLINE);
  //   __SET_MOTOR_OFFLINE(TOE_3508_M1_ID);
  // }
  // if (toe_is_error(TOE_3508_M2_ID) || __IS_MOTOR_OFFLINE(TOE_3508_M2_ID))
  // {
  //   __SET_MOTOR_CTRL_MODE(TOE_3508_M2_ID, OFFLINE);
  //   __SET_MOTOR_OFFLINE(TOE_3508_M2_ID);
  // }
  // if (toe_is_error(TOE_3508_M3_ID) || __IS_MOTOR_OFFLINE(TOE_3508_M3_ID))
  // {
  //   __SET_MOTOR_CTRL_MODE(TOE_3508_M3_ID, OFFLINE);
  //   __SET_MOTOR_OFFLINE(TOE_3508_M3_ID);
  // }
  // if (toe_is_error(TOE_3508_M4_ID) || __IS_MOTOR_OFFLINE(TOE_3508_M4_ID))
  // {
  //   __SET_MOTOR_CTRL_MODE(TOE_3508_M4_ID, OFFLINE);
  //   __SET_MOTOR_OFFLINE(TOE_3508_M4_ID);
  // }
}

void __chassis_nonforce()
{
  int index;
  for (index = 0; index < CHASSIS_MOTOR_COUNT; index++)
  {
    __SET_MOTOR_NONFORCE(index);
  }
}

void __chassis_idle_ctrl()
{
  int index;

  if (__IS_MODE_SWITCHED())
  {
    for (index = 0; index < CHASSIS_MOTOR_COUNT; index++)
    {
      __SET_MOTOR_ANGLE(index, __GET_MOTOR_ANGLE(index));
    }
  }

  for (index = 0; index < CHASSIS_MOTOR_COUNT; index++)
  {
    __ADD_MOTOR_ANGLE(index, 0);
  }
}

void __chassis_rc_ctrl()
{
  /*设置输入速度(not real)*/
  HANDLER_PTR->vx = -RC_CTRL_PTR->rc.ch[3] * 8 / 6 * VX_CTRL_SEN;
  HANDLER_PTR->vy = -RC_CTRL_PTR->rc.ch[2] * 8 / 6 * VY_CTRL_SEN;
  HANDLER_PTR->wz = -RC_CTRL_PTR->rc.ch[0] * 7 / 6 * WZ_CTRL_SEN;

  /*
  if (!remote_data.trigger)
  {
    HANDLER_PTR->vx = -GET_CH_VALUE(2) * 9 / 6 * VX_CTRL_SEN;
    HANDLER_PTR->vy = -GET_CH_VALUE(3) * 9 / 6 * VY_CTRL_SEN;
    HANDLER_PTR->wz = -GET_CH_VALUE(0) * 9 / 6 * WZ_CTRL_SEN;

    if (GET_KEY(KEY_W))
      HANDLER_PTR->vx = -330 * VX_CTRL_SEN;
    if (GET_KEY(KEY_S))
      HANDLER_PTR->vx = 330 * VX_CTRL_SEN;
    if (GET_KEY(KEY_A))
      HANDLER_PTR->vy = 330 * VY_CTRL_SEN;
    if (GET_KEY(KEY_D))
      HANDLER_PTR->vy = -330 * VY_CTRL_SEN;
    if (GET_KEY(KEY_Q))
      HANDLER_PTR->wz = 330 * WZ_CTRL_SEN;
    if (GET_KEY(KEY_E))
      HANDLER_PTR->wz = -330 * WZ_CTRL_SEN;

#define TEMP_SENS (1100 - (UL_HEIGHT > UL_MAX / 4 && UL_HEIGHT != 0 && UL_MAX != 0 ? 660 * (UL_HEIGHT - UL_MAX / 4) / (UL_MAX * 3 / 4) : 0))
    if (GET_KEY(KEY_SHIFT))
    {
      if (GET_KEY(KEY_W))
        HANDLER_PTR->vx = -TEMP_SENS * VX_CTRL_SEN;
      if (GET_KEY(KEY_S))
        HANDLER_PTR->vx = TEMP_SENS * VX_CTRL_SEN;
      if (GET_KEY(KEY_A))
        HANDLER_PTR->vy = TEMP_SENS * VY_CTRL_SEN;
      if (GET_KEY(KEY_D))
        HANDLER_PTR->vy = -TEMP_SENS * VY_CTRL_SEN;
      if (GET_KEY(KEY_Q))
        HANDLER_PTR->wz = 330 * WZ_CTRL_SEN;
      if (GET_KEY(KEY_E))
        HANDLER_PTR->wz = -330 * WZ_CTRL_SEN;
    }
#undef TEMP_SENS

    if (remote_data.mouse_x != 0)
      HANDLER_PTR->wz = -remote_data.mouse_x * 10 * WZ_CTRL_SEN;
  }
  */

  __SET_MOTOR_SPEED(DJI_LF, -HANDLER_PTR->vx - HANDLER_PTR->vy + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  __SET_MOTOR_SPEED(DJI_RF, HANDLER_PTR->vx - HANDLER_PTR->vy + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  __SET_MOTOR_SPEED(DJI_RB, HANDLER_PTR->vx + HANDLER_PTR->vy + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  __SET_MOTOR_SPEED(DJI_LB, -HANDLER_PTR->vx + HANDLER_PTR->vy + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
}

void __chassis_rc_slow_ctrl(void)
{
  /*设置输入速度(not real)*/
  HANDLER_PTR->vx = -RC_CTRL_PTR->rc.ch[3] * VX_CTRL_SEN * 0.4f;
  HANDLER_PTR->vy = -RC_CTRL_PTR->rc.ch[2] * VY_CTRL_SEN * 0.4f;
  HANDLER_PTR->wz = -RC_CTRL_PTR->rc.ch[0] * WZ_CTRL_SEN * 0.9f;

  // HANDLER_PTR->vx = -GET_CH_VALUE(2) * 0.4f * VX_CTRL_SEN;
  // HANDLER_PTR->vy = -GET_CH_VALUE(3) * 0.4f * VY_CTRL_SEN;
  // HANDLER_PTR->wz = -GET_CH_VALUE(0) * 0.9f * WZ_CTRL_SEN;

  if (GET_KEY(KEY_W))
    HANDLER_PTR->vx = -330 * VX_CTRL_SEN;
  if (GET_KEY(KEY_S))
    HANDLER_PTR->vx = 330 * VX_CTRL_SEN;
  if (GET_KEY(KEY_A))
    HANDLER_PTR->vy = 330 * VY_CTRL_SEN;
  if (GET_KEY(KEY_D))
    HANDLER_PTR->vy = -330 * VY_CTRL_SEN;
  if (GET_KEY(KEY_Q))
    HANDLER_PTR->wz = 440 * WZ_CTRL_SEN;
  if (GET_KEY(KEY_E))
    HANDLER_PTR->wz = -440 * WZ_CTRL_SEN;

  // if (remote_data.mouse_x != 0)
  //   HANDLER_PTR->wz = -remote_data.mouse_x * 3 * WZ_CTRL_SEN;

  // __SET_MOTOR_SPEED(DJI_LF, -HANDLER_PTR->vx - HANDLER_PTR->vy + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  // __SET_MOTOR_SPEED(DJI_RF, HANDLER_PTR->vx - HANDLER_PTR->vy + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  // __SET_MOTOR_SPEED(DJI_RB, HANDLER_PTR->vx + HANDLER_PTR->vy + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  // __SET_MOTOR_SPEED(DJI_LB, -HANDLER_PTR->vx + HANDLER_PTR->vy + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
}

void __chassis_union_rc_ctrl()
{
  /*设置输入速度(not real)*/
  HANDLER_PTR->vx = -RC_CTRL_PTR->rc.ch[3] * VX_CTRL_SEN;
  HANDLER_PTR->vy = -RC_CTRL_PTR->rc.ch[2] * VY_CTRL_SEN;
  HANDLER_PTR->wz = 0;

  if (GET_KEY(KEY_W))
    HANDLER_PTR->vx = -220 * VX_CTRL_SEN;
  if (GET_KEY(KEY_S))
    HANDLER_PTR->vx = 220 * VX_CTRL_SEN;
  if (GET_KEY(KEY_A))
    HANDLER_PTR->vy = 220 * VY_CTRL_SEN;
  if (GET_KEY(KEY_D))
    HANDLER_PTR->vy = -220 * VY_CTRL_SEN;
  if (GET_KEY(KEY_Q))
    HANDLER_PTR->wz = 220 * WZ_CTRL_SEN;
  if (GET_KEY(KEY_E))
    HANDLER_PTR->wz = -330 * WZ_CTRL_SEN;

  __SET_MOTOR_SPEED(DJI_LF, -HANDLER_PTR->vx * 0.5f - HANDLER_PTR->vy * 0.5f + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  __SET_MOTOR_SPEED(DJI_RF, HANDLER_PTR->vx * 0.5f - HANDLER_PTR->vy * 0.5f + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  __SET_MOTOR_SPEED(DJI_RB, HANDLER_PTR->vx * 0.5f + HANDLER_PTR->vy * 0.5f + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  __SET_MOTOR_SPEED(DJI_LB, -HANDLER_PTR->vx * 0.5f + HANDLER_PTR->vy * 0.5f + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
}

// void chassis_reset()
// {
//   uint16_t index;
//   void chassis_task_init();
//   for (index = 0; index < CHASSIS_MOTOR_COUNT; index++)
//   {
//     __CLEAR_MOTOR_OFFLINE();
//   }
// }

// 回来改，不行并不是实际的电流更改没有
// void get_botton_dji_motor_current(void)
// {
//   motor_current[0] = DJI_CAN1_Bus_ctrl.output_current200H[0];
//   motor_current[1] = DJI_CAN1_Bus_ctrl.output_current200H[1];
//   motor_current[2] = DJI_CAN1_Bus_ctrl.output_current200H[3];
//   motor_current[3] = DJI_CAN1_Bus_ctrl.output_current200H[2];
// }
(HANDLER_PTR->(DJI_Motor_Ctrl_t *)motor_instance[i])
    ->speed_rpm;
// 移植步兵功率控制
void chassis_power_control_with_supercap(void)
{

  uint16_t RefereePowerLimit = 20;
  float ChassisMaxPower = 0;

  float InitialGivePower[4]; // initial power from PID calculation
  float InitialTotalPower = 0;
  float ScaledGivePower[4];

  float chassis_energy_buffer = 0.0f;

  float toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
  float k1 = 1.23e-07;                      // k1
  float k2 = 1.453e-07;                     // k2
  float constant = 4.081f;

  float power_scale = 0;
  float eneygy_scale = 0;
  uint8_t PowerOffset = 0;

  ChassisMaxPower = RefereePowerLimit;

  for (uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
  {
    DJI_Motor_Ctrl_t *motor = (DJI_Motor_Ctrl_t *)(chassis_task_handler_ptr->motor_instance[i]);

    InitialGivePower[i] = DJI_CAN1_Bus_ctrl.output_current200H[i] * toque_coefficient * motor->speed_rpm +
                          k2 * motor->speed_rpm * motor->speed_rpm +
                          k1 * DJI_CAN1_Bus_ctrl.output_current200H[i] * DJI_CAN1_Bus_ctrl.output_current200H[i] + constant;

    if (InitialGivePower < 0) // negative power not included (transitory)
      continue;
    InitialTotalPower += InitialGivePower[i];
  }

  if (InitialTotalPower > ChassisMaxPower) // determine if larger than max power
  {
    float power_scale = ChassisMaxPower / InitialTotalPower;
    for (uint8_t i = 0; i < 4; i++)
    {
      ScaledGivePower[i] = InitialGivePower[i] * power_scale; // get scaled power
      if (ScaledGivePower[i] < 0)
      {
        continue;
      }

      float b = toque_coefficient * motor->speed_rpm;
      float c = k2 * motor->speed_rpm * motor->speed_rpm - ScaledGivePower[i] + constant;
      float inside = b * b - 4 * k1 * c;

      if (inside < 0)
      {
        continue;
      }
      else if (DJI_CAN1_Bus_ctrl.output_current200H[i] > 0) // Selection of the calculation formula according to the direction of the original moment
      {
        float temp = (-b + sqrt(inside)) / (2 * k1);
        if (temp > 8000)
        {
          DJI_CAN1_Bus_ctrl.output_current200H[i] = 8000;
        }
        else
          DJI_CAN1_Bus_ctrl.output_current200H[i] = temp;
      }
      else
      {
        float temp = (-b - sqrt(inside)) / (2 * k1);
        if (temp < -8000)
        {
          DJI_CAN1_Bus_ctrl.output_current200H[i] = -8000;
        }
        else
          DJI_CAN1_Bus_ctrl.output_current200H[i] = temp;
      }
    }
  }
}
#undef HANDLER
#undef HANDLER_PTR
