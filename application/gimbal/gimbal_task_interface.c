#include "gimbal_task_interface.h"
#include "gimbal_task.h"
#include "general_motor_module.h"
#include "DJI_motor_canbus.h"
#include "remote_control.h"
#include "angle_process.h"
#include "cmsis_os2.h"
#include "detect_task.h"
#include "Custom_ctrl.h"
#include "Ex_encoder.h"
#include "servo.h"
#include "general_movement.h"

#include "referee.h"

#define HANDLER gimbal_task_handler
#define HANDLER_PTR gimbal_task_handler_ptr
#define RC_CTRL_PTR (get_remote_control_point())
//由于26赛季将末端需求从吸盘改为夹爪，该模块多处和气泵相关的代码注释，可以如无刚需建议保留以防以后需要

/*extern*/
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan3;

/*ABS*/
#define ABS(X) ((X) > 0 ? (X) : -(X))
/*global macro variable*/
// joint mapping parameter
#define UL_MAP_K 1
#define UL_MAP_D 0
#define UL_MAX_ENCODE 438
#define UL_MIN_ENCODE 5
// controller sensity(degree per loop)
#define uplift_custom_controller_K (-(UL_MAX_ENCODE - UL_MIN_ENCODE) / (3.75f+2.25f))
#define uplift_custom_controller_D (3.75f)
/*global motor handler*/
DJI_Motor_Ctrl_t DJI_Motor_uplift;
External_ecd_handler_t uplift_ecd;
External_ecd_handler_t *uplift_ecd_ptr = &uplift_ecd;

// static uint8_t pump1 = 0;
// static uint8_t pump2 = 0;
//float temp_cc_angle;

static void __gimbal_nonforce(void);
static void __gimbal_idle_ctrl(void);
static void __gimbal_rc_ctrl(void);
static void __gimbal_uplift_custom_ctrl(void);
static void __gimbal_any_ctrl(void);

static void __detect_uplift_motor_offline(void);
static void __uplift_move2_subctrl(fp32 UL, uint8_t EN);
// static void __pump_subctrl(void);
// static void __pump_nonctrl(void);

static void __gimbal_move_GGM(void);
static void __gimbal_move_OCSM(void);
void __gimbal_oid_rc_ctrl(void);

/*general handler method*/
/**
 * macro name format:
 *  __<GET/SET>_<MOTOR/JOINT>_<ITEM>(index,[value])
 */
/*自定义控制器*/
#define cc_joint_angle (Custom_Ctrl_get_rx_pack_ptr()->adc_val)
#define cc_key_value (Custom_Ctrl_get_rx_pack_ptr()->key)
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
#define __SET_MOTOR_CTRL_MODE(index, mode) (HANDLER_PTR->motor_ctrl_mode[index] = mode)

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

// oid_set
#define __OID_LIMIT(oid_ptr, value) angle_limit(value, oid_ptr->max_ecd - oid_ptr->min_ecd, 0)
#define __ADD_ROPE_LENGTH(oid_ptr, value)                                                \
  {                                                                                      \
    (HANDLER_PTR->oid_length = __OID_LIMIT(oid_ptr, HANDLER_PTR->oid_length + (value))); \
    (HANDLER_PTR->motor_ctrl_mode[GIMBAL_UPLIFT] = SPEED_LOOP);                          \
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

// #define __BUTTON_PRESS_SWITCH_WRAP(button, mode_var, id, loop_cnt, func) \
//   {                                                                      \
//     static uint8_t press_loop_cnt = 0;                                   \
//     if (button && press_loop_cnt < loop_cnt)                             \
//     {                                                                    \
//       press_loop_cnt++;                                                  \
//     }                                                                    \
//     else if (press_loop_cnt == loop_cnt)                                 \
//     {                                                                    \
//       mode_var = mode_var == id ? 0 : id;                                \
//     }                                                                    \
//     if (button == 0)                                                     \
//     {                                                                    \
//       press_loop_cnt = 0;                                                \
//     }                                                                    \
//     if (mode_var == id)                                                  \
//     {                                                                    \
//       func();                                                            \
//     }                                                                    \
//   }

void gimbal_task_init()
{
  /*基础初始化*/
  __HALT_TICKS_COUNTING();
  __RESET_TICKS();

  /*电机初始化*/
  __SET_MOTOR_INSTANCE(DJI_UL, &DJI_Motor_uplift);
  __SET_MOTOR_TYPE(DJI_UL, DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_uplift, &DJI_CAN1_Bus_ctrl, M3508, 0x205);
  // 多圈计算sp的pid
  DJI_Motor_Speed_PID_init(&DJI_Motor_uplift, PID_POSITION, 21, 0, 0.001, 6000, 500);
  DJI_Motor_Pos_PID_init(&DJI_Motor_uplift, PID_POSITION, 80, 0.01, 0, 1900, 800);

  // oid的pid
  // DJI_Motor_Speed_PID_init(&DJI_Motor_uplift, PID_POSITION, 30, 0.01, 0.001, 6000, 500);
  //  DJI_Motor_Pos_PID_init(&DJI_Motor_uplift, PID_POSITION, 17.4, 0.06, 0, 1900, 800);
  //  DJI_Motor_Oid_PID_init(&uplift_ecd, PID_POSITION, 5.1, 0, 0.002, 5000, 400);

  DJI_Motor_set_reverse(&DJI_Motor_uplift);
  DJI_Motor_uplift.circle_count_flag = 1;
  // DJI_Motor_set_stall_detect(&DJI_Motor_uplift);

  __SET_JOINT_LIMIT(GIMBAL_CAMERA_YAW, 260 - 500, 700 - 500);
  __SET_JOINT_ANGLE(GIMBAL_CAMERA_YAW, 0);
  servo_init(PWM1, 532, 380, 500);
  servo_enable(PWM1);

  __SET_JOINT_LIMIT(GIMBAL_CAMERA_PITCH, 350 - 500, 540 - 500);
  __SET_JOINT_ANGLE(GIMBAL_CAMERA_PITCH, 0);
  servo_init(PWM2, 532, 380, 500);
  servo_enable(PWM2);

  External_can_ecd_init(&uplift_ecd, 0x14A0, 0x0904, OID_ECD, ECD_CAN_COMMUNICATION, 0x04, 0x04);
  __SET_JOINT_LIMIT(GIMBAL_UPLIFT, UL_MIN_ENCODE, UL_MAX_ENCODE);

  for (int i = 0; i < 40; i++)
  {
    osDelay(20);
    gimbal_task_get_feedback();
    __gimbal_nonforce();
  }
  DJI_CANBus_enable_bus(&DJI_CAN1_Bus_ctrl);
}

/**
 * @brief 刷新句柄的反馈值
 * @details 遍历电机控制句柄，针对点击类型刷新反馈
 * @code
 */
void gimbal_task_get_feedback()
{
  uint16_t index;

  /*motor feedback*/
  for (index = 0; index < GIMBAL_MOTOR_COUNT; index++)
  {
    GENERAL_MOTOR_GET_FEEDBACK(__GET_MOTOR_INSTANCE(index),
                               __GET_MOTOR_TYPE(index),
                               &__GET_MOTOR_CURRENT(index),
                               &__GET_MOTOR_SPEED(index),
                               &__GET_MOTOR_ANGLE(index))
  }

  /*joint angle map*/
  __GET_JOINT_ANGLE(GIMBAL_UPLIFT) = UL_MAP_K * __GET_MOTOR_ANGLE(DJI_UL) + UL_MAP_D;
  /*
  __GET_JOINT_ANGLE(index,
    ...
  )
  ...
  */
 //调参使用
  CC_handler.joint_angle[6] = (cc_joint_angle[6] - uplift_custom_controller_D) * uplift_custom_controller_K + UL_MIN_ENCODE;
}

/**
 * @brief 模式状态刷新
 * @details 根据控制器拨杆刷新模式
 */
void gimbal_task_mode_flush()
{
 // static uint8_t nonforce_start_flag = 0;

  static uint8_t last_mode = GIMBAL_MODE_NONFORCE;
  last_mode = HANDLER_PTR->ctrl_mode;

  //  switch(GET_SWITCH())
  //  {
  //    case 1:
  //      //__SET_STRUCT_MODE(GIMBAL_MODE_RC_CTRL);
  //      __SET_STRUCT_MODE(GIMBAL_MODE_RC_CTRL);
  //      break;
  //    case 2:
  //      __SET_STRUCT_MODE(GIMBAL_MODE_RC_CTRL);
  //      break;
  //    case 0:
  //      nonforce_start_flag=1;
  //    default:
  //      __SET_STRUCT_MODE(GIMBAL_MODE_NONFORCE);
  //  }

  /**/
  if (switch_is_mid(get_remote_control_point()->rc.s[1]))
  {
    if (switch_is_down(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(GIMBAL_MODE_IDLE);
    else if (switch_is_mid(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(GIMBAL_MODE_RC_CTRL);
    //__SET_STRUCT_MODE(GIMBAL_OID_RC_CTRL);
    else if (switch_is_up(get_remote_control_point()->rc.s[0]))
      //__SET_STRUCT_MODE(GIMBAL_MODE_UPLIFT_RC_CTRL);
      __SET_STRUCT_MODE(GIMBAL_MODE_IDLE);
  }
  else if (switch_is_up(get_remote_control_point()->rc.s[1]))
  {
    if (switch_is_mid(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(GIMBAL_MODE_IDLE);
    else
      __SET_STRUCT_MODE(GIMBAL_MODE_IDLE);
  }
  else if (switch_is_down(get_remote_control_point()->rc.s[1]))
  {
    if (switch_is_mid(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(GIMBAL_MODE_CUSTOM_CTRL);
    else
      __SET_STRUCT_MODE(GIMBAL_MODE_IDLE);
  }

  if (switch_is_down(get_remote_control_point()->rc.s[1]) && switch_is_down(get_remote_control_point()->rc.s[0]))
  {
    __SET_STRUCT_MODE(GIMBAL_MODE_NONFORCE);
    //nonforce_start_flag = 1;
  }

  if (toe_is_error(DBUSTOE))
  {
    __SET_STRUCT_MODE(GIMBAL_MODE_NONFORCE);
  }

  // /*PUMP*/
  // {
  //   static uint8_t press_loop_cnt = 0;
  //   if ((GET_KEY(KEY_F) || remote_data.fn_1) && press_loop_cnt < 6)
  //   {
  //     press_loop_cnt++;
  //   }
  //   if (press_loop_cnt == 5)
  //   {
  //     pump1 = pump1 == 1 ? 0 : 1;
  //   }
  //   if ((GET_KEY(KEY_F) || remote_data.fn_1) == 0)
  //   {
  //     press_loop_cnt = 0;
  //   }
  // }

  // {
  //   static uint8_t press_loop_cnt = 0;
  //   if ((GET_KEY(KEY_R) || remote_data.fn_2) && press_loop_cnt < 2)
  //   {
  //     press_loop_cnt++;
  //   }
  //   if (press_loop_cnt == 1)
  //   {
  //     pump2 = pump2 == 1 ? 0 : 1;
  //   }
  //   if ((GET_KEY(KEY_R) || remote_data.fn_2) == 0)
  //   {
  //     press_loop_cnt = 0;
  //   }
  // }

  
  /*movement*/
  //if (__GET_STRUCT_MODE() == GIMBAL_MODE_IDLE) 
    if (get_movement() == ONCE_CLICK_SAVE_MINE)
    {
      __SET_STRUCT_MODE(GIMBAL_MODE_SM_CTRL);
    }
    // else
    // {
    //   //__SET_STRUCT_MODE(GIMBAL_MODE_RC_CTRL);//比赛时用到
    //   __SET_STRUCT_MODE(GIMBAL_MODE_IDLE);//测试时放置和机械臂控制干涉
    // }
  //}

  // /*put in the last*/
  // if (GetMatchReady())
  // {
  //   __SET_STRUCT_MODE(GIMBAL_MODE_IDLE);
  // }

  // if (!nonforce_start_flag)
  // {
  //   __SET_STRUCT_MODE(GIMBAL_MODE_NONFORCE);
  // }

  // if (toe_is_error(DBUSTOE) && toe_is_error(CAMERA_TOE))
  // {
  //   __SET_STRUCT_MODE(GIMBAL_MODE_NONFORCE);
  // }

  if (HANDLER_PTR->ctrl_mode == last_mode)
    HANDLER_PTR->mode_switch = 0;
  else
  {
    __RESET_TICKS();
    __HALT_TICKS_COUNTING();
    HANDLER_PTR->mode_switch = 1;
  }
}
/**
 * @brief 设置输出量(电流|速度|位置|力矩)
 * @details 模式控制
 */
void gimbal_task_set_output()
{
  switch (__GET_STRUCT_MODE())
  {
  case GIMBAL_MODE_IDLE:
    __gimbal_idle_ctrl();
    break;
  case GIMBAL_MODE_RC_CTRL:
    __gimbal_rc_ctrl();
    break;
  case GIMBAL_MODE_CUSTOM_CTRL:
    __gimbal_uplift_custom_ctrl();
   break;
  case GIMBAL_MODE_SM_CTRL:
    __gimbal_move_OCSM();
    break;
  case GIMBAL_OID_RC_CTRL:
    __gimbal_oid_rc_ctrl();
    break;
  case GIMBAL_MODE_NONFORCE:
  default:
    __gimbal_nonforce();
  }
  __gimbal_any_ctrl();

  __detect_uplift_motor_offline();
}

/**
 * @brief 控制输出
 * @details 根据电机种类与控制状态设定输出
 */
void gimbal_task_output()
{
  uint16_t index;
  /*joint map to motor state*/
  if (__GET_MOTOR_CTRL_MODE(DJI_UL) == POS_LOOP)
    __SET_MOTOR_ANGLE(DJI_UL, (HANDLER_PTR->joint_angle[DJI_UL] - UL_MAP_D) / UL_MAP_K);

  // 测试抽绳编码器时的代码
  // External_ecd_oid_set_mode(&hfdcan3, 0x01, 0x06, 0x00, 0x00); // 抽绳编码器设置发送指令（暂时测试使用，之后请分层）

  /*motor output*/
  for (index = 0; index < GIMBAL_MOTOR_COUNT; index++)
  {
    GENERAL_MOTOR_SET_OUTPUT(__GET_MOTOR_INSTANCE(index),
                             __GET_MOTOR_TYPE(index),
                             __GET_MOTOR_CTRL_MODE(index),
                             HANDLER_PTR->motor_current[index],
                             HANDLER_PTR->motor_speed[index],
                             HANDLER_PTR->motor_angle[index])
  }
}

void __gimbal_nonforce()
{
  int index;
  for (index = 0; index < GIMBAL_MOTOR_COUNT; index++)
  {
    __SET_JOINT_ANGLE(index, HANDLER_PTR->feedback_joint_angle[index]); // 设置关节输出值为当前关节角度
    __SET_MOTOR_NONFORCE(index);
  }
 // __pump_nonctrl();
}

void __gimbal_idle_ctrl()
{
  __ADD_JOINT_ANGLE(GIMBAL_UPLIFT, 0);
  //__pump_subctrl();
}

void __gimbal_rc_ctrl()
{
  __ADD_JOINT_ANGLE(GIMBAL_UPLIFT, RC_CTRL_PTR->rc.ch[1] * 0.00024f);

  //__pump_subctrl();

  if (GET_KEY(KEY_C))
    __ADD_JOINT_ANGLE(GIMBAL_UPLIFT, -660 * 0.00024f);
  if (GET_KEY(KEY_V))
    __ADD_JOINT_ANGLE(GIMBAL_UPLIFT, 660 * 0.00024f);
  // if (GET_KEY(KEY_SHIFT) && HANDLER_PTR->joint_angle[GIMBAL_UPLIFT] > UL_MAX_ENCODE / 4)
  //   __ADD_JOINT_ANGLE(GIMBAL_UPLIFT, -660 * 0.00024f);
}

// 实测发现抽绳编码器在使用过程中存在抖动，不太好用
void __gimbal_oid_rc_ctrl()
{
  __ADD_ROPE_LENGTH(uplift_ecd_ptr, (int32_t)(RC_CTRL_PTR->rc.ch[1] * 0.002f));
  HANDLER_PTR->motor_speed[GIMBAL_UPLIFT] = __ADD_OID_LENGTH_OUTPUT(HANDLER_PTR->oid_length, &uplift_ecd);
}


void __gimbal_uplift_custom_ctrl(void)
{
  // static float middle_pos = UL_MIN_ENCODE;
  // if (__IS_MODE_SWITCHED())
  // {
  //   middle_pos = __GET_JOINT_ANGLE(GIMBAL_UPLIFT);
  // }

  // //__pump_subctrl();
  // if (-300 > RC_CTRL_PTR->rc.ch[1] || RC_CTRL_PTR->rc.ch[1] > 300)
  // {
  //   middle_pos += RC_CTRL_PTR->rc.ch[1] * 0.00048f;
  // }

  // if (-300 > GET_CH_VALUE(1) || GET_CH_VALUE(1) > 300)
  // {
  //   middle_pos += GET_CH_VALUE(1) * 0.00048f;
  // }

  // if (GET_KEY(KEY_C))
  //   middle_pos -= 660 * 0.00048f;
  // if (GET_KEY(KEY_V))
  //   middle_pos += 660 * 0.00048f;

  // if (middle_pos < UL_MIN_ENCODE)
  //   middle_pos = UL_MIN_ENCODE;
  // else if (middle_pos > UL_MAX_ENCODE)
  //   middle_pos = UL_MAX_ENCODE;

  if (GET_KEY(KEY_C))
    __ADD_JOINT_ANGLE(GIMBAL_UPLIFT, -660 * 0.00024f);
  if (GET_KEY(KEY_V))
    __ADD_JOINT_ANGLE(GIMBAL_UPLIFT, 660 * 0.00024f);
  //__uplift_move2_subctrl(middle_pos + (UL_MAX_ENCODE - UL_MIN_ENCODE) / 2 * (cc_joint_angle[4] - 0.5f), 0x01);
  __uplift_move2_subctrl((cc_joint_angle[6] - uplift_custom_controller_D) * uplift_custom_controller_K + UL_MIN_ENCODE, 0x01);


  // __ADD_JOINT_ANGLE(GIMBAL_CAMERA_YAW, (float)-remote_data.mouse_x / 50.0f);//yaw轴不需要动
  // servo_set_offset(0, HANDLER_PTR->joint_angle[GIMBAL_CAMERA_YAW]);//尝试用于一键存矿
}


void __gimbal_any_ctrl(void)//可设置为舵机运动
{

  if (__GET_STRUCT_MODE() != GIMBAL_MODE_IDLE)
  {
    __ADD_JOINT_ANGLE(GIMBAL_CAMERA_PITCH, (float)remote_data.mouse_y / 120.0f);
    servo_set_offset(1, HANDLER_PTR->joint_angle[GIMBAL_CAMERA_PITCH]);
  }
}

// void __pump_subctrl()
// {
// if(cc_key_value.k1 || GET_KEY(KEY_F))
// {
//   pump=PUMP_PULL;
//   __RESET_TICKS();
//   __HALT_TICKS_COUNTING();
// }
// else
// {
//   __HALT_TICKS_COUNTING();
//   if(__GET_TICKS()<500)
//   {
//     pump=PUMP_PULL;
//     __HOLD_TICKS_COUNTING();
//   }
//   else
//   {
//     pump=PUMP_RESET;
//   }
// }

// if(RC_CTRL_PTR->rc.ch[4]>660/3*2)
// {
//   if(pump==PUMP_PULL)
//     pump=PUMP_RESET;
//   else if(pump==PUMP_RESET)
//     pump=PUMP_PULL;
// }
// else if(RC_CTRL_PTR->rc.ch[4]<-660/3*2)
// {
//   pump=PUMP_PUSH;
// }
// else
// {
//   if(pump==PUMP_PUSH)
//   {
//     pump=PUMP_RESET;
//   }
// }

//   switch (pump1)
//   {
//   case PUMP_PULL:
//     PUMP1_ON();
//     break;
//   case PUMP_RESET:
//   default:
//     PUMP1_OFF();
//     break;
//   }

//   switch (pump2)
//   {
//   case PUMP_PULL:
//     PUMP2_ON();
//     break;
//   case PUMP_RESET:
//   default:
//     PUMP2_OFF();
//     break;
//   }
// }

// void __pump_nonctrl(void)
// {
//   PUMP1_OFF();
//   PUMP2_OFF();
// }

void __detect_uplift_motor_offline(void) 
{
  // 掉电检测

  if (toe_is_error(TOE_UPLIFT))
  {
    HANDLER_PTR->motor_ctrl_mode[GIMBAL_UPLIFT] = OFFLINE;
  }
  else
  {
    __CLEAR_MOTOR_OFFLINE(GIMBAL_UPLIFT);
  }

  
  if (HANDLER_PTR->motor_ctrl_mode[GIMBAL_UPLIFT] == OFFLINE)
  {
    __SET_MOTOR_OFFLINE(GIMBAL_UPLIFT);
  }
}

void __uplift_move2_subctrl(fp32 UL, uint8_t EN)
{
  if (EN)
  {
    if (ABS(UL - HANDLER_PTR->joint_angle[GIMBAL_UPLIFT]) > 1.0f)
    {
      __ADD_JOINT_ANGLE(GIMBAL_UPLIFT, UL > HANDLER_PTR->joint_angle[GIMBAL_UPLIFT] ? 0.2f : -0.2f);
    }
    else
    {
      __SET_JOINT_ANGLE(GIMBAL_UPLIFT, HANDLER_PTR->feedback_joint_angle[GIMBAL_UPLIFT]);
    }
  }
}

void __gimbal_move_GGM(void)
{
  if (get_step() == GGM_uplift_to_higher)
  {
    if (is_angle_around(__GET_JOINT_ANGLE(GIMBAL_UPLIFT), GGM_STEP1_HEIGHT, 5))
    {
      next_step();
    }
    else
    {
      __SET_JOINT_ANGLE(GIMBAL_UPLIFT, GGM_STEP1_HEIGHT);
    }
  }
  else if (get_step() == GGM_uplift_down)
  {
    //pump1 = 1;
    if (is_angle_around(__GET_JOINT_ANGLE(GIMBAL_UPLIFT), GGM_STEP3_HEIGHT, 5))
    {
      next_step();
    }
    else
    {
      __SET_JOINT_ANGLE(GIMBAL_UPLIFT, GGM_STEP3_HEIGHT);
    }
  }
  else if (get_step() == GGM_complete)
  {
    if (is_angle_around(__GET_JOINT_ANGLE(GIMBAL_UPLIFT), GGM_CPLT_HEIGHT, 5))
    {
      set_movement(0);
    }
    else
    {
      __SET_JOINT_ANGLE(GIMBAL_UPLIFT, GGM_CPLT_HEIGHT);
    }
  }
}

void __gimbal_move_OCSM(void)
{
  if (movement.hand_move_out_flag == 1)
  {
    set_movement(0); // 退出固定动作
    __SET_STRUCT_MODE(GIMBAL_MODE_IDLE);
    movement.hand_move_out_flag = 0;
  }
  if (get_step() == SM_uplift_to_pos)
  {
    if (is_angle_around(__GET_JOINT_ANGLE(GIMBAL_UPLIFT), SM_STEP1_HEIGHT, 5.0))
    {
      next_step();
    }
    else
    {
      __uplift_move2_subctrl(SM_STEP1_HEIGHT, 1);
    }
  }

  if (get_step() == SM_uplift_down)
  {
    if (is_angle_around(__GET_JOINT_ANGLE(GIMBAL_UPLIFT), SM_STEP4_HEIGHT, 5.0))
    {
      next_step();
    }
    else
    {
      //__SET_JOINT_ANGLE(GIMBAL_UPLIFT, SM_STEP3_HEIGHT);
      __uplift_move2_subctrl(SM_STEP4_HEIGHT, 1);
    }
  }

  if (get_step() == SM_complete)
  {
    if (is_angle_around(__GET_JOINT_ANGLE(GIMBAL_UPLIFT), SM_STEP4_HEIGHT, 1))
    {
      set_movement(0);//0没有规定步骤，即为设置退出一键模式
    }
    else
    {
      __SET_JOINT_ANGLE(GIMBAL_UPLIFT, SM_STEP4_HEIGHT);
    }
  }

  //__pump_subctrl();
}

#undef HANDLER
#undef HANDLER_PTR
