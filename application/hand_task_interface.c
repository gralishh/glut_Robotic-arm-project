/**
 * @brief
 * @attention lockup的逻辑大抵没有大🐱饼
 * @note hand_task内的角度单位为rad
 */
#include <string.h>
#include "hand_task_interface.h"
#include "struct_typedef.h"
#include "hand_task.h"
#include "general_motor_module.h"
#include "DJI_motor_canbus.h"
#include "remote_control.h"
#include "angle_process.h"
#include "detect_task.h"
#include "cmsis_os2.h"
#include "ws2812.h"
#include "Custom_ctrl.h"
#include "bsp_buzzer.h"
#include "general_movement.h"

#include "main.h"
#include "Vofa.h"

#define HANDLER hand_task_handler
#define HANDLER_PTR hand_task_handler_ptr
#define RC_CTRL_PTR (get_remote_control_point())

/*extern*/
extern FDCAN_HandleTypeDef hfdcan2;

/*ABS*/
#define ABS(X) ((X) > 0 ? (X) : -(X))
/*global macro variable*/
#define HAND_CTRL_CAN
// joint mapping parameter
// #define J1_MAP_K   0.167 /*for m8010*/
#define J1_MAP_K (3.14f / 180.0f)
#define J1_MAP_D 0
#define J2_MAP_K 1.0f
#define J2_MAP_D 0
#define J3_MAP_K (-3.14f / 42.4f) /*old value(-1.0f/19.2f)*/
#define J3_MAP_D (-1.668)
#define PITCH_MAP_K (PI / 2 / (135 - 50))
#define PITCH_MAP_D (-50 * PITCH_MAP_K)
#define ROLL_MAP_K (PITCH_MAP_K / 2)
#define ROLL_MAP_D 0
// controller sensity(degree per loop)
#define J1_CTRL_SEN 0
#define J2_CTRL_SEN 0
#define J3_CTRL_SEN 0
#define PITCH_CTRL_SEN 0
#define ROLL_CTRL_SEN 0

#define J1_EN (0x01 << 0)
#define J2_EN (0x01 << 1)
#define J3_EN (0x01 << 2)
#define J4_EN (0x01 << 3)
#define J5_EN (0x01 << 4)

/*global motor handler*/
extern M8010_motor_t joint1_motor;
extern AK_Joint_Motor_t AK70_10_motor;
extern Joint_Motor_t DM_Motor_J2;
DJI_Motor_Ctrl_t DJI_Motor_J3;
DJI_Motor_Ctrl_t DJI_Motor_headendL;
DJI_Motor_Ctrl_t DJI_Motor_headendR;

/*global variable*/
static fp32 J1_D = 0;
static fp32 J3_D = 0;

/*custom controller remap(custom controller -> joint_angle)*/
float custom_controller_K[5] = {1, 1, 1, -1, 1};
float custom_controller_D[5] = {0, 0, 0, 1.56, 0};

static void __hand_nonforce(void);
static void __hand_idle_ctrl(void);
static void __hand_rc_ctrl(void);
static void __hand_custom_ctrl(void);
// static void __hand_pose_ctrl(void);
static void __hand_gold_catch_ctrl(void);

static void __hand_catch_ground(void);

static void __hand_move2_subctrl(fp32 J1, fp32 J2, fp32 J3, fp32 J4, fp32 J5, uint8_t EN);

static uint8_t __hand_J1_init(uint8_t);
static uint8_t __hand_J2_init(void);
static uint8_t __hand_J3_init(void);
static uint8_t __hand_pitch_pos_init(uint8_t);

static void __hand_move_GGM(void);
static void __hand_move_SM(void);

void __hand_move_reset(void);
void hand_J1_reset(void);
void hand_J2_reset(void);
void hand_J3_reset(void);
void hand_pitch_reset(void);

/*general handler method*/
/**
 * macro name format:
 *  __<GET/SET>_<MOTOR/JOINT>_<ITEM>(index[,value])
 */
/*自定义控制器*/
#define cc_joint_angle (Custom_Ctrl_get_rx_pack_ptr()->adc_val)
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
#define __IS_JOINT_AROUND(INDEX, ANGLE) (is_angle_around(ANGLE, __GET_JOINT_ANGLE(INDEX), 0.05f))

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
#define __RESET_JOINT_LIMIT(index) __SET_JOINT_LIMIT(index, 0, 0)
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
#define __GET_JOINT_MAX_LIM(index) (HANDLER_PTR->max_joint_angle[index])

/*时间控制*/
#define __RESET_TICKS() (HANDLER_PTR->tick = 0)
#define __HALT_TICKS_COUNTING() (HANDLER_PTR->tick_count_halt = 1)
#define __HOLD_TICKS_COUNTING() (HANDLER_PTR->tick_count_halt = 0)
#define __IS_TIMER_HALT() (1 == HANDLER_PTR->tick_count_halt)
#define __GET_TICKS() (HANDLER_PTR->tick)
// unit:seconds
#define __GET_TICKS_TIME() (HANDLER_PTR->tick * 1)
#define __GET_PROCESS_PERCENTAGE(PROCESS_TIME) (__GET_TICKS_TIME() / PROCESS_TIME)
#define __GET_TICKS_STACK(index) (HANDLER_PTR->tick_stack[index])
#define __RECORD_TICKS(index) (HANDLER_PTR->tick_stack[index] = __GET_TICKS_TIME())
#define __RESET_RECORD_TICKS(index) (HANDLER_PTR->tick_stack[index] = 0)
#define __IS_MODE_SWITCHED() (1 == HANDLER_PTR->mode_switch)

#define __BUTTON_PRESS_SWITCH_WRAP(button, mode_var, id, loop_cnt, func) \
  {                                                                      \
    static uint8_t press_loop_cnt = 0;                                   \
    if (button && press_loop_cnt < loop_cnt)                             \
    {                                                                    \
      press_loop_cnt++;                                                  \
    }                                                                    \
    else if (press_loop_cnt == loop_cnt)                                 \
    {                                                                    \
      mode_var = mode_var == id ? 0 : id;                                \
    }                                                                    \
    if (button == 0)                                                     \
    {                                                                    \
      press_loop_cnt = 0;                                                \
    }                                                                    \
    if (mode_var == id)                                                  \
    {                                                                    \
      func();                                                            \
    }                                                                    \
  } // 未理解，可酌情删除

void hand_task_init()
{
  osDelay(1000);

  /*基础初始化*/
  __HALT_TICKS_COUNTING();
  __RESET_TICKS();

  /*电机初始化*/
  // J1[M8010]
  //__SET_MOTOR_INSTANCE(M8010_J1,&joint1_motor);
  //__SET_MOTOR_TYPE(M8010_J1,M8010_MOTOR);
  // M8010_motor_init(&joint1_motor,3,0.76,0.088);
  // J1[AK]
  __SET_MOTOR_INSTANCE(AK_J1, &AK70_10_motor);
  __SET_MOTOR_TYPE(AK_J1, AK_MOTOR);
  AK_joint_motor_init(&AK70_10_motor, 93);
  AK_joint_motor_enable(&AK70_10_motor);

  // J2
  __SET_MOTOR_INSTANCE(DM_J2, &DM_Motor_J2);
  __SET_MOTOR_TYPE(DM_J2, M4310_MOTOR);
  joint_motor_init(&DM_Motor_J2, 1, POS_MODE, 1.0, 1.0);

  // J3
  __SET_MOTOR_INSTANCE(DJI_J3, &DJI_Motor_J3);
  __SET_MOTOR_TYPE(DJI_J3, DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_J3, &DJI_CAN2_Bus_ctrl, M3508, 0x204);
  // DJI_Motor_set_angle_limit()
  // DJI_Motor_set_speed_limit()
  DJI_Motor_Speed_PID_init(&DJI_Motor_J3, PID_POSITION, 22, 0.000, 0.05, 6000.000, 800);
  DJI_Motor_Pos_PID_init(&DJI_Motor_J3, PID_POSITION, 55, 0.0, 0.0, 200, 100);
  // DJI_Motor_set_sum_angle(&DJI_Motor_J3);
  DJI_Motor_set_multiple_circle_angle(&DJI_Motor_J3);

  // Headend_L
  __SET_MOTOR_INSTANCE(DJI_HE_L, &DJI_Motor_headendL);
  __SET_MOTOR_TYPE(DJI_HE_L, DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_headendL, &DJI_CAN2_Bus_ctrl, M2006, 0x201);
  DJI_Motor_Speed_PID_init(&DJI_Motor_headendL, PID_POSITION, 22, 0.001, 0, 5500, 1000);
  DJI_Motor_Pos_PID_init(&DJI_Motor_headendL, PID_POSITION, 75, 0, 0, 2000, 1000);
  // DJI_Motor_set_offline_detect(&DJI_Motor_headendL,8,0);
  // DJI_Motor_set_sum_angle(&DJI_Motor_headendL);
  DJI_Motor_set_multiple_circle_angle(&DJI_Motor_headendL);

  // Headend_R
  __SET_MOTOR_INSTANCE(DJI_HE_R, &DJI_Motor_headendR);
  __SET_MOTOR_TYPE(DJI_HE_R, DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_headendR, &DJI_CAN2_Bus_ctrl, M2006, 0x208);
  DJI_Motor_Speed_PID_init(&DJI_Motor_headendR, PID_POSITION, 10, 0.001, 0, 5500, 1000);
  DJI_Motor_Pos_PID_init(&DJI_Motor_headendR, PID_POSITION, 75, 0, 0, 2000, 0);
  // DJI_Motor_set_offline_detect(&DJI_Motor_headendR,8,0);
  // DJI_Motor_set_sum_angle(&DJI_Motor_headendR);
  DJI_Motor_set_multiple_circle_angle(&DJI_Motor_headendR);

  // joint_pitch
  __SET_JOINT_LIMIT(HAND_PITCH, 5 * PITCH_MAP_K + PITCH_MAP_D, 150.0f * PITCH_MAP_K + PITCH_MAP_D);
  __SET_JOINT_ANGLE(HAND_PITCH, PITCH_MAP_D);

  // limit
  __SET_JOINT_LIMIT(HAND_J1, -3.10, 0);
  __SET_JOINT_LIMIT(HAND_J2, -3.14 / 7 * 5, 3.14 / 7 * 5);
  __SET_JOINT_LIMIT(HAND_J3, -2, 2);

  while (
      toe_is_error(TOE_HE_L) ||
      toe_is_error(TOE_HE_R) ||
      toe_is_error(TOE_J1) ||
      toe_is_error(TOE_J2) ||
      toe_is_error(TOE_J3))
    ;
  __CLEAR_MOTOR_OFFLINE(AK_J1);
  __CLEAR_MOTOR_OFFLINE(DM_J2);
  __CLEAR_MOTOR_OFFLINE(DJI_J3);
  __CLEAR_MOTOR_OFFLINE(DJI_HE_L);
  __CLEAR_MOTOR_OFFLINE(DJI_HE_R);

  osDelay(20);
  hand_task_get_feedback();

  DJI_CANBus_enable_bus(&DJI_CAN2_Bus_ctrl);
  __hand_pitch_pos_init(1);
  for (int i = 0; i < 40; i++)
  {
    hand_task_get_feedback();
    __hand_nonforce();
    if (__hand_pitch_pos_init(0))
    {
      break;
    }
    hand_task_output();
    osDelay(1);
  }

  while (!__hand_J3_init())
  {
    hand_task_output();
    osDelay(1);
    hand_task_get_feedback();
    __hand_nonforce();
  }

  __hand_nonforce();
  while (!__hand_J2_init())
    ;
  while (!__IS_JOINT_AROUND(HAND_J2, -1.9))
  {
    hand_task_get_feedback();
    __hand_move2_subctrl(0.0f, -1.9f, 0.0f, 0.0f, 0.0f, J2_EN);
    hand_task_output();
    osDelay(1);
  }
  __hand_nonforce();
  hand_task_output();

  __hand_J1_init(1);
  do
  {
    hand_task_get_feedback();
    osDelay(1);
  } while (!__hand_J1_init(0));

  osDelay(10);
  hand_task_get_feedback();
  __hand_nonforce();
  WS2812_Ctrl(30, 100, 50);
  // buzzer_off();
}

/**
 * @brief 刷新句柄的反馈值
 * @details 遍历电机控制句柄，针对点击类型刷新反馈
 * @code
 */
void hand_task_get_feedback()
{
  uint16_t index;

  /*motor feedback*/
  for (index = 0; index < HAND_MOTOR_COUNT; index++)
  {
    GENERAL_MOTOR_GET_FEEDBACK(__GET_MOTOR_INSTANCE(index),
                               __GET_MOTOR_TYPE(index),
                               &__GET_MOTOR_CURRENT(index),
                               &__GET_MOTOR_SPEED(index),
                               &__GET_MOTOR_ANGLE(index))
  }

  /*joint angle map*/
  //__GET_JOINT_ANGLE(HAND_J1)=J1_MAP_K*__GET_MOTOR_ANGLE(M8010_J1) +J1_MAP_D;
  __GET_JOINT_ANGLE(HAND_J1) = J1_MAP_K * __GET_MOTOR_ANGLE(AK_J1) + J1_MAP_D;
  __GET_JOINT_ANGLE(HAND_J2) = J2_MAP_K * __GET_MOTOR_ANGLE(DM_J2) + J2_MAP_D;
  __GET_JOINT_ANGLE(HAND_J3) = J3_MAP_K * __GET_MOTOR_ANGLE(DJI_J3) + J3_MAP_D;
  __GET_JOINT_ANGLE(HAND_PITCH) = HANDLER_PTR->joint_angle[HAND_PITCH];
  __GET_JOINT_ANGLE(HAND_ROLL) = HANDLER_PTR->joint_angle[HAND_ROLL];
  // 已知电机角度 theta_L 和 theta_R 时，计算关节角度：
  __GET_JOINT_ANGLE(HAND_PITCH) = PITCH_MAP_D + 0.5f * PITCH_MAP_K * (__GET_MOTOR_ANGLE(DJI_HE_R) - __GET_MOTOR_ANGLE(DJI_HE_L));
  __GET_JOINT_ANGLE(HAND_ROLL) = ROLL_MAP_D + 0.5f * ROLL_MAP_K * (__GET_MOTOR_ANGLE(DJI_HE_R) + __GET_MOTOR_ANGLE(DJI_HE_L));
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
void hand_task_mode_flush()
{
  static uint8_t last_mode = HAND_MODE_NONFORCE;
  last_mode = HANDLER_PTR->ctrl_mode;

  //  switch(GET_SWITCH())
  //  {
  //    case 1:
  //      //__SET_STRUCT_MODE(HAND_MODE_RC_CTRL);
  //      __SET_STRUCT_MODE(HAND_MODE_IDLE);
  //      break;
  //    case 2:
  //      __SET_STRUCT_MODE(HAND_MODE_CUSTOM_CTRL);
  //      break;
  //    case 0:
  //    default:
  //      __SET_STRUCT_MODE(HAND_MODE_NONFORCE);
  //  }

  if (switch_is_up(get_remote_control_point()->rc.s[1]))
  {
    if (switch_is_down(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(HAND_MODE_IDLE);
    else if (switch_is_mid(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(HAND_MODE_RC_CTRL);
    else if (switch_is_up(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(HAND_MODE_CUSTOM_CTRL);
      //__SET_STRUCT_MODE(HAND_MODE_IDLE);
  }
  else if (switch_is_mid(get_remote_control_point()->rc.s[1]))
  {
    if (switch_is_up(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(HAND_MODE_IDLE);
    else
      __SET_STRUCT_MODE(HAND_MODE_IDLE);
  }
  else
  {
    __SET_STRUCT_MODE(HAND_MODE_IDLE);
  }

  static uint8_t mode_var = 0;//没用
  if (__GET_STRUCT_MODE() == HAND_MODE_IDLE && !GetMatchReady())
  {
    //__BUTTON_PRESS_SWITCH_WRAP(GET_KEY(KEY_Z),mode_var,1,10,__hand_catch_ground);
    //__BUTTON_PRESS_SWITCH_WRAP
    // if(GET_KEY(KEY_Z))
    //  set_movement(GGM);
    if (GET_KEY(KEY_Z))
      set_movement(SM);
    if (GET_KEY(KEY_G))
      __hand_pitch_pos_init(1);
    if (remote_data.trigger)
      __hand_rc_ctrl();

    if (get_movement() == SM)
    {
      __SET_STRUCT_MODE(HAND_MODE_SM_CTRL);
    }
    else
    {
      __SET_STRUCT_MODE(HAND_MODE_IDLE);
    }
  }
  else
  {
    mode_var = 0;
  }

  if (GetMatchReady())
  {
    __SET_STRUCT_MODE(HAND_MODE_IDLE);
  }

  if (toe_is_error(DBUSTOE) && toe_is_error(CAMERA_TOE))
  {
    __SET_STRUCT_MODE(HAND_MODE_NONFORCE);
  }

  if (HANDLER_PTR->ctrl_mode == last_mode)
    HANDLER_PTR->mode_switch = 0;
  else
  {
    __RESET_TICKS();
    __HALT_TICKS_COUNTING();
    HANDLER_PTR->mode_switch = 1;
    set_movement(0);
  }
}
/**
 * @brief 设置输出量(电流|速度|位置|力矩)
 * @details 模式控制
 */
void hand_task_set_output()
{
  switch (__GET_STRUCT_MODE())
  {
  case HAND_MODE_IDLE:
    __hand_idle_ctrl();
    break;
  case HAND_MODE_RC_CTRL:
    __hand_rc_ctrl();
    break;
  // case HAND_MODE_POSE_CTRL:
  //   __hand_pose_ctrl();
  //   break;
  case HAND_MODE_CUSTOM_CTRL:
    __hand_custom_ctrl();
    break;
  case HAND_MODE_GOLE_CATCH_CTRL:
    __hand_gold_catch_ctrl();
    break;
  case HAND_MODE_SM_CTRL:
    __hand_move_SM();
    break;
  case HAND_MODE_RESET_CTRL:
    __hand_move_reset();
    break;
  case HAND_MODE_NONFORCE:
  default:
    __hand_nonforce();
  }

  /*掉电检测*/
  if (toe_is_error(TOE_HE_L) || __IS_MOTOR_OFFLINE(DJI_HE_L))
  {
    __SET_MOTOR_CTRL_MODE(DJI_HE_L, OFFLINE);
    __SET_MOTOR_OFFLINE(DJI_HE_L);
  }
  if (toe_is_error(TOE_HE_R) || __IS_MOTOR_OFFLINE(DJI_HE_R))
  {
    __SET_MOTOR_CTRL_MODE(DJI_HE_R, OFFLINE);
    __SET_MOTOR_OFFLINE(DJI_HE_R);
  }
  if (toe_is_error(TOE_J1) || __IS_MOTOR_OFFLINE(AK_J1))
  {
    __SET_MOTOR_CTRL_MODE(AK_J1, OFFLINE);
    __SET_MOTOR_OFFLINE(AK_J1);
  }
  if (toe_is_error(TOE_J2) || __IS_MOTOR_OFFLINE(DM_J2))
  {
    __SET_MOTOR_CTRL_MODE(DM_J2, OFFLINE);
    __SET_MOTOR_OFFLINE(DM_J2);
  }
  if (toe_is_error(TOE_J3) || __IS_MOTOR_OFFLINE(DJI_J3))
  {
    __SET_MOTOR_CTRL_MODE(DJI_J3, OFFLINE);
    __SET_MOTOR_OFFLINE(DM_J2);
  }
//加个按键
 // __hand_move_reset();

  if (toe_is_error(TOE_HE_L) &&
      toe_is_error(TOE_HE_R) &&
      toe_is_error(TOE_J1) &&
      toe_is_error(TOE_J2) &&
      toe_is_error(TOE_J3))
  {
    hand_task_init();
  }
}

/**
 * @brief 控制输出
 * @details 根据电机种类与控制状态设定输出
 */
void hand_task_output()
{
  uint16_t index;

  __CLEAR_MOTOR_OFFLINE(DJI_HE_L);
  __CLEAR_MOTOR_OFFLINE(DJI_HE_R);

  /*joint map to motor state*/
  // if(__GET_MOTOR_CTRL_MODE(M8010_J1)==POS_LOOP)
  //   __SET_MOTOR_ANGLE(M8010_J1,(HANDLER_PTR->joint_angle[HAND_J1]-J1_MAP_D)/J1_MAP_K);
  if (__GET_MOTOR_CTRL_MODE(AK_J1) == POS_LOOP)
    __SET_MOTOR_ANGLE(AK_J1, (HANDLER_PTR->joint_angle[HAND_J1] - J1_MAP_D) / J1_MAP_K);
  if (__GET_MOTOR_CTRL_MODE(DM_J2) == POS_LOOP)
    __SET_MOTOR_ANGLE(DM_J2, (HANDLER_PTR->joint_angle[HAND_J2] - J2_MAP_D) / J2_MAP_K);
  if (__GET_MOTOR_CTRL_MODE(DJI_J3) == POS_LOOP)
    __SET_MOTOR_ANGLE(DJI_J3, (HANDLER_PTR->joint_angle[HAND_J3] - J3_MAP_D) / J3_MAP_K);
  if (__GET_MOTOR_CTRL_MODE(DJI_HE_L) == POS_LOOP)
    __SET_MOTOR_ANGLE(DJI_HE_L, -(HANDLER_PTR->joint_angle[HAND_PITCH] - PITCH_MAP_D) / PITCH_MAP_K +
                                    (HANDLER_PTR->joint_angle[HAND_ROLL] - ROLL_MAP_D) / ROLL_MAP_K);
  if (__GET_MOTOR_CTRL_MODE(DJI_HE_R) == POS_LOOP)
    __SET_MOTOR_ANGLE(DJI_HE_R, (HANDLER_PTR->joint_angle[HAND_PITCH] - PITCH_MAP_D) / PITCH_MAP_K +
                                    (HANDLER_PTR->joint_angle[HAND_ROLL] - ROLL_MAP_D) / ROLL_MAP_K);

  /*motor output*/
  for (index = 0; index < HAND_MOTOR_COUNT; index++)
  {
    GENERAL_MOTOR_SET_OUTPUT(__GET_MOTOR_INSTANCE(index),
                             __GET_MOTOR_TYPE(index),
                             __GET_MOTOR_CTRL_MODE(index),
                             HANDLER_PTR->motor_current[index],
                             HANDLER_PTR->motor_speed[index],
                             HANDLER_PTR->motor_angle[index]);
  }
}

void __hand_nonforce()
{
  int index;
  for (index = 0; index < HAND_JOINT_COUNT; index++)
  {
    __SET_JOINT_ANGLE(index, HANDLER_PTR->feedback_joint_angle[index]); // 设置关节输出值为当前关节角度
  }

  for (index = 0; index < HAND_MOTOR_COUNT; index++)
  {
    __SET_MOTOR_NONFORCE(index);
  }
}

void __hand_idle_ctrl()
{
  //__ADD_MOTOR_ANGLE(DJI_HE_L,0);
  //__ADD_MOTOR_ANGLE(DJI_HE_R,0);
  __ADD_JOINT_ANGLE(HAND_ROLL, 0);
  __ADD_JOINT_ANGLE(HAND_PITCH, 0);
  __ADD_JOINT_ANGLE(HAND_J1, 0);
  __ADD_JOINT_ANGLE(HAND_J2, 0);
  __ADD_JOINT_ANGLE(HAND_J3, 0);

  __hand_pitch_pos_init(0);
}

void __hand_rc_ctrl()
{
  //__ADD_MOTOR_ANGLE(DJI_HE_L,RC_CTRL_PTR->rc.ch[3]*0.00005f+RC_CTRL_PTR->rc.ch[1]*0.0001f);
  //__ADD_MOTOR_ANGLE(DJI_HE_R,RC_CTRL_PTR->rc.ch[3]*0.00005f-RC_CTRL_PTR->rc.ch[1]*0.0001f);
  __ADD_JOINT_ANGLE(HAND_PITCH, RC_CTRL_PTR->rc.ch[1] * 0.0001f * PITCH_MAP_K);
  __ADD_JOINT_ANGLE(HAND_ROLL, RC_CTRL_PTR->rc.ch[3] * 0.0001f * ROLL_MAP_K);
  __ADD_JOINT_ANGLE(HAND_J1, -RC_CTRL_PTR->rc.ch[2] * 0.0000007f);
  __ADD_JOINT_ANGLE(HAND_J2, -RC_CTRL_PTR->rc.ch[0] * 0.0000015f);
  __ADD_JOINT_ANGLE(HAND_J3, -RC_CTRL_PTR->rc.ch[4] * 0.000025f * J3_MAP_K);

  //  __ADD_JOINT_ANGLE(HAND_PITCH, GET_CH_VALUE(1)*0.0001f*PITCH_MAP_K);
  //  __ADD_JOINT_ANGLE(HAND_ROLL , GET_CH_VALUE(2)*0.0001f*ROLL_MAP_K);
  //  __ADD_JOINT_ANGLE(HAND_J1   ,-GET_CH_VALUE(3)*0.0000007f);
  //  __ADD_JOINT_ANGLE(HAND_J2   ,-GET_CH_VALUE(0)*0.0000015f);
  //  __ADD_JOINT_ANGLE(HAND_J3   ,GET_WHEEL_VALUE()*0.000025f*J3_MAP_K);
}

void __hand_custom_ctrl(void)
{
  __hand_move2_subctrl(
      (cc_joint_angle[0] - PI / 2 - custom_controller_D[0]) * custom_controller_K[0],
      (cc_joint_angle[1] - custom_controller_D[1]) * custom_controller_K[1],
      (cc_joint_angle[2] - custom_controller_D[2]) * custom_controller_K[2],
      (cc_joint_angle[3] - custom_controller_D[3]) * custom_controller_K[3], -cc_joint_angle[5], J1_EN | J2_EN | J3_EN | J4_EN | J5_EN);
}

/**/
void __hand_move2_subctrl(fp32 J1, fp32 J2, fp32 J3, fp32 J4, fp32 J5, uint8_t EN)
{
  if (EN & J5_EN)
  {
    if (ABS(J5 - HANDLER_PTR->joint_angle[HAND_ROLL]) > 0.020f)
    {
      __ADD_JOINT_ANGLE(HAND_ROLL,
                        J5 > HANDLER_PTR->joint_angle[HAND_ROLL] ? 0.001f : -0.001f);
    }
    else
    {
      __SET_JOINT_ANGLE(HAND_ROLL, J5);
    }
  }

  if (EN & J4_EN)
  {
    if (ABS(J4 - HANDLER_PTR->joint_angle[HAND_PITCH]) > 0.020f)
    {
      __ADD_JOINT_ANGLE(HAND_PITCH,
                        J4 > HANDLER_PTR->joint_angle[HAND_PITCH] ? 0.008f : -0.008);
    }
  }

  if (EN & J1_EN)
  {
    if (ABS(J1 - HANDLER_PTR->joint_angle[HAND_J1]) > 0.03f)
    {
      __SET_JOINT_ANGLE(HAND_J1, J1);
      //__ADD_JOINT_ANGLE(HAND_J1,
      //  J1>HANDLER_PTR->joint_angle[HAND_J1]?
      //     0.00105f:
      //    -0.00105f);
      // __ADD_JOINT_ANGLE(HAND_J1,0.002f*(J1-HANDLER_PTR->joint_angle[HAND_J1]));
    }
    else if (ABS(J1 - HANDLER_PTR->joint_angle[HAND_J1]) > 0.01f)
    {
      //__ADD_JOINT_ANGLE(HAND_J1,
      //  J1>HANDLER_PTR->joint_angle[HAND_J1]?
      //     0.0014f:
      //    -0.0014f);
      // __ADD_JOINT_ANGLE(HAND_J1,0.002f*(J1-HANDLER_PTR->joint_angle[HAND_J1]));
    }
  }

  if (EN & J2_EN)
  {
    if (ABS(J2 - HANDLER_PTR->joint_angle[HAND_J2]) > 0.08f)
    {
      __ADD_JOINT_ANGLE(HAND_J2,
                        J2 > HANDLER_PTR->joint_angle[HAND_J2] ? 0.0023f : -0.0023f);
      //__ADD_JOINT_ANGLE(HAND_J2,0.002f*(J2-HANDLER_PTR->joint_angle[HAND_J2]));
    }
    else if (ABS(J2 - HANDLER_PTR->joint_angle[HAND_J2]) > 0.01f)
    {
      __ADD_JOINT_ANGLE(HAND_J2,
                        J2 > HANDLER_PTR->joint_angle[HAND_J2] ? 0.0008f : -0.0008f);
    }
    // else
    //{
    //   __SET_JOINT_ANGLE(HAND_J2,J2);
    // }
  }

  if (EN & J3_EN)
  {
    if (ABS(J3 - HANDLER_PTR->joint_angle[HAND_J3]) > 0.010f)
    {
      __ADD_JOINT_ANGLE(HAND_J3,
                        J3 > HANDLER_PTR->joint_angle[HAND_J3] ? 0.00105f : -0.00105f);
      //__ADD_JOINT_ANGLE(HAND_J3,0.001f*(J3-HANDLER_PTR->joint_angle[HAND_J3]));
    }
  }
}

uint8_t __hand_pitch_pos_init(uint8_t reset)
{
  static int loop_count = 0;
  static float last_L_angle = 0.0f;
  static float init_start_flag = 0;
  static uint8_t init_complete_flag = 0;
  if (reset)
  {
    init_start_flag = 1;
    init_complete_flag = 0;
    loop_count = 0;
    last_L_angle = 0.0f;
    __CLEAR_MOTOR_OFFLINE(DJI_HE_L);
    __CLEAR_MOTOR_OFFLINE(DJI_HE_R);
  }

  if (init_complete_flag == 0 && init_start_flag == 1)
  {

    osDelay(50);

    if (loop_count < 10)
    {
      DJI_Motor_clear_offline_flag(__GET_MOTOR_INSTANCE(DJI_HE_R));
      DJI_Motor_clear_offline_flag(__GET_MOTOR_INSTANCE(DJI_HE_L));
      __SET_MOTOR_CURRENT(DJI_HE_R, -1000);
      __SET_MOTOR_CURRENT(DJI_HE_L, 1000);
      loop_count++;
    }
    else if (ABS(last_L_angle - __GET_MOTOR_ANGLE(DJI_HE_L)) > 0.3 && init_complete_flag != 1)
    {
      last_L_angle = __GET_MOTOR_ANGLE(DJI_HE_L);
      __SET_MOTOR_CURRENT(DJI_HE_R, -700);
      __SET_MOTOR_CURRENT(DJI_HE_L, 700);
    }
    else
    {
      init_start_flag = 0;
      init_complete_flag = 1;
      // buzzer_on(0,0);
      //__SET_JOINT_LIMIT(HAND_PITCH,0+PITCH_MAP_D,145.0f*PITCH_MAP_K+PITCH_MAP_D);
      __SET_JOINT_ANGLE(HAND_PITCH, PITCH_MAP_D);
      __SET_JOINT_ANGLE(HAND_ROLL, 0.0f);
      __SET_MOTOR_CURRENT(DJI_HE_R, 0);
      __SET_MOTOR_CURRENT(DJI_HE_L, 0);
      DJI_Motor_clear_circle_count(__GET_MOTOR_INSTANCE(DJI_HE_R));
      DJI_Motor_clear_circle_count(__GET_MOTOR_INSTANCE(DJI_HE_L));
      return 1;
    }
  }
  return 0;
}

uint8_t __hand_J1_init(uint8_t reset)
{
  static uint8_t send_init_delay_count = 0;
  static float last_joint_angle = 0.0;
  static uint8_t to_limit_flag = 0;

  if (reset)
  {
    send_init_delay_count = 0;
    last_joint_angle = 0.0f;
    to_limit_flag = 0;
    AK_joint_motor_speed_ctrl(__GET_MOTOR_INSTANCE(AK_J1), 1000);
    osDelay(50);
    return 0;
  }

  if (toe_is_error(TOE_J1))
    return 0;

  if (!to_limit_flag)
  {
    if (send_init_delay_count < 100)
    {
      send_init_delay_count++;
      AK_joint_motor_speed_ctrl(__GET_MOTOR_INSTANCE(AK_J1), 2000);
      return 0;
    }
    else if (__GET_MOTOR_SPEED(AK_J1) > 800)
    {
      last_joint_angle = __GET_JOINT_ANGLE(HAND_J1);
      AK_joint_motor_speed_ctrl(__GET_MOTOR_INSTANCE(AK_J1), 3000);
      osDelay(50);
      return 0;
    }
    else
    {
      to_limit_flag = 1;
      send_init_delay_count = 0;
    }
  }

  if (to_limit_flag)
  {
    if ((0.0f != __GET_JOINT_ANGLE(HAND_J1) || toe_is_error(TOE_J1))) /*等待J1控制板初始化*/
    {
      //__hand_idle_ctrl();/*J1电机需要给一个信号才发反馈*/
      //__hand_nonforce();/*防止移动*/
      /*上述注释代码不会产生任何移动*/

      __SET_MOTOR_CTRL_MODE(HAND_J1, NON_FORCE);
      if (send_init_delay_count == 0)
      {
        AK_joint_motor_set_zero_pos(__GET_MOTOR_INSTANCE(AK_J1));
        send_init_delay_count = 50;
        osDelay(30);
      }
      AK_joint_motor_nonforce_ctrl(__GET_MOTOR_INSTANCE(AK_J1));
      send_init_delay_count--;
      return 0;
    }
    send_init_delay_count = 50;
    return 1;
  }
  return 0;
}

uint8_t __hand_J2_init(void)
{
  if (toe_is_error(TOE_J2) || DM_Motor_J2.para.state == 0x00)
  {
    __SET_MOTOR_CTRL_MODE(HAND_J2, NON_FORCE);
    disable_motor_mode(&hfdcan2, 1, MIT_MODE);
    osDelay(1);
    enable_motor_mode(&hfdcan2, 1, POS_MODE);
    return 0;
  }
  return 1;
}

uint8_t __hand_J3_init(void)
{
  static int loop_count = 0;
  static float last_L_angle = 0.0f;
  static uint8_t init_complete_flag = 0;
  if (__IS_MODE_SWITCHED())
  {
    init_complete_flag = 0;
    loop_count = 0;
    last_L_angle = 0.0f;
  }

  if (init_complete_flag == 0)
  {

    osDelay(50);

    if (loop_count < 20)
    {
      DJI_Motor_clear_offline_flag(__GET_MOTOR_INSTANCE(DJI_J3));
      __SET_MOTOR_CURRENT(DJI_J3, 1000);
      loop_count++;
    }
    else if (ABS(last_L_angle - __GET_MOTOR_ANGLE(DJI_J3)) > 0.2 && init_complete_flag != 1)
    {
      last_L_angle = __GET_MOTOR_ANGLE(DJI_J3);
      __SET_MOTOR_CURRENT(DJI_J3, 800);
    }
    else
    {
      init_complete_flag = 1;
      __SET_JOINT_ANGLE(HAND_J3, 0.0f);
      __SET_MOTOR_CURRENT(DJI_J3, 0);
      DJI_Motor_clear_circle_count(__GET_MOTOR_INSTANCE(DJI_J3));
      hand_task_get_feedback();
      // J3_D=-__GET_MOTOR_ANGLE(DJI_J3);
      return 1;
    }
  }
  return 0;
}

// 人来移动机械臂到自己设置的角度来让电机知道自己当前的确切角度初始化（有减速比，一个关节角度（带减速箱）对应多个电机当前角度
// 下方代码一段和这段相关->__hand_pose_ctrl
// void __hand_custom_map_subctrl(void)
// {
//   static int Joint_init = 0;
//   float Joint_pos1[5];
//   float Joint_pos2[5];

//   if (Joint_init == 0)
//   {
//     __hand_move2_subctrl(0, -PI / 2, 0, 0, 0, J1_EN | J2_EN | J3_EN);
//     if (RC_CTRL_PTR->rc.ch[3] == 660)
//     {
//       __HOLD_TICKS_COUNTING();
//       if (__GET_TICKS_STACK(0) == 0)
//         __RECORD_TICKS(0);
//       else if (__GET_TICKS_TIME() - __GET_TICKS_STACK(0) > 1000)
//       {
//         __RESET_TICKS();
//         __RESET_RECORD_TICKS(0);
//         __HALT_TICKS_COUNTING();
//         memcpy((void *)Joint_pos1, (void *)cc_joint_angle, sizeof(float) * 3);
//         Joint_init = 1;
//       }
//     }
//     else
//     {
//       __RESET_TICKS();
//       __RESET_RECORD_TICKS(0);
//       __HALT_TICKS_COUNTING();
//     }
//   }

//   if (Joint_init == 1)
//   {
//     __hand_move2_subctrl(-PI / 2, 0, PI / 2, 0, 0, J1_EN | J2_EN | J3_EN);
//     if (RC_CTRL_PTR->rc.ch[3] == 660)
//     {
//       __HOLD_TICKS_COUNTING();
//       if (__GET_TICKS_STACK(0) == 0)
//         __RECORD_TICKS(0);
//       else if (__GET_TICKS_TIME() - __GET_TICKS_STACK(0) > 1000)
//       {
//         __RESET_TICKS();
//         __RESET_RECORD_TICKS(0);
//         __HALT_TICKS_COUNTING();
//         memcpy((void *)Joint_pos2, (void *)cc_joint_angle, sizeof(float) * 3);
//         Joint_init = 2;
//       }
//     }
//     else
//     {
//       __RESET_TICKS();
//       __RESET_RECORD_TICKS(0);
//       __HALT_TICKS_COUNTING();
//     }
//   }

//   if (Joint_init == 2)
//   {
//     custom_controller_D[0] = Joint_pos2[0];
//     custom_controller_D[1] = Joint_pos2[1];
//     custom_controller_D[2] = Joint_pos1[2];

//     custom_controller_K[0] = (PI / 2) / ABS(Joint_pos1[0] - Joint_pos2[0]);
//     custom_controller_K[1] = (PI / 2) / ABS(Joint_pos1[1] - Joint_pos2[0]);
//     custom_controller_K[2] = (PI / 2) / ABS(Joint_pos1[2] - Joint_pos2[0]);
//   }
// }

void __hand_gold_catch_ctrl(void)
{
  __ADD_JOINT_ANGLE(HAND_J1, -RC_CTRL_PTR->rc.ch[2] * 0.0000007f * 2);
  __ADD_JOINT_ANGLE(HAND_J3, -RC_CTRL_PTR->rc.ch[4] * 0.000025f * J3_MAP_K);
  __ADD_JOINT_ANGLE(HAND_PITCH, -RC_CTRL_PTR->rc.ch[3] * 0.0001f * PITCH_MAP_K);
  if (__GET_JOINT_ANGLE(HAND_PITCH) > 0)
    __SET_JOINT_ANGLE(HAND_PITCH, 0);
  __hand_move2_subctrl(0, -0.950, 0, 0, 0, J2_EN);
}

void __hand_catch_ground(void)
{
  __hand_move2_subctrl(-PI / 4, -PI * 2 / 3, 0.0f, __GET_JOINT_MAX_LIM(HAND_PITCH), 0.0f, J1_EN | J2_EN | J3_EN | J4_EN);
}

// void __hand_pose_ctrl(void)
// {
//   static uint8_t pose_mode = 0;
//   if (__IS_MODE_SWITCHED())
//   {
//     __RESET_TICKS();
//     __HALT_TICKS_COUNTING();
//     pose_mode = 0;
//   }

//   /*Pose control command*/
//   if (pose_mode == 0)
//   {
//     if (RC_CTRL_PTR->rc.ch[3] == -660)
//     {
//       __HOLD_TICKS_COUNTING();
//       if (__GET_TICKS_STACK(0) == 0)
//         __RECORD_TICKS(0);
//       else if (__GET_TICKS_TIME() - __GET_TICKS_STACK(0) > 1000)
//         pose_mode = 1;
//     }
//     else if (RC_CTRL_PTR->rc.ch[1] == -660)
//     {
//       __HOLD_TICKS_COUNTING();
//       if (__GET_TICKS_STACK(0) == 0)
//         __RECORD_TICKS(0);
//       else if (__GET_TICKS_TIME() - __GET_TICKS_STACK(0) > 2000)
//         pose_mode = 2;
//     }
//     else if (RC_CTRL_PTR->rc.ch[1] == 660)
//     {
//       __HOLD_TICKS_COUNTING();
//       if (__GET_TICKS_STACK(0) == 0)
//         __RECORD_TICKS(0);
//       else if (__GET_TICKS_TIME() - __GET_TICKS_STACK(0) > 2000)
//         pose_mode = 4;
//     }
//     else if (RC_CTRL_PTR->rc.ch[3] == 660)
//     {
//       __HOLD_TICKS_COUNTING();
//       if (__GET_TICKS_STACK(0) == 0)
//         __RECORD_TICKS(0);
//       else if (__GET_TICKS_TIME() - __GET_TICKS_STACK(0) > 2000)
//         pose_mode = 5;
//     }
//     else
//     {
//       __RESET_TICKS();
//       __RESET_RECORD_TICKS(0);
//       __HALT_TICKS_COUNTING();
//     }
//   }

//   if (pose_mode == 1)
//     __hand_move2_subctrl(-PI / 8, -2.2, 0.5, 0, 0, J1_EN | J2_EN | J3_EN);
//   else if (pose_mode == 2)
//     //__hand_custom_ctrl();
//     __hand_rc_ctrl();
//   else if (pose_mode == 3)
//     __hand_custom_map_subctrl();
//   else if (pose_mode == 4)
//     __hand_gold_catch_ctrl();
//   else
//     ;
// }

void __hand_move_GGM(void)
{
  if (get_step() == GGM_hand_to_pos)
  {
    if (
        is_angle_around(GGM_STEP2_J1_ANGLE, __GET_JOINT_ANGLE(HAND_J1), 0.1f) &&
        is_angle_around(GGM_STEP2_J2_ANGLE, __GET_JOINT_ANGLE(HAND_J2), 0.1f) &&
        is_angle_around(GGM_STEP2_J3_ANGLE, __GET_JOINT_ANGLE(HAND_J3), 0.1f) &&
        is_angle_around(GGM_STEP2_PITCH_ANGLE, __GET_JOINT_ANGLE(HAND_PITCH), 0.1f))
      next_step();
    else
      __hand_move2_subctrl(GGM_STEP2_J1_ANGLE, GGM_STEP2_J2_ANGLE, GGM_STEP2_J3_ANGLE, GGM_STEP2_PITCH_ANGLE, 0.0f, J1_EN | J2_EN | J3_EN | J4_EN);
  }
}

void __hand_move_SM(void)
{
  if (get_step() == SM_hand_to_pos)
  {
    if (
        is_angle_around(SM_STEP2_J1_ANGLE, __GET_JOINT_ANGLE(HAND_J1), 0.1f) &&
        is_angle_around(SM_STEP2_J2_ANGLE, __GET_JOINT_ANGLE(HAND_J2), 0.1f) &&
        is_angle_around(SM_STEP2_J3_ANGLE, __GET_JOINT_ANGLE(HAND_J3), 0.1f) &&
        is_angle_around(SM_STEP2_PITCH_ANGLE, __GET_JOINT_ANGLE(HAND_PITCH), 0.1f))
      next_step();
    else
      __hand_move2_subctrl(SM_STEP2_J1_ANGLE, SM_STEP2_J2_ANGLE, SM_STEP2_J3_ANGLE, SM_STEP2_PITCH_ANGLE, 0.0f, J1_EN | J2_EN | J3_EN | J4_EN);
  }

  if (get_step() == SM_hand_to_pos2)
  {
    if (
        is_angle_around(SM_STEP2_J1_ANGLE2, __GET_JOINT_ANGLE(HAND_J1), 0.1f))
      next_step();
    else
      __hand_move2_subctrl(SM_STEP2_J1_ANGLE2, 0.0f, 0.0f, 0.0f, 0.0f, J1_EN);
  }
}

void hand_J1_reset()
{
  __SET_MOTOR_INSTANCE(AK_J1, &AK70_10_motor);
  __SET_MOTOR_TYPE(AK_J1, AK_MOTOR);
  AK_joint_motor_init(&AK70_10_motor, 93);
  AK_joint_motor_enable(&AK70_10_motor);
  __SET_JOINT_LIMIT(HAND_J1, -3.10, 0);

  __hand_J1_init(1);
  do
  {
    hand_task_get_feedback();
    osDelay(1);
  } while (!__hand_J1_init(0));
  hand_task_output();

  __CLEAR_MOTOR_OFFLINE(AK_J1);
  __SET_JOINT_ANGLE(AK_J1, HANDLER_PTR->feedback_joint_angle[AK_J1]);
  __SET_MOTOR_NONFORCE(AK_J1);
}
void hand_J2_reset()
{
  __SET_MOTOR_INSTANCE(DM_J2, &DM_Motor_J2);
  __SET_MOTOR_TYPE(DM_J2, M4310_MOTOR);
  joint_motor_init(&DM_Motor_J2, 1, POS_MODE, 1.0, 1.0);
  __SET_JOINT_LIMIT(HAND_J2, -3.14 / 7 * 5, 3.14 / 7 * 5);

  while (!__hand_J2_init())
    ;
  while (!__IS_JOINT_AROUND(HAND_J2, -1.9))
  {
    hand_task_get_feedback();
    __hand_move2_subctrl(0.0f, -1.9f, 0.0f, 0.0f, 0.0f, J2_EN);
    hand_task_output();
    osDelay(1);
  }

  __CLEAR_MOTOR_OFFLINE(DM_J2);
  __SET_JOINT_ANGLE(DM_J2, HANDLER_PTR->feedback_joint_angle[DM_J2]);
  __SET_MOTOR_NONFORCE(DM_J2);
}

void hand_J3_reset()
{
  __SET_MOTOR_INSTANCE(DJI_J3, &DJI_Motor_J3);
  __SET_MOTOR_TYPE(DJI_J3, DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_J3, &DJI_CAN2_Bus_ctrl, M3508, 0x204);
  // DJI_Motor_set_angle_limit()
  // DJI_Motor_set_speed_limit()
  DJI_Motor_Speed_PID_init(&DJI_Motor_J3, PID_POSITION, 22, 0.000, 0.05, 6000.000, 800);
  DJI_Motor_Pos_PID_init(&DJI_Motor_J3, PID_POSITION, 55, 0.0, 0.0, 200, 100);
  // DJI_Motor_set_sum_angle(&DJI_Motor_J3);
  DJI_Motor_set_multiple_circle_angle(&DJI_Motor_J3);
  __SET_JOINT_LIMIT(HAND_J3, -2, 2);

  DJI_CANBus_enable_bus(&DJI_CAN2_Bus_ctrl);
  while (!__hand_J3_init())
  {
    hand_task_output();
    osDelay(1);
    hand_task_get_feedback();
    __hand_nonforce();

    __CLEAR_MOTOR_OFFLINE(DJI_J3);
    __SET_JOINT_ANGLE(DJI_J3, HANDLER_PTR->feedback_joint_angle[DJI_J3]);
    __SET_MOTOR_NONFORCE(DJI_J3);
  }
}

void hand_pitch_reset()
{
  // Headend_L
  __SET_MOTOR_INSTANCE(DJI_HE_L, &DJI_Motor_headendL);
  __SET_MOTOR_TYPE(DJI_HE_L, DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_headendL, &DJI_CAN2_Bus_ctrl, M2006, 0x201);
  DJI_Motor_Speed_PID_init(&DJI_Motor_headendL, PID_POSITION, 22, 0.001, 0, 5500, 1000);
  DJI_Motor_Pos_PID_init(&DJI_Motor_headendL, PID_POSITION, 75, 0, 0, 2000, 1000);
  // DJI_Motor_set_offline_detect(&DJI_Motor_headendL,8,0);
  // DJI_Motor_set_sum_angle(&DJI_Motor_headendL);
  DJI_Motor_set_multiple_circle_angle(&DJI_Motor_headendL);
  // Headend_R
  __SET_MOTOR_INSTANCE(DJI_HE_R, &DJI_Motor_headendR);
  __SET_MOTOR_TYPE(DJI_HE_R, DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_headendR, &DJI_CAN2_Bus_ctrl, M2006, 0x208);
  DJI_Motor_Speed_PID_init(&DJI_Motor_headendR, PID_POSITION, 10, 0.001, 0, 5500, 1000);
  DJI_Motor_Pos_PID_init(&DJI_Motor_headendR, PID_POSITION, 75, 0, 0, 2000, 0);
  // DJI_Motor_set_offline_detect(&DJI_Motor_headendR,8,0);
  // DJI_Motor_set_sum_angle(&DJI_Motor_headendR);
  DJI_Motor_set_multiple_circle_angle(&DJI_Motor_headendR);
  __SET_JOINT_LIMIT(HAND_PITCH, 5 * PITCH_MAP_K + PITCH_MAP_D, 150.0f * PITCH_MAP_K + PITCH_MAP_D);
  __SET_JOINT_ANGLE(HAND_PITCH, PITCH_MAP_D);

  DJI_CANBus_enable_bus(&DJI_CAN2_Bus_ctrl);
  __hand_pitch_pos_init(1);
  for (int i = 0; i < 40; i++)
  {
    hand_task_get_feedback();
    __hand_nonforce();
    if (__hand_pitch_pos_init(0))
    {
      break;
    }
    hand_task_output();
    osDelay(1);
  }

  __CLEAR_MOTOR_OFFLINE(DJI_HE_L);
  __SET_JOINT_ANGLE(DJI_HE_L, HANDLER_PTR->feedback_joint_angle[DJI_HE_L]);
  __SET_MOTOR_NONFORCE(DJI_HE_L);
  __CLEAR_MOTOR_OFFLINE(DJI_HE_R);
  __SET_JOINT_ANGLE(DJI_HE_R, HANDLER_PTR->feedback_joint_angle[DJI_HE_R]);
  __SET_MOTOR_NONFORCE(DJI_HE_R);
}
//当检测到掉电时通过general_motor_module在电机内部重新初始化，然后标志位在hand中读取到在单独甩该电机大臂
void __hand_move_reset(void)
{
  if (__IS_MOTOR_OFFLINE(AK_J1))
  {
    hand_J1_reset();
  }
  if (__IS_MOTOR_OFFLINE(DM_J2))
  {
    hand_J2_reset();
  }
  if (__IS_MOTOR_OFFLINE(DJI_J3))
  {
    hand_J3_reset();
  }
  if (__IS_MOTOR_OFFLINE(DJI_HE_L) || __IS_MOTOR_OFFLINE(DJI_HE_R))
  {
    hand_pitch_reset();
  }
}
// case OFFLINE:

/*
  $$$$$$$}.........$$$$$$$$$   "00000000$$$$""""""""""""""""""""""*$$$$%000$$     ........$$.....
  ...               .$"   "00000000$$$$"""""""""""""""""""""""""""""""*$$j...$.          ...$w...
  .              $$   "000000$$$$$""""""""""""""""""""""""""""""""""""""$$.....$$         ...j$..
  .....$ """   ""0$$$$$$;""""""""""""""""""""""""""""""""""""""""""""""""$jjj...$   $$   $$$$$$$$
  ....$ $jjjj$$$$"-"""""""""""""""""  $$$$$$.                     """"""""$j....w$$  $$  ........
  ...$"$.j$$$$""""""""    """"         :$$$.                              $.....j$$$$ ...
  ../"0.$$$$-""     "    ""              ;l                              .j.....$$.......    $ $
  .j$ $$$$"""      "" ""                                                 $j....$ ..  j....    $
  $$" $$"""""     ""              B$$$;.                                .jjj.jjU...   j.....   $
  #$ $$f"""      "             $$000000000p$$$$'                       ;$j>.jjj$.....  j.....  $
  $$$$"""""   ""            .$00000000000000000000$$$.                $...jjjjjj$$...... $$$$x
  .$$:""""                .$000000$$$$$$$$$000000000000$$$$B......;$$p*$jjjjjjj$jjj$$j$$$.$$$$$
  j$0""""               .$00000$$0000$$$$$$$$$$$$$$$%000000000000000$$$$$jjjjj$j.jjjj$$$$$$
  .$"""""              $"0000$$00000$$$$$$$$$$$$0$$$$$$$$$$$$$$$$$$$$$$$$$j$$     >$
  .j"""""            ;" 0000$000$$$`/jjjjjj>..jj$$$$$$$$$$$$$$$$$$$$$$$j.$"
  ..$"""           ^$ Q000$$00$$>jjjjjjjjjjjjjj>....``>//jjjjjjjjUjjj...`$
  ..j$""         .$00000$$$0$$.....$jjjjjj/`..............jjjjjjj$.....$$$$$;
  ..Uj$""      $$00000$$$$$$........$jj.........................$.........jjjjjjjjjjjj$$
  j.....j$$$$0  '00$$$$$$$j..........j$............j...........$  ........$jjjjjjjwjjjjj[$$
  $$jjjj.$0000""$$$$$$$$jj........j$$$jjj...........j.........$.jj.   ....$jjjj/$j$jjjjjjjjjj$$
  $$$jjjj$$$$$$$$$$$$jjj#.....`$j       j$j.........[........$. $$$$$$$$$$$$$j$j...$jjjjjjjjjjjjj
  .$$$$$j$$$$$$$$$$[jjj$......$.         .$j$...............j $..$$$$$$$$$$$$$$j....$jjjjjjjjjjjj
  ..$$$/[jj.    jjjjjj$.......  ..jjjj...  ...jj..........j.  .$$$$$B;;;;;$$  $$$...[$jjjjjjjj/$j
  ..$$$/jjjjjjjjjjjjj$....                   ....jw......j   $$  j$:;;;;;;;$$   $$j..jjjjjjjjjjjj
  ...$$jjjjjjjjjjjj[$....                      .....$..j.  .$    $;:":;;":;$$    $$U..$$jjjjjjjjj
  ....$$.jjjjjjjjjjjj...                          ..@.j.   .       "";$$""";$     $$/..$[jjjjjjjj
  .....$$..[jjjjjj/$...                           ..j.           $;""u$$"""$j    .@...jjjjjjjjjjj
  ......j$.....jj$$>...       .......jj           ..             j$"""""";;$     $....$jjwjjjjjj/
    ......$.......$...         .$$$$$j.                           $$""""""$     $....jj$jjjjjjjjj
  .......jjj$......$.    .#$$$$$$$$$$$$$$.    .$$$$$                       ..jj..... j$jjjjjjjjjj
  ....jjjjjjj$$.....$  j$[$$$$$$$$.            $$$                                    jjjjjjjjjjj
  $$$$$$$jjjjjjj$j...$..j$$$$j..                                                     $jjjjjjjjjjj
      $$jjjjjjjjj$j$$..$$j...                       $$j                             $jjjjjjjjjjjj
      $$jjjjjjjjjj$j......                       $jjjjjjjjj$  $$                   $jjjjjjjjjjjjj
     $$[jjjjjjjjjjj$                         $$jjjjjjjjjjjjjj$j$                  xjjjjj>>jjjjjjj
    $$jjjjjjjjjjjjj$                        $$jjjjjjjjjjjjjjjjj$                 $jjj.....jjjjjjj
   $$.jjjjjjjjjjj$                           $jjjjjjjjjjjjjjjjjj               jj.........jjjjjjj
  $$.[jjjjjjjjj$                              $jjjjjjjjjjjjjjj$              $............[jjjjjj
  $..jjjjjjjj$                                 jjjjjjjjjjjjjj$            .j....   .......>jjjjjj
   .jjjjjjjjj[j$                                 j$jjjjjjjjj$            j................`jjjjjj
  ..jjjjjjjjjjjjjjjj$                                 jj               j....           ....j[.  $
  .jjjjjjjjjjjjjjjj/$    w$$$                                         $..               .       $
  .jjjjjjjjjjjjjjjjjj$w$$$$j/jjjj$$@                                  j                        $$
  jjjjjjjjjjjjjjjjjjj$$$   $$$jjjjjjj/jj#$$                          ^                         $$
  jjjjjjjjjjjjjjjjjj$$        $$$`jjjjjjjjjj.$$                       $                         $
  jjjjjjjjjjjjjjjj$$            $$$jjjjjjjjjjjj$                       $
  jjjjjjjjjjjjjj$$$                $$jj$$$$$$$$$$                       "$                  $'
  jjjjjjjjjjjj$$$               $$$$jj/jjjjj$$$$.                         $$                 $$$$


  */

#undef HANDLER
#undef HANDLER_PTR
