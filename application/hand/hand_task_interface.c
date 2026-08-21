/**
 * @brief
 * @attention lockup的逻辑大抵没有大🐱饼
 * @note hand_task内的角度单位为rad
 */

// notice:(未解决)
/*调试时在debug运行程序开控打断点或者直接停止后再运行3508电机会出现奇怪现象(遗留问题)
底盘会抽搐，抬升可能是pid的i值积累到最大瞬间输出，猜测可能调试阶段芯片一部分控制权给keil，停止后一部分任在运行导致程序错乱
造成该问题的原因可能是打断点的时候timer_ctrl还在运行*/

// notice:
/*基于上述,千万注意调车的时候将车的底盘垫高,不然有可能出现十分恐怖的情况 */

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
		extern FDCAN_HandleTypeDef hfdcan1;
    extern FDCAN_HandleTypeDef hfdcan2;

/*ABS*/
#define ABS(X) ((X) > 0 ? (X) : -(X))
/*global macro variable*/
#define HAND_CTRL_CAN
// joint mapping parameter
// #define J1_MAP_K   0.167 /*for m8010*/
#define J1_MAP_K -0.841f//J1已改
#define J1_MAP_D -0.0434f

#define J2_MAP_K 1.0f
#define J2_MAP_D 0

#define J3_MAP_K 1.0f
#define J3_MAP_D 0

#define J4_MAP_K 1.0f
#define J4_MAP_D 0

#define J5_MAP_K -0.6710f//J5已更改
#define J5_MAP_D 1.6349f

#define GR_MAP_K 1.0f
#define GR_MAP_D 0
// #define J3_MAP_K (-3.14f / 42.4f) /*old value(-1.0f/19.2f)*/
// #define J3_MAP_D (-1.668)
// #define J4_MAP_K (180.0f / -457.0f)
// #define J4_MAP_D (1.18f)
#define dm_g_max -2.515f
#define dm_g_min -2.856f
// #define PITCH_MAP_K (PI / 2 / (135.0f - 50.0f))
// #define PITCH_MAP_D (-50 * PITCH_MAP_K)
// #define ROLL_MAP_K (PITCH_MAP_K / 2)
// #define ROLL_MAP_D 0
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
#define JG_EN (0x01 << 5)

/*global motor handler*/
extern Joint_Motor_t DM_Motor_J1;
extern Joint_Motor_t DM_Motor_J2;
extern Joint_Motor_t DM_Motor_J3;
extern Joint_Motor_t DM_Motor_J4;
extern Joint_Motor_t DM_Motor_J5;
extern Joint_Motor_t DM_Motor_gripper;//暂时定gripper为机械臂J6

// extern M8010_motor_t joint1_motor;
// DJI_Motor_Ctrl_t DJI_Motor_headendL;
// DJI_Motor_Ctrl_t DJI_Motor_headendR;

/*global variable*/
// static fp32 J1_D = 0;
// static fp32 J3_D = 0;
 uint8_t idle_flag = 0;

static void __hand_nonforce(void);
static void __hand_idle_ctrl(void);
static void __hand_rc_ctrl(void);
static void __hand_custom_ctrl(void);
// static void __hand_catch_ground(void);

static void __detect_hand_motor_offline(void);
static uint8_t __hand_motor_refresh_online(int index);
static void __set_motor_offline_flag(void);
static void __detect_hand_motor_stall(void);

static void __hand_move2_subctrl(fp32 J1, fp32 J2, fp32 J3, fp32 J4, fp32 J5, fp32 JG, uint8_t EN);
static void __hand_motor_go_setting_angle(HAND_JOINT_INDEX index, float angle_value, float h_dead, float l_dead, float h_value, float l_value);

static void basic_motor_init(void);
static void __J1_init(void);
static void __J2_init(void);
static void __J3_init(void);
static void __J4_init(void);
static void __J5_init(void);
static void __gripper_init(void);

static void __DM_go_setting_angle(HAND_JOINT_INDEX index, float angle_value, float h_value, float l_value);
static uint8_t __hand_J1_init(void);
// static uint8_t __hand_J1_init(void);
static uint8_t __hand_J2_init(void);
static uint8_t __hand_J3_init(void);
static uint8_t __hand_J4_init(void);
static uint8_t __hand_J5_init(void);
static uint8_t __hand_gripper_init(void);

static void __hand_move_BTD(void);
static void __hand_move_GSM(void);
static void __hand_move_OCSM(void);

static void __hand_rc2_ctrl(void);

// static void hand_pitch_reset(void);

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
#define __IS_JOINT_AROUND(INDEX, ANGLE) (is_angle_around(ANGLE, __GET_JOINT_ANGLE(INDEX), 0.05f))
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
#define __RESET_TICKS() (HANDLER_PTR->tick = 0)                    // tick为运行当前任务次数
#define __HALT_TICKS_COUNTING() (HANDLER_PTR->tick_count_halt = 1) // 停止
#define __HOLD_TICKS_COUNTING() (HANDLER_PTR->tick_count_halt = 0) // 运行
#define __IS_TIMER_HALT() (1 == HANDLER_PTR->tick_count_halt)
#define __GET_TICKS() (HANDLER_PTR->tick)
// unit:seconds
#define __GET_TICKS_TIME() (HANDLER_PTR->tick * 1)                                 // 若运行次数和时间为一定比例可修改tick乘积
#define __GET_PROCESS_PERCENTAGE(PROCESS_TIME) (__GET_TICKS_TIME() / PROCESS_TIME) // 分频率
#define __GET_TICKS_STACK(index) (HANDLER_PTR->tick_stack[index])                  // 机械臂每个电机单独tick//当前暂不使用
#define __RECORD_TICKS(index) (HANDLER_PTR->tick_stack[index] = __GET_TICKS_TIME())
#define __RESET_RECORD_TICKS(index) (HANDLER_PTR->tick_stack[index] = 0)
#define __IS_MODE_SWITCHED() (1 == HANDLER_PTR->mode_switch)

#define __LONG_PRESS_TRIGGER_FUNCTION(key, steady_move, deir) \
  {                                                           \
    static uint16_t press_count = 0;                          \
    static uint8_t aviod_triggered_again = 0;                 \
    if (key)                                                  \
    {                                                         \
      if (!aviod_triggered_again)                             \
      {                                                       \
        press_count++;                                        \
        if (press_count >= 1000)                              \
        {                                                     \
          set_movement(steady_move);                          \
          movement.mine_place = deir;                         \
          aviod_triggered_again = 1;                          \
        }                                                     \
      }                                                       \
    }                                                         \
    else                                                      \
    {                                                         \
      press_count = 0;                                        \
      aviod_triggered_again = 0;                              \
    }                                                         \
  }

void hand_task_init()
{
  osDelay(1000);

  // 基础初始化
  __HALT_TICKS_COUNTING();
  __RESET_TICKS();

  basic_motor_init();
  osDelay(100);
  // 机械臂初始化
   __gripper_init();
   osDelay(50);
    __J5_init();
    osDelay(50);
    __J4_init();
    osDelay(50);
    __J3_init();
    osDelay(50);
   __J2_init();
   osDelay(50);
   __J1_init();
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

    StallUpdateHook(index, __GET_MOTOR_ANGLE(index), __GET_MOTOR_SPEED(index), __GET_MOTOR_CURRENT(index));
  }

  /*joint angle map*/
  __GET_JOINT_ANGLE(HAND_J1) = J1_MAP_K * __GET_MOTOR_ANGLE(DM_J1) + J1_MAP_D;
  __GET_JOINT_ANGLE(HAND_J2) = J2_MAP_K * __GET_MOTOR_ANGLE(DM_J2) + J2_MAP_D;
  __GET_JOINT_ANGLE(HAND_J3) = J3_MAP_K * __GET_MOTOR_ANGLE(DM_J3) + J3_MAP_D;
  __GET_JOINT_ANGLE(HAND_J4) = J4_MAP_K * __GET_MOTOR_ANGLE(DM_J4) + J4_MAP_D;
  __GET_JOINT_ANGLE(HAND_J5) = J5_MAP_K * __GET_MOTOR_ANGLE(DM_J5) + J5_MAP_D;
  __GET_JOINT_ANGLE(HAND_G) = GR_MAP_K * __GET_MOTOR_ANGLE(dm_gripper) + GR_MAP_D;

  
}

/**
 * @brief 模式状态刷新
 * @details 根据控制器拨杆刷新模式(二级模式会与UI耦合)
 */
void hand_task_mode_flush()
{
  // static uint8_t last_mode = HAND_MODE_NONFORCE;
  // last_mode = HANDLER_PTR->ctrl_mode;

  // 新控
  //   switch(GET_SWITCH())
  //   {
  //     case 1:
  //       //__SET_STRUCT_MODE(HAND_MODE_RC_CTRL);
  //       __SET_STRUCT_MODE(HAND_MODE_IDLE);
  //       break;
  //     case 2:
  //       __SET_STRUCT_MODE(HAND_MODE_CUSTOM_CTRL);
  //       break;
  //     case 0:
  //     default:
  //       __SET_STRUCT_MODE(HAND_MODE_NONFORCE);
  //   }

  // 老控
  if (switch_is_up(get_remote_control_point()->rc.s[1]))
  {
    if (switch_is_down(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(HAND_MODE_IDLE);
    else if (switch_is_mid(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(HAND_MODE_RC_CTRL);
    else if (switch_is_up(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(HAND_MODE_RC2_CTRL);
    // __SET_STRUCT_MODE(HAND_MODE_CUSTOM_CTRL);
    //__SET_STRUCT_MODE(HAND_MODE_IDLE);
  }
  else if (switch_is_mid(get_remote_control_point()->rc.s[1]))
  {
    if (switch_is_up(get_remote_control_point()->rc.s[0]))
      //__SET_STRUCT_MODE(HAND_MODE_CUSTOM_CTRL);
      __SET_STRUCT_MODE(HAND_MODE_IDLE);

    else
      __SET_STRUCT_MODE(HAND_MODE_IDLE);
  }
  else if (switch_is_down(get_remote_control_point()->rc.s[1]))
  {
    if (switch_is_mid(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(HAND_MODE_IDLE);
    else
      __SET_STRUCT_MODE(HAND_MODE_IDLE);
  }

  // static uint8_t mode_var = 0; // 没用
  // if (__GET_STRUCT_MODE() == HAND_MODE_IDLE && !GetMatchReady())
  if (__GET_STRUCT_MODE() == HAND_MODE_CUSTOM_CTRL)
  {
    // __BUTTON_PRESS_SWITCH_WRAP(GET_KEY(KEY_Z),mode_var,1,10,__hand_catch_ground);//与气泵有关暂时保留
    // __BUTTON_PRESS_SWITCH_WRAP

    //  if (remote_data.trigger)//新控的
    //    __hand_rc_ctrl();

    // if (switch_is_up(get_remote_control_point()->rc.s[0]) && switch_is_mid(get_remote_control_point()->rc.s[1]))//使用控调试
    //   set_movement(ONCE_CLICK_SAVE_MINE); // 里面会把步骤又置为0，所以要拨到挡当然后退出该挡，后续在键盘上为按下按键不会有无法进行下一步的情况
    // 比赛时设置一个按键可以打算所有固定动作(防止在途中卡住)，或者让自定义控制器优先级比这个高当自定义控制时可以打断固定动作

    // 牢骚：对于一键取矿本质是一键存矿的倒转动作，可以设置step++变为step--,但实测发现因为j1电机精度问题夹爪伸进存矿位置有概率伸歪，所有只能再写一套动作
    // 后来者如果有条件可以把一键存取左右两边四个动作的代码整合到一个函数内（或者将它抽象封装）
    __LONG_PRESS_TRIGGER_FUNCTION(GET_KEY(KEY_Z), ONCE_CLICK_SAVE_MINE, LEFT);
    __LONG_PRESS_TRIGGER_FUNCTION(GET_KEY(KEY_X), ONCE_CLICK_SAVE_MINE, RIGHT);
    __LONG_PRESS_TRIGGER_FUNCTION(GET_KEY(KEY_G), BTD, NON_DEIR);
    __LONG_PRESS_TRIGGER_FUNCTION(GET_KEY(KEY_Z) && GET_KEY(KEY_CTRL), ONCE_CLICK_GET_MINE, LEFT);
    __LONG_PRESS_TRIGGER_FUNCTION(GET_KEY(KEY_X) && GET_KEY(KEY_CTRL), ONCE_CLICK_GET_MINE, RIGHT);

    if (GET_KEY(KEY_B))
    {
      set_movement(NON); // 退出固定动作
      movement.hand_step_complete = 0;
      movement.height_step_complete = 0;
      movement.mine_place = NON_DEIR;
      movement.movement_step = 0;
      // movement.hand_move_out_flag = 1;
    }
    if (get_movement() == BTD)
    {
      __SET_STRUCT_MODE(HAND_MODE_BTD_CTRL);
    }

    if (get_movement() == ONCE_CLICK_SAVE_MINE)
    {
      __SET_STRUCT_MODE(HAND_MODE_SM_CTRL);
    }
    if (get_movement() == ONCE_CLICK_GET_MINE)
    {
      __SET_STRUCT_MODE(HAND_MODE_GSM_CTRL);
    }
    
  }

  {
    static uint16_t press_count = 0;
    static uint8_t aviod_triggered_again = 0;
    if (GET_KEY(KEY_CTRL) && GET_KEY(KEY_R))
    {
      if (!aviod_triggered_again)
      {
        press_count++;
        if (press_count >= 500)
        __J4_init();

      }
    }
    else
    {
      press_count = 0;
      aviod_triggered_again = 0;
    }
  }

  // if ( GET_KEY(KEY_F)) // 机械臂保持不动
  //   idle_flag =0;
  
  // if (idle_flag == 1)
  //   __SET_STRUCT_MODE(HAND_MODE_IDLE);
 

    // else
    // {
    //   mode_var = 0;
    // }

    // 比赛开始阶段保持当前位置不动
    //  if (GetMatchReady())
    //  {
    //    __SET_STRUCT_MODE(HAND_MODE_IDLE);
    //  }

// if (toe_is_error(DBUSTOE) && toe_is_error(CAMERA_TOE))
// {
//   __SET_STRUCT_MODE(HAND_MODE_NONFORCE);
// }

  // if (HANDLER_PTR->ctrl_mode == last_mode)
  //   HANDLER_PTR->mode_switch = 0;
  // else
  // {
  //   __RESET_TICKS();
  //   __HALT_TICKS_COUNTING();
  //   HANDLER_PTR->mode_switch = 1;
  //   set_movement(0);
  //}
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
  case HAND_MODE_RC2_CTRL:
    __hand_rc2_ctrl();
    break;

  // case HAND_MODE_RESET_CTRL:
  //   __hand_move_reset();
  //  break;
  case HAND_MODE_NONFORCE:
  default:
    __hand_nonforce();
  }

  __detect_hand_motor_offline();

  __set_motor_offline_flag();
  __detect_hand_motor_stall();
}

/**
 * @brief 控制输出
 * @details 根据电机种类与控制状态设定输出
 */
void hand_task_output()
{
  uint16_t index;

  /*joint map to motor state*/
  
  if (__GET_MOTOR_CTRL_MODE(DM_J1) == POS_LOOP)
    __SET_MOTOR_ANGLE(DM_J1, (HANDLER_PTR->joint_angle[HAND_J1] - J1_MAP_D) / J1_MAP_K);
  if (__GET_MOTOR_CTRL_MODE(DM_J2) == POS_LOOP)
    __SET_MOTOR_ANGLE(DM_J2, (HANDLER_PTR->joint_angle[HAND_J2] - J2_MAP_D) / J2_MAP_K);
  if (__GET_MOTOR_CTRL_MODE(DM_J3) == POS_LOOP)
    __SET_MOTOR_ANGLE(DM_J3, (HANDLER_PTR->joint_angle[HAND_J3] - J3_MAP_D) / J3_MAP_K);
  if (__GET_MOTOR_CTRL_MODE(DM_J4) == POS_LOOP)
    __SET_MOTOR_ANGLE(DM_J4, (HANDLER_PTR->joint_angle[HAND_J4] - J4_MAP_D) / J4_MAP_K);
  if (__GET_MOTOR_CTRL_MODE(DM_J5) == POS_LOOP)
    __SET_MOTOR_ANGLE(DM_J5, (HANDLER_PTR->joint_angle[HAND_J5] - J5_MAP_D) / J5_MAP_K);
  if (__GET_MOTOR_CTRL_MODE(dm_gripper) == POS_LOOP)
    __SET_MOTOR_ANGLE(dm_gripper, (HANDLER_PTR->joint_angle[HAND_G] - GR_MAP_D) / GR_MAP_K);
  
  // osDelay(5);//防止挂载过多发送通信堵塞
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

void basic_motor_init(void)
{
  // 电机初始化
  //  J1[M8010]
  //__SET_MOTOR_INSTANCE(M8010_J1,&joint1_motor);
  //__SET_MOTOR_TYPE(M8010_J1,M8010_MOTOR);
  //  M8010_motor_init(&joint1_motor,3,0.76,0.088);

  // // J1[AK]
  // __SET_MOTOR_INSTANCE(AK_J1, &AK70_10_motor);
  // __SET_MOTOR_TYPE(AK_J1, AK_MOTOR);
  // AK_joint_motor_init(&AK70_10_motor, 93);
  // AK_joint_motor_enable(&AK70_10_motor);

  // J1
  __SET_MOTOR_INSTANCE(DM_J1, &DM_Motor_J1);
  __SET_MOTOR_TYPE(DM_J1, M4310_MOTOR);
  joint_motor_init(&DM_Motor_J1, 1, POS_MODE, 0.55, 1.2);


  // J2
  __SET_MOTOR_INSTANCE(DM_J2, &DM_Motor_J2);
  __SET_MOTOR_TYPE(DM_J2, M4310_MOTOR);
  joint_motor_init(&DM_Motor_J2, 2, POS_MODE, 0.55, 1.2);

  // J3
  __SET_MOTOR_INSTANCE(DM_J3, &DM_Motor_J3);
  __SET_MOTOR_TYPE(DM_J3, M4310_MOTOR);
  joint_motor_init(&DM_Motor_J3, 3, POS_MODE, 0.55, 1.2);

  // // 之前j3的3508
  // __SET_MOTOR_INSTANCE(DJI_2006_J4, &DJI_Motor_J4);
  // __SET_MOTOR_TYPE(DJI_2006_J4, DJI_MOTOR);
  // DJI_Motor_init(&DJI_Motor_J4, &DJI_CAN2_Bus_ctrl, M2006, 0x204);
  // // DJI_Motor_set_angle_limit()
  // // DJI_Motor_set_speed_limit()
  // DJI_Motor_Speed_PID_init(&DJI_Motor_J4, PID_POSITION, 22, 0.000, 0.05, 6000.000, 800);
  // DJI_Motor_Pos_PID_init(&DJI_Motor_J4, PID_POSITION, 55, 0.0, 0.0, 200, 100);
  // // DJI_Motor_set_sum_angle(&DJI_Motor_J3);
  // DJI_Motor_set_multiple_circle_angle(&DJI_Motor_J4);

  // J4
  __SET_MOTOR_INSTANCE(DM_J4, &DM_Motor_J4);
  __SET_MOTOR_TYPE(DM_J4, M4310_MOTOR);
  joint_motor_init(&DM_Motor_J4, 6, POS_MODE, 0.55, 1.2);
  
  // J5
  __SET_MOTOR_INSTANCE(DM_J5, &DM_Motor_J5);
  __SET_MOTOR_TYPE(DM_J5, M4310_MOTOR);
  joint_motor_init(&DM_Motor_J5, 5, POS_MODE, 1.0, 1.0);

  // gripper
  __SET_MOTOR_INSTANCE(dm_gripper, &DM_Motor_gripper);
  __SET_MOTOR_TYPE(dm_gripper, M4310_MOTOR);
  joint_motor_init(&DM_Motor_gripper, 4, POS_MODE, 1.3, 1.0);

  // // j4
  // __SET_MOTOR_INSTANCE(DJI_2006_J4, &DJI_Motor_J4);
  // __SET_MOTOR_TYPE(DJI_2006_J4, DJI_MOTOR);
  // DJI_Motor_init(&DJI_Motor_J4, &DJI_CAN2_Bus_ctrl, M2006, 0x201);
  // // DJI_Motor_set_angle_limit();
  // // DJI_Motor_set_speed_limit();
  // DJI_Motor_Speed_PID_init(&DJI_Motor_J4, PID_POSITION, 15.3, 0.004, 0, 5500, 5500);
  // DJI_Motor_Pos_PID_init(&DJI_Motor_J4, PID_POSITION, 31.5, 0, 0, 2000, 1000);
  // DJI_Motor_set_multiple_circle_angle(&DJI_Motor_J4);

  // limit
  // dm电机有些上电后位置是2PI~0有些是-PI~PI是模式不同，可以在上位机中更改和设置0点，但都直接改数值也可以使用就懒得设置模式更改
 //先小范围测试
  __SET_JOINT_LIMIT(HAND_J1, -1.57f, 1.57f);
  __SET_JOINT_LIMIT(HAND_J2, -2.0f,-0.613f );       
  __SET_JOINT_LIMIT(HAND_J3, -6.0f, -2.93f); 
  __SET_JOINT_LIMIT(HAND_J4, -5.0f, 4.0f);          
  __SET_JOINT_LIMIT(HAND_J5, -1.57f, 1.57f);         
  __SET_JOINT_LIMIT(HAND_G, -3.14f, 3.14f);            

  __CLEAR_MOTOR_OFFLINE(DM_J1);
  __CLEAR_MOTOR_OFFLINE(DM_J2);
  __CLEAR_MOTOR_OFFLINE(DM_J3);
  __CLEAR_MOTOR_OFFLINE(DM_J4);
  __CLEAR_MOTOR_OFFLINE(DM_J5);
  __CLEAR_MOTOR_OFFLINE(dm_gripper);

  DJI_CANBus_enable_bus(&DJI_CAN1_Bus_ctrl);
  DJI_CANBus_enable_bus(&DJI_CAN2_Bus_ctrl);
  hand_task_get_feedback();

  for (uint8_t index = 0; index < HAND_MOTOR_COUNT; index++)
  {
    StallDetectEnable(index, 1);
  }
}
// void __J1_init(void)
// {
//   __hand_J1_init(1);
//   do
//   {
//     hand_task_get_feedback();
//     osDelay(1);
//   } while (!__hand_J1_init(0));

//   osDelay(10);
//   hand_task_get_feedback();
//   __hand_nonforce();
//   WS2812_Ctrl(30, 100, 50);
//   // buzzer_off();
// }

void __J1_init(void)
{
  while (!__hand_J1_init())
    hand_task_get_feedback();
  __DM_go_setting_angle(HAND_J1, 0, 0.00078f, 0.00043f);
}


void __J2_init(void)
{
  while (!__hand_J2_init())
    hand_task_get_feedback();
  __DM_go_setting_angle(HAND_J2, -1, 0.00078f, 0.00043f);
}

void __J3_init(void)
{
  while (!__hand_J3_init())
    hand_task_get_feedback();
  __DM_go_setting_angle(HAND_J3, -4, 0.00078f, 0.00043f);
}

void __J4_init(void)
{
  while (!__hand_J4_init())
    hand_task_get_feedback();
  __DM_go_setting_angle(HAND_J4, -4.85f, 2.0f, 0.00043f);
}



void __J5_init(void)
{
  while (!__hand_J5_init())
    hand_task_get_feedback();
  __DM_go_setting_angle(HAND_J5, 0, 0.00042f, 0.00023f);
}
void __gripper_init(void)
{
  while (!__hand_gripper_init())
    hand_task_get_feedback();
 __DM_go_setting_angle(HAND_G, 0, 0.00042f, 0.00023f);
}

void __DM_go_setting_angle(HAND_JOINT_INDEX index, float angle_value, float h_value, float l_value)
{
  // get current value and avoid perform go zero
  __SET_JOINT_ANGLE(index, __GET_JOINT_ANGLE(index));

  while (!__IS_JOINT_AROUND(index, angle_value))
  {
    hand_task_get_feedback();
    if (ABS(angle_value - HANDLER_PTR->joint_angle[index]) > 0.08f)
    {
      __ADD_JOINT_ANGLE(index, angle_value > HANDLER_PTR->joint_angle[index] ? h_value : -h_value);
    }
    else if (ABS(angle_value - HANDLER_PTR->joint_angle[index]) > 0.01f)
    {
      __ADD_JOINT_ANGLE(index, angle_value > HANDLER_PTR->joint_angle[index] ? l_value : -l_value);
    }
    hand_task_output();
    osDelay(1);
  }
  __hand_nonforce();
  hand_task_output();
}


uint8_t __hand_J1_init(void)
{ // if (DM_Motor_J2.para.state == 0x00)

  if (toe_is_error(TOE_J1) || DM_Motor_J1.para.state == 0x00)
  {
    __SET_MOTOR_CTRL_MODE(HAND_J1, NON_FORCE);
    disable_motor_mode(&hfdcan1, 1, POS_MODE);
    osDelay(5);
    enable_motor_mode(&hfdcan1, 1, POS_MODE);
    return 0;
  }
  return 1;
}

uint8_t __hand_J2_init(void)
{ // if (DM_Motor_J2.para.state == 0x00)

  if (toe_is_error(TOE_J2) || DM_Motor_J2.para.state == 0x00)
  {
    __SET_MOTOR_CTRL_MODE(HAND_J2, NON_FORCE);
    disable_motor_mode(&hfdcan1, 2, POS_MODE);
    osDelay(5);
    enable_motor_mode(&hfdcan1, 2, POS_MODE);
    return 0;
  }
  return 1;
}
uint8_t __hand_J3_init(void)
{
  if (toe_is_error(TOE_J3) || DM_Motor_J3.para.state == 0x00)
  {
    __SET_MOTOR_CTRL_MODE(HAND_J3, NON_FORCE);
    disable_motor_mode(&hfdcan1, 3, POS_MODE);
    osDelay(5);
    enable_motor_mode(&hfdcan1, 3, POS_MODE);
    return 0;
  }
  return 1;
}

uint8_t __hand_J4_init(void)
{ // if (DM_Motor_J2.para.state == 0x00)

  if (toe_is_error(TOE_J4) || DM_Motor_J4.para.state == 0x00)
  {
    __SET_MOTOR_CTRL_MODE(HAND_J4, NON_FORCE);
    disable_motor_mode(&hfdcan2, 6, POS_MODE);
    osDelay(5);
    enable_motor_mode(&hfdcan2, 6, POS_MODE);
    return 0;
  }
  return 1;
}

uint8_t __hand_J5_init(void)
{
  if (toe_is_error(TOE_J5) || DM_Motor_J5.para.state == 0x00)
  {
    __SET_MOTOR_CTRL_MODE(HAND_J5, NON_FORCE);
    disable_motor_mode(&hfdcan2, 5, POS_MODE);
    osDelay(5);
    enable_motor_mode(&hfdcan2, 5, POS_MODE);
    return 0;
  }
  return 1;
}
uint8_t __hand_gripper_init(void)
{
  if (toe_is_error(TOE_G) || DM_Motor_gripper.para.state == 0x00)
  {
    __SET_MOTOR_CTRL_MODE(HAND_G, NON_FORCE);
    // disable_motor_mode(&hfdcan2, 4, POS_MODE);
    osDelay(5);
    enable_motor_mode(&hfdcan2, 4, POS_MODE);
    return 0;
  }
  return 1;
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

  __ADD_JOINT_ANGLE(HAND_J1, 0);
  __ADD_JOINT_ANGLE(HAND_J2, 0);
  __ADD_JOINT_ANGLE(HAND_J3, 0);
  __ADD_JOINT_ANGLE(HAND_J4, 0);
  __ADD_JOINT_ANGLE(HAND_J5, 0);
  __ADD_JOINT_ANGLE(HAND_G, 0);

  // __hand_pitch_pos_init(0);
}

void __hand_rc_ctrl(void)
{
  __ADD_JOINT_ANGLE(HAND_J1, -RC_CTRL_PTR->rc.ch[0] * 0.0000007f);
  __ADD_JOINT_ANGLE(HAND_J2, -RC_CTRL_PTR->rc.ch[1] * 0.0000015f);
  __ADD_JOINT_ANGLE(HAND_J3, -RC_CTRL_PTR->rc.ch[2] * 0.0000007f);
  __ADD_JOINT_ANGLE(HAND_J4, -RC_CTRL_PTR->rc.ch[3] * 0.000060f);

  //  __ADD_JOINT_ANGLE(HAND_PITCH, GET_CH_VALUE(1)*0.0001f*PITCH_MAP_K);
  //  __ADD_JOINT_ANGLE(HAND_ROLL , GET_CH_VALUE(2)*0.0001f*ROLL_MAP_K);
  //  __ADD_JOINT_ANGLE(HAND_J1   ,-GET_CH_VALUE(3)*0.0000007f);
  //  __ADD_JOINT_ANGLE(HAND_J2   ,-GET_CH_VALUE(0)*0.0000015f);
  //  __ADD_JOINT_ANGLE(HAND_J3   ,GET_WHEEL_VALUE()*0.000025f*J3_MAP_K);
}
void __hand_rc2_ctrl(void)
{
  __ADD_JOINT_ANGLE(HAND_J5, -RC_CTRL_PTR->rc.ch[0] * 0.0000015f);
  __ADD_JOINT_ANGLE(HAND_G, -RC_CTRL_PTR->rc.ch[1] * 0.0000002f);
}

// 检测应当在模块层实现但由于检测是通过detect任务实现故放在应用层
void __detect_hand_motor_offline(void)
{
  // 掉电检测
  int index;
  for (index = 6; index < HAND_JOINT_COUNT + 6; index++) // toe_J1=6
  {
    if (toe_is_error(index))
    {
      __SET_MOTOR_CTRL_MODE(index - 6, OFFLINE);
    }

    // 掉线标志位用于应用层处理使用
    // 模式offline用于模块层处理使用

    //  加个按键按一次后置标志让他一直初始化
    // if (GET_KEY(KEY_R))//如果掉线时位置没有被改变虽然掉线标志位没有被清除，但任然能正常使用，可自主选择
    // {
    //   static count = 0;
    //   if(count == 200)//按下一段时间后才初始化
    //   {
    if (HANDLER_PTR->motor_ctrl_mode[index - 6] != OFFLINE && __IS_MOTOR_OFFLINE(index - 6) == 1)
    {
      __SET_MOTOR_CTRL_MODE(index - 6, NON_FORCE);
      if (__hand_motor_refresh_online(index - 6))
      {
        __CLEAR_MOTOR_OFFLINE(index - 6);
        // count = 0;
      }
    }
    //     }
    //     count++;
    //  }
  }
}

uint8_t __hand_motor_refresh_online(int index)
{
  switch (index) // 达妙电机失能了就重新使能读取位置，若原先就为使能(位置不会改变)，就直接退出
  {
  case dm_gripper:
    if (__hand_gripper_init())
      return 1;
    else
      return 0;
  case DM_J5:
    if (__hand_J5_init())
      return 1;
    else
      return 0;
  case DM_J4:
    __J4_init();
    return 1;
  case DM_J3:
    if (__hand_J3_init())
      return 1;
    else
      return 0;
  case DM_J2:
    if (__hand_J2_init())
      return 1;
    else
      return 0;
  case DM_J1:
    __J1_init();
    return 1;
  default:
    return 0;
  }
}
void __set_motor_offline_flag(void)
{
  int index;
  for (index = 0; index < HAND_JOINT_COUNT; index++)
  {
    if (HANDLER_PTR->motor_ctrl_mode[index] == OFFLINE)
    {
      __SET_MOTOR_OFFLINE(index);
    }
  }
}

void __detect_hand_motor_stall(void)
{
  int index;
  for (index = 0; index < HAND_JOINT_COUNT; index++)
  {
    if (toe_is_stall(index))
    {
      __SET_MOTOR_CTRL_MODE(index, STAll);
    }
  }
}

// 自定义控制器数据流在自定义芯片上处理
void __hand_custom_ctrl(void)
{
  // if (CC_handler.get_cc_data_flag)
  // {
  __hand_move2_subctrl(
      CC_handler.joint_angle[0],
      CC_handler.joint_angle[1],
      CC_handler.joint_angle[2],
      CC_handler.joint_angle[3],
      CC_handler.joint_angle[4],
      CC_handler.G_angle,
      J1_EN | J2_EN | J3_EN | J4_EN | J5_EN | JG_EN);
  //   CC_handler.get_cc_data_flag = 0;
  // }
  // else
  //   __hand_idle_ctrl();
}

/**/
void __hand_move2_subctrl(fp32 J1, fp32 J2, fp32 J3, fp32 J4, fp32 J5, fp32 JG, uint8_t EN)
{
  if (EN & JG_EN)
  {
    __hand_motor_go_setting_angle(HAND_G, JG, 0.08f, 0.01f, 0.0023f, 0.0008f);
  }
  if (EN & J5_EN)
  {
    //__hand_motor_go_setting_angle(HAND_J5, J5, 0.08f, 0.01f, 0.0023f, 0.001f);
    __SET_JOINT_ANGLE(HAND_J5, J5);
  }

  if (EN & J4_EN)
  {
    __hand_motor_go_setting_angle(HAND_J4, J4, 2.5f, 1.2f, 0.032f, 0.02f);
  }

  if (EN & J3_EN)
  {
    __SET_JOINT_ANGLE(HAND_J3, J3);
  }

  if (EN & J2_EN)
  {
    //__hand_motor_go_setting_angle(HAND_J2, J2, 0.08f, 0.01f, 0.023f, 0.01f);
    __SET_JOINT_ANGLE(HAND_J2, J2);
  }

  if (EN & J1_EN)
  {
    //(HAND_J1, J1, 0.08f, 0.01f, 0.0023f, 0.0008f);
    __SET_JOINT_ANGLE(HAND_J1, J1);
  }
}

void __hand_motor_go_setting_angle(HAND_JOINT_INDEX index, float angle_value, float h_dead, float l_dead, float h_value, float l_value)
{
  if (ABS(angle_value - HANDLER_PTR->joint_angle[index]) > h_dead)
  {
    __ADD_JOINT_ANGLE(index, angle_value > HANDLER_PTR->joint_angle[index] ? h_value : -h_value);
  }
  else if (ABS(angle_value - HANDLER_PTR->joint_angle[index]) > l_dead)
  {
    __ADD_JOINT_ANGLE(index, angle_value > HANDLER_PTR->joint_angle[index] ? l_value : -l_value);
  }
  else
  {
    __SET_JOINT_ANGLE(index, HANDLER_PTR->feedback_joint_angle[index]);
  }
}

// void __hand_catch_ground(void)
//{
//   //__hand_move2_subctrl(-PI / 4, -PI * 2 / 3, 0.0f, __GET_JOINT_MAX_LIM(HAND_PITCH), 0.0f, J1_EN | J2_EN | J3_EN | J4_EN);
// }

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

void __hand_move_GSM(void)
{
  if (get_step() == GSM_uplift_to_higher)
  {
    if (is_angle_around(GSM_STEP1_G_ANGLE, __GET_JOINT_ANGLE(HAND_G), 0.03f) &&
        is_angle_around(GSM_STEP1_J5_ANGLE, __GET_JOINT_ANGLE(HAND_J5), 0.1f) &&
        is_angle_around(GSM_STEP1_J4_ANGLE, __GET_JOINT_ANGLE(HAND_J4), 3.5f) &&
        is_angle_around(GSM_STEP1_J3_ANGLE, __GET_JOINT_ANGLE(HAND_J3), 0.1f)) // 如果到达目标位置
      movement.hand_step_complete = 1;
    else
      __hand_move2_subctrl(0.0f, 0.0f, GSM_STEP1_J3_ANGLE, GSM_STEP1_J4_ANGLE, GSM_STEP1_J5_ANGLE, GSM_STEP1_G_ANGLE, J3_EN | J4_EN | J5_EN | JG_EN);

    if (movement.hand_step_complete == 1 && movement.height_step_complete == 1)
    {
      next_step();
      movement.hand_step_complete = 0;
      movement.height_step_complete = 0;
    }
  }

  if (get_step() == GSM_hand_to_pos1)
  {
    if (movement.mine_place == RIGHT)
    {
      if (
          is_angle_around(GSM_STEP2_J2_ANGLE_RIGHT, __GET_JOINT_ANGLE(HAND_J2), 0.08f) &&
          is_angle_around(GSM_STEP2_J1_ANGLE_RIGHT, __GET_JOINT_ANGLE(HAND_J1), 0.06f))
      {
        osDelay(1000); // 等待j1稳定
        next_step();
      }
      else
        __hand_move2_subctrl(GSM_STEP2_J1_ANGLE_RIGHT, GSM_STEP2_J2_ANGLE_RIGHT, 0.0f, 0.0f, 0.0f, 0.0f, J1_EN | J2_EN);
    }
    else if (movement.mine_place == LEFT)
    {
      if (
          is_angle_around(GSM_STEP2_J2_ANGLE_LEFT, __GET_JOINT_ANGLE(HAND_J2), 0.08f) &&
          is_angle_around(GSM_STEP2_J1_ANGLE_LEFT, __GET_JOINT_ANGLE(HAND_J1), 0.06f))
      {
        osDelay(1000); // 等待j1稳定
        next_step();
      }
      else
        __hand_move2_subctrl(GSM_STEP2_J1_ANGLE_LEFT, GSM_STEP2_J2_ANGLE_LEFT, 0.0f, 0.0f, 0.0f, 0.0f, J1_EN | J2_EN);
    }
  }
  if (get_step() == GSM_hand_to_pos2)
  {
    if (movement.mine_place == RIGHT)
    {
      if (
          is_angle_around(GSM_STEP3_J2_ANGLE_RIGHT, __GET_JOINT_ANGLE(HAND_J2), 0.08f) &&
          is_angle_around(GSM_STEP3_J1_ANGLE_RIGHT, __GET_JOINT_ANGLE(HAND_J1), 0.06f))
        next_step();

      else
        __hand_motor_go_setting_angle(HAND_J2, GSM_STEP3_J2_ANGLE_RIGHT, 0.08f, 0.01f, 0.008f, 0.0005f);
      __SET_JOINT_ANGLE(HAND_J1, GSM_STEP3_J1_ANGLE_RIGHT);
    }
    else if (movement.mine_place == LEFT)
    {
      if (
          is_angle_around(GSM_STEP3_J2_ANGLE_LEFT, __GET_JOINT_ANGLE(HAND_J2), 0.08f) &&
          is_angle_around(GSM_STEP3_J1_ANGLE_LEFT, __GET_JOINT_ANGLE(HAND_J1), 0.06f))
        next_step();

      else
        __hand_motor_go_setting_angle(HAND_J2, GSM_STEP3_J2_ANGLE_LEFT, 0.08f, 0.01f, 0.008f, 0.0005f);
      __SET_JOINT_ANGLE(HAND_J1, GSM_STEP3_J1_ANGLE_LEFT);
    }
  }
  if (get_step() == GSM_hand_to_pos3)
  {
    if (movement.mine_place == RIGHT)
    {
      if (
          is_angle_around(GSM_STEP4_J2_ANGLE_RIGHT, __GET_JOINT_ANGLE(HAND_J2), 0.08f) &&
          is_angle_around(GSM_STEP4_J1_ANGLE_RIGHT, __GET_JOINT_ANGLE(HAND_J1), 0.06f))
        next_step();

      else
        __hand_motor_go_setting_angle(HAND_J2, GSM_STEP4_J2_ANGLE_RIGHT, 0.08f, 0.01f, 0.008f, 0.0005f);
      __SET_JOINT_ANGLE(HAND_J1, GSM_STEP4_J1_ANGLE_RIGHT);
    }
    else if (movement.mine_place == LEFT)
    {
      if (
          is_angle_around(GSM_STEP4_J2_ANGLE_LEFT, __GET_JOINT_ANGLE(HAND_J2), 0.08f) &&
          is_angle_around(GSM_STEP4_J1_ANGLE_LEFT, __GET_JOINT_ANGLE(HAND_J1), 0.06f))
        next_step();

      else
        __hand_motor_go_setting_angle(HAND_J2, GSM_STEP4_J2_ANGLE_LEFT, 0.08f, 0.01f, 0.008f, 0.0005f);
      __SET_JOINT_ANGLE(HAND_J1, GSM_STEP4_J1_ANGLE_LEFT);
    }
  }
  if (get_step() == GSM_hand_gri_close)
  {
    if (
        is_angle_around(GSM_STEP5_G_ANGLE, __GET_JOINT_ANGLE(HAND_G), 0.03f))
      next_step();
    else
      __hand_move2_subctrl(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, GSM_STEP5_G_ANGLE, JG_EN);
  }
}

void __hand_move_BTD(void)
{
  if (get_step() == BTD_hand_to_pos)
  {
    if (is_angle_around(BTD_STEP1_G_ANGLE, __GET_JOINT_ANGLE(HAND_G), 0.03f) &&
        is_angle_around(BTD_STEP1_J5_ANGLE, __GET_JOINT_ANGLE(HAND_J5), 0.1f) &&
        is_angle_around(BTD_STEP1_J4_ANGLE, __GET_JOINT_ANGLE(HAND_J4), 3.5f) &&
        is_angle_around(BTD_STEP1_J3_ANGLE, __GET_JOINT_ANGLE(HAND_J3), 0.08f)) // 如果到达目标位置
      next_step();
    else
      __hand_move2_subctrl(0.0f, 0.0f, BTD_STEP1_J3_ANGLE, BTD_STEP1_J4_ANGLE, BTD_STEP1_J5_ANGLE, BTD_STEP1_G_ANGLE, J3_EN | J4_EN | J5_EN | JG_EN);
  }
  if (get_step() == BTD_uplift_to_pos)
  {
    if (
        is_angle_around(BTD_STEP2_J2_ANGLE, __GET_JOINT_ANGLE(HAND_J2), 0.06f) &&
        is_angle_around(BTD_STEP2_J1_ANGLE, __GET_JOINT_ANGLE(HAND_J1), 0.06f)) // 如果到达目标位置
      movement.hand_step_complete = 1;

    else
      __hand_move2_subctrl(BTD_STEP2_J1_ANGLE, BTD_STEP2_J2_ANGLE, 0.0f, 0.0f, 0.0f, 0.0f, J1_EN | J2_EN);

    if (movement.hand_step_complete == 1 && movement.height_step_complete == 1)
    {
      next_step();
      movement.hand_step_complete = 0;
      movement.height_step_complete = 0;
    }
  }
}

// 退出固定模式后直接设置为自定义模式就回到原来位置了
void __hand_move_OCSM(void)
{
  if (get_step() == SM_uplift_to_pos)
  {
    if (is_angle_around(SM_STEP1_G_ANGLE, __GET_JOINT_ANGLE(HAND_G), 0.03f) &&
        is_angle_around(SM_STEP1_J5_ANGLE, __GET_JOINT_ANGLE(HAND_J5), 0.1f) &&
        is_angle_around(SM_STEP1_J4_ANGLE, __GET_JOINT_ANGLE(HAND_J4), 3.5f) &&
        is_angle_around(SM_STEP1_J3_ANGLE, __GET_JOINT_ANGLE(HAND_J3), 0.08f)) // 如果到达目标位置
      movement.hand_step_complete = 1;
    else
      __hand_move2_subctrl(0.0f, 0.0f, SM_STEP1_J3_ANGLE, SM_STEP1_J4_ANGLE, SM_STEP1_J5_ANGLE, SM_STEP1_G_ANGLE, J3_EN | J4_EN | J5_EN | JG_EN);

    if (movement.hand_step_complete == 1 && movement.height_step_complete == 1)
    {
      next_step();
      movement.hand_step_complete = 0;
      movement.height_step_complete = 0;
    }
  }

  if (get_step() == SM_hand_to_pos)
  {
    if (movement.mine_place == RIGHT)
    {
      if (
          is_angle_around(SM_STEP2_J2_ANGLE_RIGHT, __GET_JOINT_ANGLE(HAND_J2), 0.1f) &&
          is_angle_around(SM_STEP2_J1_ANGLE_RIGHT, __GET_JOINT_ANGLE(HAND_J1), 0.06f)) // 如果到达目标位置
      {
        osDelay(1000); // 等待j1稳定
        next_step();
      }
      else
        __hand_move2_subctrl(SM_STEP2_J1_ANGLE_RIGHT, SM_STEP2_J2_ANGLE_RIGHT, 0.0f, 0.0f, 0.0f, 0.0f, J1_EN | J2_EN);
    }
    else if (movement.mine_place == LEFT)
    {
      if (
          is_angle_around(SM_STEP2_J2_ANGLE_LEFT, __GET_JOINT_ANGLE(HAND_J2), 0.1f) &&
          is_angle_around(SM_STEP2_J1_ANGLE_LEFT, __GET_JOINT_ANGLE(HAND_J1), 0.06f)) // 如果到达目标位置
      {
        osDelay(1000); // 等待j1稳定
        next_step();
      }
      else
        __hand_move2_subctrl(SM_STEP2_J1_ANGLE_LEFT, SM_STEP2_J2_ANGLE_LEFT, 0.0f, 0.0f, 0.0f, 0.0f, J1_EN | J2_EN);
    }
  }

  if (get_step() == SM_hand_gri_open)
  {
    if (
        is_angle_around(SM_STEP4_JG_ANGLE, __GET_JOINT_ANGLE(HAND_G), 0.1f))
      next_step();
    else
      __hand_move2_subctrl(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, SM_STEP4_JG_ANGLE, JG_EN);
  }

  if (get_step() == SM_hand_out)
  {
    if (movement.mine_place == RIGHT)
    {
      if (
          is_angle_around(SM_STEP5_J2_ANGLE_RIGHT, __GET_JOINT_ANGLE(HAND_J2), 0.1f) &&
          is_angle_around(SM_STEP5_J1_ANGLE_RIGHT, __GET_JOINT_ANGLE(HAND_J1), 0.06f))
      {
        next_step();
        movement.mine_place = NON_DEIR;
      }
      else
        __hand_move2_subctrl(SM_STEP5_J1_ANGLE_RIGHT, SM_STEP5_J2_ANGLE_RIGHT, 0.0f, 0.0f, 0.0f, 0.0f, J1_EN | J2_EN);
    }
    else if (movement.mine_place == LEFT)
    {
      if (
          is_angle_around(SM_STEP5_J2_ANGLE_LEFT, __GET_JOINT_ANGLE(HAND_J2), 0.1f) &&
          is_angle_around(SM_STEP5_J1_ANGLE_LEFT, __GET_JOINT_ANGLE(HAND_J1), 0.05f))
      {
        next_step();
        movement.mine_place = NON_DEIR;
      }
      else
        __hand_move2_subctrl(SM_STEP5_J1_ANGLE_LEFT, SM_STEP5_J2_ANGLE_LEFT, 0.0f, 0.0f, 0.0f, 0.0f, J1_EN | J2_EN);
    }
  }

  // if (movement.hand_move_out_flag == 1) // 用于出错时紧急退出
  // {
  //   set_movement(0); // 退出固定动作
  //   movement.hand_move_out_flag = 0;
  // }
}

// 如果j1电机为双编码能实现掉电仍能获取位置可使用
//   uint8_t __hand_J1_init(uint8_t reset)
//   {
//     if (reset)
//     {
//     }
//       if (toe_is_error(TOE_J1))
//         return 0;

//      if ((__GET_JOINT_ANGLE(HAND_J1) - 0.0f <= 0.02f))
//        return 1;
//       else if (( __GET_JOINT_ANGLE(HAND_J1))- 0.0f >0.02f  && (!toe_is_error(TOE_J1)))
//         {
//           if ((__GET_JOINT_ANGLE(HAND_J1)) < HANDLER_PTR->max_joint_angle[HAND_J1] && (__GET_JOINT_ANGLE(HAND_J1)) >= 0.3f)
//           {
//             AK_joint_motor_speed_ctrl(__GET_MOTOR_INSTANCE(AK_J1), -1000);
//             return 0;
//           }
//           else if ((__GET_JOINT_ANGLE(HAND_J1)) > HANDLER_PTR->min_joint_angle[HAND_J1] && (__GET_JOINT_ANGLE(HAND_J1)) < 0.3f)
//           {
//             AK_joint_motor_speed_ctrl(__GET_MOTOR_INSTANCE(AK_J1), -400);
//             return 0;
//           }
//           else{
//             __hand_nonforce();
//             hand_task_output();
//             return 0;
//           }
//         }
//         else
//         {
//           __hand_nonforce();
//           hand_task_output();
//           return 0;
//         }
//     return 0;
// }

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
  参拜学长的吉祥物
  */

#undef HANDLER
#undef HANDLER_PTR
