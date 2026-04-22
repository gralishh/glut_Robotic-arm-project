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

#include "user_lib.h"
#include "main.h"
#include "Vofa.h"

// 分电板上对应底盘前左轮的can线线序与其他的can相反，大抵是画板的时候连错线了//已更换新分电板

// fp32 vofa_display_power[4] = {0};
// int16_t limit_flag = 0;
// fp32 vofa_display_power1[4] = {0};
// float total_power = 0;
// int16_t vofa_power_limit = 0; // vofac查看输出功率
// fp32 motor_current[4] = {0}; // 测试使用

#define HANDLER chassis_task_handler
#define HANDLER_PTR chassis_task_handler_ptr
#define RC_CTRL_PTR (get_remote_control_point())


/*global macro variable*/
// joint mapping parameter

// controller sensity(degree per loop)
#define VX_CTRL_SEN 4.23f*8/6
#define VY_CTRL_SEN 4.23f*8 / 6
#define WZ_CTRL_SEN 9.0f * 7 / 6

#define VX_ADD_SPEED_SEN 13.3f
#define VY_ADD_SPEED_SEN 13.3f
#define WZ_ADD_SPEED_SEN 15.0f
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
static void __chassis_KeybardMouse_rc_ctrl(void);
static void __chassis_union_rc_ctrl(void);
static void __detect_chassis_motor_offline(void);
void chassis_power_control_limit(void);
void chassis_power_control(void);
// void getrealpower(void);

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

#define __SET_MOTOR_CTRL_MODE(index, mode) (HANDLER_PTR->motor_ctrl_mode[index] = mode)
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
#define __ADD_MOTOR_SPEED(index, value)                 \
  {                                                     \
    (HANDLER_PTR->motor_speed[index] += value);         \
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
  DJI_Motor_Speed_PID_init(&DJI_Motor_LeftFront, PID_POSITION, 18, 0.000, 0.05, 16000, 1000);
  DJI_Motor_Pos_PID_init(&DJI_Motor_LeftFront, PID_POSITION, 15, 0, 0, 1000, 0);

  __SET_MOTOR_INSTANCE(DJI_RF, &DJI_Motor_RightFront);
  __SET_MOTOR_TYPE(DJI_RF, DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_RightFront, &DJI_CAN1_Bus_ctrl, M3508, 0x202);
  DJI_Motor_Speed_PID_init(&DJI_Motor_RightFront, PID_POSITION, 18, 0.000, 0.05, 16000, 1000);
  DJI_Motor_Pos_PID_init(&DJI_Motor_RightFront, PID_POSITION, 15, 0, 0, 1000, 0);

  __SET_MOTOR_INSTANCE(DJI_RB, &DJI_Motor_RightBack);
  __SET_MOTOR_TYPE(DJI_RB, DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_RightBack, &DJI_CAN1_Bus_ctrl, M3508, 0x203);
  DJI_Motor_Speed_PID_init(&DJI_Motor_RightBack, PID_POSITION, 18, 0.000, 0.05, 16000, 1000);
  DJI_Motor_Pos_PID_init(&DJI_Motor_RightBack, PID_POSITION, 15, 0, 0, 1000, 0);

  __SET_MOTOR_INSTANCE(DJI_LB, &DJI_Motor_LeftBack);
  __SET_MOTOR_TYPE(DJI_LB, DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_LeftBack, &DJI_CAN1_Bus_ctrl, M3508, 0x204);
  DJI_Motor_Speed_PID_init(&DJI_Motor_LeftBack, PID_POSITION, 22, 0.000, 0.05, 16000, 1000);
  DJI_Motor_Pos_PID_init(&DJI_Motor_LeftBack, PID_POSITION, 15, 0, 0, 1000, 0);

  __chassis_idle_ctrl();
  DJI_CANBus_enable_bus(&DJI_CAN1_Bus_ctrl);

  __CLEAR_MOTOR_OFFLINE(DJI_LF);
  __CLEAR_MOTOR_OFFLINE(DJI_RF);
  __CLEAR_MOTOR_OFFLINE(DJI_RB);
  __CLEAR_MOTOR_OFFLINE(DJI_LB);
  osDelay(20);
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
  // getrealpower();
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
      __SET_STRUCT_MODE(CHASSIS_MODE_IDLE);
    else
      __SET_STRUCT_MODE(CHASSIS_MODE_IDLE);
  }
  else if (switch_is_mid(get_remote_control_point()->rc.s[1]))
  {
    if (switch_is_mid(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(CHASSIS_MODE_UNION_CTRL);
    else if (switch_is_up(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(CHASSIS_MODE_KEYMOUSE_CTRL);
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
//这段代码和上方的功能重复了
  // if (switch_is_down(get_remote_control_point()->rc.s[1]) && switch_is_down(get_remote_control_point()->rc.s[0]))
  // {
  //   __SET_STRUCT_MODE(CHASSIS_MODE_NONFORCE);
  // }

  // 系统卡死、死锁时的恢复手段
 //或者死后重启不了就手动重启
    if ((GET_KEY(KEY_SHIFT) && GET_KEY(KEY_CTRL) && GET_KEY(KEY_R)))
    {
      __set_FAULTMASK(1);    //  系统复位
      NVIC_SystemReset();    // 重启系统
    }

  // {//{}限制static范围
  //   static uint16_t count = 0;
  //   if (count < 1000 && remote_data.pause)
  //     count++;
  //   if (count == 1000)
  //   {
  //     __set_FAULTMASK(1); //  系统复位
  //     NVIC_SystemReset(); // 重启系统
  //   }
  //   if (!remote_data.pause)
  //     count = 0;
  // }



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
  case CHASSIS_MODE_KEYMOUSE_CTRL:
    __chassis_KeybardMouse_rc_ctrl();
    break;
  case CHASSIS_MODE_NONFORCE:
  default:
    __chassis_nonforce();
  }

  chassis_power_control_limit();

  __detect_chassis_motor_offline();
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
      //__SET_MOTOR_ANGLE(index, __GET_MOTOR_ANGLE(index));
      __SET_MOTOR_SPEED(index, 0);
    }
  }

  // for (index = 0; index < CHASSIS_MOTOR_COUNT; index++)
  // {
  //   __ADD_MOTOR_ANGLE(index, 0);
  // }
}

void __chassis_rc_ctrl()
{
  /*设置输入速度(not real)*/
 
  HANDLER_PTR->vx = -RC_CTRL_PTR->rc.ch[3] * 8 / 6 * VX_CTRL_SEN;
  HANDLER_PTR->vy = -RC_CTRL_PTR->rc.ch[2] * 8 / 6 * VY_CTRL_SEN;
  HANDLER_PTR->wz = -RC_CTRL_PTR->rc.ch[0] * 7 / 6 * WZ_CTRL_SEN;

  // if (!remote_data.trigger)
  // {
  //   HANDLER_PTR->vx = -GET_CH_VALUE(2) * 9 / 6 * VX_CTRL_SEN;
  //   HANDLER_PTR->vy = -GET_CH_VALUE(3) * 9 / 6 * VY_CTRL_SEN;
  //   HANDLER_PTR->wz = -GET_CH_VALUE(0) * 9 / 6 * WZ_CTRL_SEN;

        __SET_MOTOR_SPEED(DJI_LF, -HANDLER_PTR->vx - HANDLER_PTR->vy + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
        __SET_MOTOR_SPEED(DJI_RF, HANDLER_PTR->vx - HANDLER_PTR->vy + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
        __SET_MOTOR_SPEED(DJI_RB, HANDLER_PTR->vx + HANDLER_PTR->vy + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
        __SET_MOTOR_SPEED(DJI_LB, -HANDLER_PTR->vx + HANDLER_PTR->vy + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  }

  void __chassis_KeybardMouse_rc_ctrl(void)
  {
    // vx
    if (GET_KEY(KEY_W))
    {
      HANDLER_PTR->vx += -VX_ADD_SPEED_SEN;
      HANDLER_PTR->vx = fp32_constrain(HANDLER_PTR->vx, -660 * VX_CTRL_SEN*8 / 6, 0);
    }
    else if (GET_KEY(KEY_S))
    {
      HANDLER_PTR->vx += VX_ADD_SPEED_SEN;
      HANDLER_PTR->vx = fp32_constrain(HANDLER_PTR->vx, 0, 660 * VX_CTRL_SEN * 8 / 6);
    }
    else
      HANDLER_PTR->vx += -sign(HANDLER_PTR->vx) * VX_ADD_SPEED_SEN;
    // HANDLER_PTR->vx *= 0.99f;

    // vy
    if (GET_KEY(KEY_A))
    {
      HANDLER_PTR->vy += VY_ADD_SPEED_SEN;
      HANDLER_PTR->vy = fp32_constrain(HANDLER_PTR->vy, 0, 660 * VY_CTRL_SEN * 2/3);
    }
    else if (GET_KEY(KEY_D))
    {
      HANDLER_PTR->vy += -VY_ADD_SPEED_SEN;
      HANDLER_PTR->vy = fp32_constrain(HANDLER_PTR->vy, -660 * VY_CTRL_SEN * 2/3, 0);
    }
    else
      HANDLER_PTR->vy += -sign(HANDLER_PTR->vy) * VY_ADD_SPEED_SEN;
    // HANDLER_PTR->vy *= 0.99f;

    // wz
    if (remote_data.mouse_x != 0)
    {
      HANDLER_PTR->wz += -remote_data.mouse_x * WZ_CTRL_SEN;
      HANDLER_PTR->wz = fp32_constrain(HANDLER_PTR->wz, -660 * WZ_CTRL_SEN , 660 * WZ_CTRL_SEN );
    }
    else if (GET_KEY(KEY_Q))
    {
      HANDLER_PTR->wz += WZ_ADD_SPEED_SEN;
      HANDLER_PTR->wz = fp32_constrain(HANDLER_PTR->wz, 0, 660 * WZ_CTRL_SEN * 1/2);
    }
    else if (GET_KEY(KEY_E))
    {
      HANDLER_PTR->wz += -WZ_ADD_SPEED_SEN;
      HANDLER_PTR->wz = fp32_constrain(HANDLER_PTR->wz, -660 * WZ_CTRL_SEN * 1/2, 0);
    }
    else
       HANDLER_PTR->wz += -sign(HANDLER_PTR->wz) * WZ_ADD_SPEED_SEN;
      //HANDLER_PTR->wz *= 0.99f;

    if (fabsf(HANDLER_PTR->vx) < 0.5f)
      HANDLER_PTR->vx = 0.0f;
    if (fabsf(HANDLER_PTR->vy) < 0.5f)
      HANDLER_PTR->vy = 0.0f;
    if (fabsf(HANDLER_PTR->wz) < 0.5f)
      HANDLER_PTR->wz = 0.0f;


    //}

    __SET_MOTOR_SPEED(DJI_LF, -HANDLER_PTR->vx - HANDLER_PTR->vy + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
    __SET_MOTOR_SPEED(DJI_RF, HANDLER_PTR->vx - HANDLER_PTR->vy + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
    __SET_MOTOR_SPEED(DJI_RB, HANDLER_PTR->vx + HANDLER_PTR->vy + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
    __SET_MOTOR_SPEED(DJI_LB, -HANDLER_PTR->vx + HANDLER_PTR->vy + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
}
    void __chassis_union_rc_ctrl()//暂未用到考虑删除
    {
      /*设置输入速度(not real)*/
      HANDLER_PTR->vx = -RC_CTRL_PTR->rc.ch[3] * VX_CTRL_SEN;
      HANDLER_PTR->vy = -RC_CTRL_PTR->rc.ch[2] * VY_CTRL_SEN;
      HANDLER_PTR->wz = 0;
      // vx
      if (GET_KEY(KEY_W))
      {
        HANDLER_PTR->vx += -VX_ADD_SPEED_SEN;
        HANDLER_PTR->vx = fp32_constrain(HANDLER_PTR->vx, -660 * VX_CTRL_SEN , 0);
      }
      else if (GET_KEY(KEY_S))
      {
        HANDLER_PTR->vx += VX_ADD_SPEED_SEN;
        HANDLER_PTR->vx = fp32_constrain(HANDLER_PTR->vx, 0, 660 * VX_CTRL_SEN );
      }
      else
        HANDLER_PTR->vx += -sign(HANDLER_PTR->vx) * VX_ADD_SPEED_SEN;
      // HANDLER_PTR->vx *= 0.99f;

      // xy
      if (GET_KEY(KEY_A))
      {
        HANDLER_PTR->vy += VY_ADD_SPEED_SEN;
        HANDLER_PTR->vy = fp32_constrain(HANDLER_PTR->vy, 0, 660 * VY_CTRL_SEN );
      }
      else if (GET_KEY(KEY_D))
      {
        HANDLER_PTR->vy += -VY_ADD_SPEED_SEN;
        HANDLER_PTR->vy = fp32_constrain(HANDLER_PTR->vy, -660 * VY_CTRL_SEN , 0);
      }
      else
        HANDLER_PTR->vy += -sign(HANDLER_PTR->vy) * VY_ADD_SPEED_SEN;
      // HANDLER_PTR->vy *= 0.99f;

      // wz
      if (GET_KEY(KEY_Q))
      {
        HANDLER_PTR->wz += WZ_ADD_SPEED_SEN;
        HANDLER_PTR->wz = fp32_constrain(HANDLER_PTR->wz, 0, 660 * WZ_CTRL_SEN );
      }
      else if (GET_KEY(KEY_E))
      {
        HANDLER_PTR->wz += -WZ_ADD_SPEED_SEN;
        HANDLER_PTR->wz = fp32_constrain(HANDLER_PTR->wz, -660 * WZ_CTRL_SEN , 0);
      }
      else
        HANDLER_PTR->wz += -sign(HANDLER_PTR->wz) * WZ_ADD_SPEED_SEN;
      // HANDLER_PTR->wz *= 0.99f;

      if (fabsf(HANDLER_PTR->vx) < 0.5f)
        HANDLER_PTR->vx = 0.0f;
      if (fabsf(HANDLER_PTR->vy) < 0.5f)
        HANDLER_PTR->vy = 0.0f;
      if (fabsf(HANDLER_PTR->wz) < 0.5f)
        HANDLER_PTR->wz = 0.0f;

      __SET_MOTOR_SPEED(DJI_LF, -HANDLER_PTR->vx * 0.5f - HANDLER_PTR->vy * 0.5f + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
      __SET_MOTOR_SPEED(DJI_RF, HANDLER_PTR->vx * 0.5f - HANDLER_PTR->vy * 0.5f + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
      __SET_MOTOR_SPEED(DJI_RB, HANDLER_PTR->vx * 0.5f + HANDLER_PTR->vy * 0.5f + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
      __SET_MOTOR_SPEED(DJI_LB, -HANDLER_PTR->vx * 0.5f + HANDLER_PTR->vy * 0.5f + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
    }

void __detect_chassis_motor_offline(void)
{
  // 掉电检测

  int index;
  for (index = 2; index < CHASSIS_MOTOR_COUNT+2; index++) 
  {
    if (toe_is_error(index))
    {
      __SET_MOTOR_CTRL_MODE(index-2, OFFLINE);
    }
    else
    {
      __CLEAR_MOTOR_OFFLINE(index-2);
    }
  }


  int count;
  for (count = 0; count < CHASSIS_MOTOR_COUNT; count++)
  {
    if (HANDLER_PTR->motor_ctrl_mode[count] == OFFLINE)
    {
      __SET_MOTOR_OFFLINE(count);
    }
  }
}
  

// void getrealpower(void)
//{
//	  float InitialGivePower[4]; // initial power from PID calculation

//  float toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
//  float k1 = 1.23e-07;                      // k1
//  float k2 = 1.453e-07;                     // k2
//  float constant = 4.081f;
//  // vofa展示
//  float vofa_InitialGivePower[4] = 0;
//  float vofa_InitialTotalPower = 0;

//  // 检测限制后的功率
//  for (uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
//  {
//    // vofa_display_power[i] = (int16_t)__GET_MOTOR_SPEED(i);//
//    // vofa_display_power[i] = __GET_SET_MOTOR_CURRENT(i);//
//    //__GET_MOTOR_SPEED(i) = 3000;//
//    vofa_InitialGivePower[i] = __GET_SET_MOTOR_CURRENT(i) * toque_coefficient * (int16_t)__GET_MOTOR_SPEED(i) +
//                               k2 * (int16_t)__GET_MOTOR_SPEED(i) * (int16_t)__GET_MOTOR_SPEED(i) +
//                               k1 * __GET_SET_MOTOR_CURRENT(i) * __GET_SET_MOTOR_CURRENT(i) + constant;

//    // 输入功率=机械功率+铜损+磁损+控制器静态功耗
//    // 机械功率=力矩电流控制值*（20/16384）*0.3*（187/3591）*转速/9.55
//    // 铜损和磁损分别为k2*转速^2和k1*力矩电流控制值^2
//    // 化为以下的关于out的二次方程
//    // k1 * out^2 + (toque_coefficient * speed_rpm) * out + (k2 * speed_rpm^2 + constant - ScaledGivePower[i]) = 0
//    // 通过对其的逆解算，解出pid的输出值从而对其进行限幅

//    // vofa_display_power[i] = InitialGivePower[i];

//    if (InitialGivePower < 0) // negative power not included (transitory)
//      continue;
//    vofa_InitialTotalPower += vofa_InitialGivePower[i];
//    total_power = vofa_InitialTotalPower; //
//  }
//}

//功率控制不能用的速度位置环来控制底盘
void chassis_power_control_limit(void)
{
  for (int8_t index = 0; index < CHASSIS_MOTOR_COUNT; index++)
  {
    __SET_MOTOR_CURRENT(index, PID_Calc(&(((DJI_Motor_Ctrl_t *)__GET_MOTOR_INSTANCE(index))->pid_speed_loop), __GET_MOTOR_SPEED(index), (fp32)__GET_SET_MOTOR_SPEED(index)));
  }
  chassis_power_control();
}
// 移植步兵功率控制
void chassis_power_control(void)
{

  uint16_t RefereePowerLimit = 115;
  float ChassisMaxPower = 0;

  float InitialGivePower[4]; // initial power from PID calculation
  float InitialTotalPower = 0;
  float ScaledGivePower[4];

  // float chassis_energy_buffer = 50.0f;

  float toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
  float k1 = 1.23e-07;                      // k1
  float k2 = 1.453e-07;                     // k2
                                            //  float constant = 4.081f;
  float constant = 10.081f;

  ChassisMaxPower = (float)RefereePowerLimit;

  for (uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
  {
    // vofa_display_power[i] = (int16_t)__GET_MOTOR_SPEED(i);//
    // vofa_display_power[i] = __GET_SET_MOTOR_CURRENT(i);//
    //__GET_MOTOR_SPEED(i) = 3000;//
    InitialGivePower[i] = __GET_SET_MOTOR_CURRENT(i) * toque_coefficient * (int16_t)__GET_MOTOR_SPEED(i) +
                          k2 * (int16_t)__GET_MOTOR_SPEED(i) * (int16_t)__GET_MOTOR_SPEED(i) +
                          k1 * __GET_SET_MOTOR_CURRENT(i) * __GET_SET_MOTOR_CURRENT(i) + constant;

    // 输入功率=机械功率+铜损+磁损+控制器静态功耗
    // 机械功率=力矩电流控制值*（20/16384）*0.3*（187/3591）*转速/9.55
    // 铜损和磁损分别为k2*转速^2和k1*力矩电流控制值^2
    // 化为以下的关于out的二次方程
    // k1 * out^2 + (toque_coefficient * speed_rpm) * out + (k2 * speed_rpm^2 + constant - ScaledGivePower[i]) = 0
    // 通过对其的逆解算，解出pid的输出值从而对其进行限幅

    // vofa_display_power[i] = InitialGivePower[i];

    if (InitialGivePower < 0) // negative power not included (transitory)
      continue;
    InitialTotalPower += InitialGivePower[i];
    // total_power = InitialTotalPower; //
  }

  if (InitialTotalPower > ChassisMaxPower) // determine if larger than max power
  {
    float power_scale = ChassisMaxPower / InitialTotalPower;

    for (uint8_t i = 0; i < 4; i++)
    {

      ScaledGivePower[i] = InitialGivePower[i] * power_scale; // get scaled power

      float b = toque_coefficient * (int16_t)__GET_MOTOR_SPEED(i);
      float c = k2 * (int16_t)__GET_MOTOR_SPEED(i) * (int16_t)__GET_MOTOR_SPEED(i) - ScaledGivePower[i] + constant;
      float inside = b * b - 4 * k1 * c;

      if (inside < 0)
      {
        continue;
      }
      // 判别式小于0则跳过该电机
      else if (__GET_SET_MOTOR_CURRENT(i) > 0) // Selection of the calculation formula according to the direction of the original moment
      {
        // limit_flag ++;//
        float temp = (-b + sqrt(inside)) / (2 * k1); // 用公式求出电机的输出值，对其进行限幅
        if (temp > 16000)
        {
          __GET_SET_MOTOR_CURRENT(i) = 16000;
        }
        else
        {
          __GET_SET_MOTOR_CURRENT(i) = temp;
          // vofa_display_power[i] = __GET_SET_MOTOR_CURRENT(i);//
          // vofa_display_power[i] = temp;//
        }
      }
      else
      {
        float temp = (-b - sqrt(inside)) / (2 * k1);
        if (temp < -16000)
        {
          __GET_SET_MOTOR_CURRENT(i) = -16000;
        }
        else
          __GET_SET_MOTOR_CURRENT(i) = temp;
      }
    }
  }
}
#undef HANDLER
#undef HANDLER_PTR
//防止其他文件也定义了相同名称的宏造成意外替换
//明确宏的作用范围仅限于当前文件内


