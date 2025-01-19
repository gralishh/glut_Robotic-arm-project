/**
 * @brief 
 * @attention lockup的逻辑大抵没有大🐱饼
 */
#include "catcher_task_interface.h"
#include "struct_typedef.h"
#include "catcher_task.h"
#include "general_motor_module.h" 
#include "DJI_motor_canbus.h"
#include "remote_control.h" 
#include "angle_process.h"
#include "detect_task.h"
#include "cmsis_os2.h"
#include "ws2812.h"
#include "Custom_ctrl.h"

#define HANDLER catcher_task_handler
#define HANDLER_PTR catcher_task_handler_ptr
#define RC_CTRL_PTR (get_remote_control_point())

/*ABS*/
#define ABS(X) ((X)>0?(X):-(X))
/*global macro variable*/
// joint mapping parameter
#define UL_MAP_K   1
#define UL_MAP_D   0
#define PUSH_MAP_K 1
#define PUSH_MAP_D 0
#define PU_MAP_K   1
#define PU_MAP_D   0
// controller sensity(degree per loop)
#define UL_CTRL_SEN   0
#define PUSH_CTRL_SEN 0
#define PICK_CTRL_SEN   0

#define UL_EN (0x01<<0)
#define PUSH_EN (0x01<<0)
#define PICK_EN (0x01<<0)

/*global motor handler*/
DJI_Motor_Ctrl_t DJI_Motor_cat_uplift;
DJI_Motor_Ctrl_t DJI_Motor_cat_push;
DJI_Motor_Ctrl_t DJI_Motor_cat_pickup;

/*global variable*/

static void __catcher_nonforce(void);
static void __catcher_idle_ctrl(void);
static void __catcher_rc_ctrl(void);
static void __catcher_pose_ctrl(void);

/*general handler method*/
/** 
 * macro name format:
 *  __<GET/SET>_<MOTOR/JOINT>_<ITEM>(index[,value])
 */
/*获取电机状态*/
#define __GET_MOTOR_INSTANCE(index) (HANDLER_PTR->motor_instance[index])
#define __SET_MOTOR_INSTANCE(index,instance_ptr) (HANDLER_PTR->motor_instance[index]=((void*)instance_ptr))
#define __GET_STRUCT_MODE() (HANDLER_PTR->ctrl_mode)
#define __SET_STRUCT_MODE(value) (HANDLER_PTR->ctrl_mode=value)

#define __GET_MOTOR_TYPE(index) (HANDLER_PTR->motor_type[index])
#define __SET_MOTOR_TYPE(index,type) (HANDLER_PTR->motor_type[index]=(type))
#define __GET_MOTOR_CTRL_MODE(index) (HANDLER_PTR->motor_ctrl_mode[index])

/*获取电机反馈*/
#define __GET_MOTOR_ANGLE(index) (HANDLER_PTR->feedback_motor_angle[index])
#define __GET_MOTOR_SPEED(index) (HANDLER_PTR->feedback_motor_speed[index])
#define __GET_MOTOR_CURRENT(index) (HANDLER_PTR->feedback_motor_current[index])

/*电机输出控制*/
#define __SET_MOTOR_ANGLE(index,value) {(HANDLER_PTR->motor_angle[index]=value);(HANDLER_PTR->motor_ctrl_mode[index]=POS_LOOP);}
#define __ADD_MOTOR_ANGLE(index,value) {(HANDLER_PTR->motor_angle[index]+=value);(HANDLER_PTR->motor_ctrl_mode[index]=POS_LOOP);}
#define __SET_MOTOR_SPEED(index,value) {(HANDLER_PTR->motor_speed[index]=value);(HANDLER_PTR->motor_ctrl_mode[index]=SPEED_LOOP);}
#define __SET_MOTOR_CURRENT(index,value) {(HANDLER_PTR->motor_current[index]=value);(HANDLER_PTR->motor_ctrl_mode[index]=GIVING_CURRENT);}
#define __SET_MOTOR_LOCKUP(index) (HANDLER_PTR->motor_ctrl_mode[index]=LOCK)
#define __SET_MOTOR_NONFORCE(index) (HANDLER_PTR->motor_ctrl_mode[index]=NON_FORCE)

/*关节控制*/
#define __GET_JOINT_ANGLE(index) (HANDLER_PTR->feedback_joint_angle[index])
#define __SET_JOINT_LIMIT(index,min,max) {(HANDLER_PTR->max_joint_angle[index]=max);(HANDLER_PTR->min_joint_angle[index]=min);}
#define __JOINT_LIMIT(index,value) angle_limit(value,HANDLER_PTR->max_joint_angle[index],HANDLER_PTR->min_joint_angle[index])
#define __SET_JOINT_ANGLE(index,value) {(HANDLER_PTR->joint_angle[index]=__JOINT_LIMIT(index,value));(HANDLER_PTR->motor_ctrl_mode[index]=POS_LOOP);}
#define __ADD_JOINT_ANGLE(index,value) {(HANDLER_PTR->joint_angle[index]=__JOINT_LIMIT(index,HANDLER_PTR->joint_angle[index]+(value)));(HANDLER_PTR->motor_ctrl_mode[index]=POS_LOOP);}

/*时间控制*/
#define __RESET_TICKS() (HANDLER_PTR->tick=0)
#define __HALT_TICKS_COUNTING() (HANDLER_PTR->tick_count_halt=1)
#define __HOLD_TICKS_COUNTING() (HANDLER_PTR->tick_count_halt=0)
#define __IS_TIMER_HALT() (1==HANDLER_PTR->tick_count_halt)
#define __GET_TICKS() (HANDLER_PTR->tick)
// unit:seconds
#define __GET_TICKS_TIME() (HANDLER_PTR->tick*1)
#define __GET_PROCESS_PERCENTAGE(PROCESS_TIME) (__GET_TICKS_TIME()/PROCESS_TIME)
#define __GET_TICKS_STACK(index) (HANDLER_PTR->tick_stack[index])
#define __RECORD_TICKS(index) (HANDLER_PTR->tick_stack[index]=__GET_TICKS_TIME())
#define __RESET_RECORD_TICKS(index) (HANDLER_PTR->tick_stack[index]=0)
#define __IS_MODE_SWITCHED() (1==HANDLER_PTR->mode_switch)


void catcher_task_init()
{
  osDelay(3000);

  /*基础初始化*/
  __HALT_TICKS_COUNTING();
  __RESET_TICKS();

  /*电机初始化*/
  __SET_MOTOR_INSTANCE(DJI_CAT_UL,&DJI_Motor_cat_uplift);
  __SET_MOTOR_TYPE(DJI_CAT_UL,DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_cat_uplift,&DJI_CAN3_Bus_ctrl,M3508,0x201);
  DJI_Motor_Speed_PID_init(&DJI_Motor_cat_uplift,PID_POSITION,15,0,0.001,9000,0);
  DJI_Motor_Pos_PID_init(&DJI_Motor_cat_uplift,PID_POSITION,40,0,0,500,0);
  DJI_Motor_cat_uplift.circle_count_flag=1;

  __SET_MOTOR_INSTANCE(DJI_CAT_PUSH,&DJI_Motor_cat_push);
  __SET_MOTOR_TYPE(DJI_CAT_PUSH,DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_cat_push,&DJI_CAN3_Bus_ctrl,M3508,0x202);
  DJI_Motor_Speed_PID_init(&DJI_Motor_cat_push,PID_POSITION,15,0,0.001,9000,0);
  DJI_Motor_Pos_PID_init(&DJI_Motor_cat_push,PID_POSITION,20,0,0,500,0);
  DJI_Motor_cat_push.circle_count_flag=1;

  __SET_MOTOR_INSTANCE(DJI_CAT_PICK,&DJI_Motor_cat_pickup);
  __SET_MOTOR_TYPE(DJI_CAT_PICK,DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_cat_pickup,&DJI_CAN3_Bus_ctrl,M3508,0x203);
  DJI_Motor_Speed_PID_init(&DJI_Motor_cat_pickup,PID_POSITION,15,0,0.001,9000,0);
  DJI_Motor_Pos_PID_init(&DJI_Motor_cat_pickup,PID_POSITION,20,0,0,500,0);
  DJI_Motor_cat_pickup.circle_count_flag=1;

  DJI_CANBus_enable_bus(&DJI_CAN3_Bus_ctrl);
}

/**
 * @brief 刷新句柄的反馈值
 * @details 遍历电机控制句柄，针对点击类型刷新反馈
 * @code 
 */
void catcher_task_get_feedback()
{
  uint16_t index;

  /*motor feedback*/
  for(index=0;index<CATCHER_MOTOR_COUNT;index++)
  {
    GENERAL_MOTOR_GET_FEEDBACK(__GET_MOTOR_INSTANCE(index),
      __GET_MOTOR_TYPE(index),
      &__GET_MOTOR_CURRENT(index),
      &__GET_MOTOR_SPEED(index),
      &__GET_MOTOR_ANGLE(index)
    )
  }

  /*joint angle map*/
  __GET_JOINT_ANGLE(CAT_UPLIFT)=UL_MAP_K*__GET_MOTOR_ANGLE(DJI_CAT_UL) + UL_MAP_D;
  __GET_JOINT_ANGLE(CAT_PUSHOUT)=PUSH_MAP_K*__GET_MOTOR_ANGLE(DJI_CAT_PUSH) + PUSH_MAP_D;
  __GET_JOINT_ANGLE(CAT_PICKUP)=PU_MAP_K*__GET_MOTOR_ANGLE(DJI_CAT_PICK) + PU_MAP_D;
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
void catcher_task_mode_flush()
{
  static uint8_t last_mode=CATCHER_MODE_NONFORCE;
  last_mode=HANDLER_PTR->ctrl_mode;
  if(switch_is_mid(get_remote_control_point()->rc.s[1]))
  {
    if(switch_is_down(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(CATCHER_MODE_IDLE);
    else if(switch_is_mid(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(CATCHER_MODE_IDLE);
    else if(switch_is_up(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(CATCHER_MODE_RC_CTRL);
      //__SET_STRUCT_MODE(CATCHER_MODE_POSE_CTRL);
  }
  else
  {
    __SET_STRUCT_MODE(CATCHER_MODE_IDLE);
  }

  if(switch_is_down(get_remote_control_point()->rc.s[1]) && switch_is_down(get_remote_control_point()->rc.s[0]))
  {
    __SET_STRUCT_MODE(CATCHER_MODE_NONFORCE);
  }

	if(toe_is_error(DBUSTOE))
  {
    __SET_STRUCT_MODE(CATCHER_MODE_NONFORCE);
  }

  if(HANDLER_PTR->ctrl_mode==last_mode)
    HANDLER_PTR->mode_switch=0;
  else 
  {
    __RESET_TICKS();
    __HALT_TICKS_COUNTING();
    HANDLER_PTR->mode_switch=1;
  }
}

/**
 * @brief 设置输出量(电流|速度|位置|力矩)
 * @details 模式控制 
 */
void catcher_task_set_output()
{
  switch(__GET_STRUCT_MODE())
  {
    case CATCHER_MODE_IDLE:
      __catcher_idle_ctrl();
      break;
    case CATCHER_MODE_RC_CTRL:
      __catcher_rc_ctrl();
      break;
    case CATCHER_MODE_POSE_CTRL:
      __catcher_pose_ctrl();
      break;
    case CATCHER_MODE_NONFORCE:
    default:
      __catcher_nonforce();
  }
}

/**
 * @brief 控制输出
 * @details 根据电机种类与控制状态设定输出
 */
void catcher_task_output()
{
  uint16_t index;
  /*joint map to motor state*/
  if(__GET_MOTOR_CTRL_MODE(DJI_CAT_UL)==POS_LOOP)
    __SET_MOTOR_ANGLE(DJI_CAT_UL,(HANDLER_PTR->joint_angle[CAT_UPLIFT]-UL_MAP_D)/UL_MAP_K);
  if(__GET_MOTOR_CTRL_MODE(DJI_CAT_PUSH)==POS_LOOP)
    __SET_MOTOR_ANGLE(DJI_CAT_PUSH,(HANDLER_PTR->joint_angle[CAT_PUSHOUT]-PUSH_MAP_D)/PUSH_MAP_K);
  if(__GET_MOTOR_CTRL_MODE(DJI_CAT_PICK)==POS_LOOP)
    __SET_MOTOR_ANGLE(DJI_CAT_PICK,(HANDLER_PTR->joint_angle[CAT_PICKUP]-PU_MAP_D)/PU_MAP_K);

  /*motor output*/
  for(index=0;index<CATCHER_MOTOR_COUNT;index++)
  {
    GENERAL_MOTOR_SET_OUTPUT(__GET_MOTOR_INSTANCE(index),
    __GET_MOTOR_TYPE(index),
    __GET_MOTOR_CTRL_MODE(index),
    HANDLER_PTR->motor_current[index],
    HANDLER_PTR->motor_speed[index],
    HANDLER_PTR->motor_angle[index]
    )
  }
}

void __catcher_nonforce()
{
  int index;
  for(index=0;index<CATCHER_MOTOR_COUNT;index++)
  {
    __SET_JOINT_ANGLE(index,HANDLER_PTR->feedback_joint_angle[index]);// 设置关节输出值为当前关节角度
    __SET_MOTOR_NONFORCE(index);
  }
}

void __catcher_idle_ctrl()
{
  __ADD_JOINT_ANGLE(CAT_UPLIFT,0);
  __ADD_JOINT_ANGLE(CAT_UPLIFT,0);
  __ADD_JOINT_ANGLE(CAT_UPLIFT,0);

}

void __catcher_rc_ctrl()
{
  //__ADD_JOINT_ANGLE(CATCHER_PITCH,RC_CTRL_PTR->rc.ch[1]*0.0001f);
}

void __catcher_pose_ctrl(void)
{
  static uint8_t pose_mode=0;
  if(__IS_MODE_SWITCHED())
  {
    __RESET_TICKS();
    __HALT_TICKS_COUNTING();
    pose_mode=0;
  }

  /*Pose control command*/
  if(pose_mode==0)
  {
    if(RC_CTRL_PTR->rc.ch[3]==-660)
    {
      __HOLD_TICKS_COUNTING();
      if(__GET_TICKS_STACK(0)==0)
        __RECORD_TICKS(0);
      else if(__GET_TICKS_TIME()-__GET_TICKS_STACK(0)>1000)
        pose_mode=1;
    }
    else
    {
      __RESET_TICKS();
      __RESET_RECORD_TICKS(0);
      __HALT_TICKS_COUNTING();
    }
  }


  if(pose_mode==1)
    ;
    //__catcher_move2_subctrl(-PI/8,-2.2,0.5,0,0,J1_EN|J2_EN|J3_EN);
}


#undef HANDLER 
#undef HANDLER_PTR 
