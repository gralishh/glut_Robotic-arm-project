#include "chassis_task_interface.h"
#include "chassis_task.h"
#include "general_motor_module.h" 
#include "DJI_motor_canbus.h"
#include "remote_control.h" 
#include "angle_process.h"
#include "cmsis_os2.h"

#define HANDLER chassis_task_handler
#define HANDLER_PTR chassis_task_handler_ptr
#define RC_CTRL_PTR (get_remote_control_point())

/*extern*/
extern FDCAN_HandleTypeDef hfdcan1;

/*global macro variable*/
// joint mapping parameter
#define UL_MAP_K   1
#define UL_MAP_D   0
// controller sensity(degree per loop)
#define UL_CTRL_SEN 0

/*global motor handler*/
DJI_Motor_Ctrl_t DJI_Motor_uplift;

static void __chassis_nonforce(void);
static void __chassis_idle_ctrl(void);
static void __chassis_rc_ctrl(void);


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
#define __GET_TICKS() (HANDLER_PTR->tick)
// unit:seconds
#define __GET_TICKS_TIME() (HANDLER_PTR->tick*1)
#define __GET_PROCESS_PERCENTAGE(PROCESS_TIME) (__GET_TICKS_TIME()/PROCESS_TIME)


void chassis_task_init()
{
  /*基础初始化*/
  __HALT_TICKS_COUNTING();
  __RESET_TICKS();

  /*电机初始化*/
  __SET_MOTOR_INSTANCE(DJI_UL,&DJI_Motor_uplift);
  __SET_MOTOR_TYPE(DJI_UL,DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_uplift,&DJI_CAN1_Bus_ctrl,M3508,0x205);
  DJI_Motor_Speed_PID_init(&DJI_Motor_uplift,PID_POSITION,25,0,0.001,5000,0);
  DJI_Motor_Pos_PID_init(&DJI_Motor_uplift,PID_POSITION,50,0,0,500,0);
  DJI_Motor_uplift.circle_count_flag=1;

  __chassis_idle_ctrl();
  DJI_CANBus_enable_bus(&DJI_CAN1_Bus_ctrl);
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
  for(index=0;index<CHASSIS_MOTOR_COUNT;index++)
  {
    GENERAL_MOTOR_GET_FEEDBACK(__GET_MOTOR_INSTANCE(index),
      __GET_MOTOR_TYPE(index),
      &__GET_MOTOR_CURRENT(index),
      &__GET_MOTOR_SPEED(index),
      &__GET_MOTOR_ANGLE(index)
    )
  }

  /*joint angle map*/
  __GET_JOINT_ANGLE(CHASSIS_UPLIFT)=UL_MAP_K*__GET_MOTOR_ANGLE(DJI_UL) +UL_MAP_D;
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
  if(switch_is_mid(get_remote_control_point()->rc.s[1]))
  {
    if(switch_is_down(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(CHASSIS_MODE_IDLE);
    else if(switch_is_mid(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(CHASSIS_MODE_RC_CTRL);
    else if(switch_is_up(get_remote_control_point()->rc.s[0]))
      ;
  }
  else if(switch_is_mid(get_remote_control_point()->rc.s[1]) && switch_is_down(get_remote_control_point()->rc.s[0]))
  {
    __SET_STRUCT_MODE(CHASSIS_MODE_NONFORCE);
  }
  else
  {
    __SET_STRUCT_MODE(CHASSIS_MODE_IDLE);
  }
  /*
  if(get_remote_control_point()->rc.s[0]==0 && get_remote_control_point()->rc.s[1]==0)
  {
    __SET_STRUCT_MODE(first_mode);
  }
  */
}

/**
 * @brief 设置输出量(电流|速度|位置|力矩)
 * @details 模式控制 
 */
void chassis_task_set_output()
{
  switch(__GET_STRUCT_MODE())
  {
    case CHASSIS_MODE_IDLE:
      __chassis_idle_ctrl();
      break;
    case CHASSIS_MODE_RC_CTRL:
      __chassis_rc_ctrl();
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
  if(__GET_MOTOR_CTRL_MODE(DJI_UL)==POS_LOOP)
    __SET_MOTOR_ANGLE(DJI_UL,(HANDLER_PTR->joint_angle[DJI_UL]-UL_MAP_D)/UL_MAP_K);

  /*motor output*/
  for(index=0;index<CHASSIS_MOTOR_COUNT;index++)
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

void __chassis_nonforce()
{
  int index;
  for(index=0;index<CHASSIS_MOTOR_COUNT;index++)
  {
    __SET_JOINT_ANGLE(index,HANDLER_PTR->feedback_joint_angle[index]);// 设置关节输出值为当前关节角度
    __SET_MOTOR_NONFORCE(index);
  }
}

void __chassis_idle_ctrl()
{
  int index;
  for(index=0;index<CHASSIS_MOTOR_COUNT;index++)
  {
    __SET_JOINT_ANGLE(index,HANDLER_PTR->feedback_joint_angle[index]);// 设置关节输出值为当前关节角度
    __SET_MOTOR_LOCKUP(index);
  }
}

void __chassis_rc_ctrl()
{
  __ADD_JOINT_ANGLE(CHASSIS_UPLIFT,RC_CTRL_PTR->rc.ch[2]*0.00005f);
}

#undef HANDLER 
#undef HANDLER_PTR 

