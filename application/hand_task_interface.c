#include "hand_task_interface.h"
#include "hand_task.h"
#include "general_motor_module.h" 
#include "remote_control.h" 

#define HANDLER hand_task_handler
#define HANDLER_PTR hand_task_handler_ptr

/*global macro variable*/
#define HAND_CTRL_CAN 

/*global motor handler*/
extern M8010_motor_t joint1_motor;
extern Joint_Motor_t DM_Motor_J2;
DJI_Motor_Ctrl_t DJI_Motor_J3;
DJI_Motor_Ctrl_t DJI_Motor_headendL;
DJI_Motor_Ctrl_t DJI_Motor_headendR;

static void __hand_nonforce();
static void __hand_idle_ctrl();
static void __hand_rc_ctrl();


/*general handler method*/
/** 
 * macro name format:
 *  __<GET/SET>_<MOTOR/JOINT>_<ITEM>(index[,value])
 */
#define __GET_MOTOR_INSTANCE(index) (HANDLER_PTR->motor_instance[index])
#define __SET_MOTOR_INSTANCE(index,instance_ptr) (HANDLER_PTR->motor_instance[index]=((void*)instance_ptr))
#define __GET_STRUCT_MODE() (HANDLER_PTR->ctrl_mode)
#define __SET_STRUCT_MODE(value) (HANDLER_PTR->ctrl_mode=value)

#define __GET_MOTOR_TYPE(index) (HANDLER_PTR->motor_type[index])
#define __SET_MOTOR_TYPE(index,type) (HANDLER_PTR->motor_type[index]=(type))
#define __GET_MOTOR_CTRL_MODE(index) (HANDLER_PTR->motor_ctrl_mode[index])

#define __GET_MOTOR_ANGLE(index) (HANDLER_PTR->feedback_motor_angle[index])
#define __GET_MOTOR_SPEED(index) (HANDLER_PTR->feedback_motor_speed[index])
#define __GET_MOTOR_CURRENT(index) (HANDLER_PTR->feedback_motor_current[index])

#define __SET_MOTOR_ANGLE(index,value) (HANDLER_PTR->motor_angle[index]=value)
#define __SET_MOTOR_SPEED(index,value) (HANDLER_PTR->motor_speed[index]=value)
#define __SET_MOTOR_CURRENT(index,value) (HANDLER_PTR->motor_current[index]=value)

#define __GET_JOINT_ANGLE(index) (HANDLER_PTR->feedback_joint_angle[index])
#define __SET_JOINT_ANGLE(index,value) (HANDLER_PTR->joint_angle[index]=value)


void hand_task_init()
{
  __SET_MOTOR_INSTANCE(M8010_J1,&joint1_motor);
  __SET_MOTOR_TYPE(M8010_J1,M8010_MOTOR);
  M8010_motor_init(&joint1_motor,1,10,0.075);

  __SET_MOTOR_INSTANCE(DM_J2,&DM_Motor_J2);
  __SET_MOTOR_TYPE(DM_J2,M4310_MOTOR);
  enable_motor_mode(hfdcan2,1,MIT_MODE);
  joint_motor_init(&DM_Motor_J2,1,MIT_MODE,10.0,1.0);

  __SET_MOTOR_INSTANCE(DJI_J3,&DJI_Motor_J3);
  __SET_MOTOR_TYPE(DJI_J3,DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_J3,&DJI_CAN2_Bus_ctrl,M3508,2);
  //DJI_Motor_set_angle_limit()
  //DJI_Motor_set_speed_limit()
  //DJI_Motor_Speed_PID_init()
  //DJI_Motor_Pos_PID_init()

  __SET_MOTOR_INSTANCE(DJI_HE_L,&DJI_Motor_headendL);
  __SET_MOTOR_TYPE(DJI_HE_L,DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_headendL,&DJI_CAN2_Bus_ctrl,M2006,3);

  __SET_MOTOR_INSTANCE(DJI_HE_R,&DJI_Motor_headendR);
  __SET_MOTOR_TYPE(DJI_HE_R,DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_headendR,&DJI_CAN2_Bus_ctrl,M2006,4);
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
  for(index=0;index<HAND_MOTOR_COUNT;index++)
  {
    GENERAL_MOTOR_GET_FEEDBACK(__GET_MOTOR_INSTANCE(index),
      __GET_MOTOR_TYPE(index),
      &__GET_MOTOR_CURRENT(index),
      &__GET_MOTOR_SPEED(index),
      &__GET_MOTOR_ANGLE(index)
    )
  }

  /*joint angle map*/
  __GET_JOINT_ANGLE(HAND_J1)=__GET_MOTOR_ANGLE(M8010_J1);
  __GET_JOINT_ANGLE(HAND_J2)=__GET_MOTOR_ANGLE(DM_J2);
  __GET_JOINT_ANGLE(HAND_J3)=__GET_MOTOR_ANGLE(DJI_J3);
  __GET_JOINT_ANGLE(HAND_PITCH);
  __GET_JOINT_ANGLE(HAND_ROLL);
  /*
  __GET_JOINT_ANLGE(index,
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
  /**/
  if(get_remote_control_point()->rc.s[0]==3)
  {
    if(get_remote_control_point()->rc.s[1]==1)
      __SET_STRUCT_MODE(HAND_MODE_IDLE);
    else if(get_remote_control_point()->rc.s[1]==2)
      __SET_STRUCT_MODE(HAND_MODE_RC_CTRL);
    else if(get_remote_control_point()->rc.s[1]==3)
      ;
  }

  if(get_remote_control_point()->rc.s[0]==1 && get_remote_control_point()->rc.s[1]==1)
  {
    __SET_STRUCT_MODE(HAND_MODE_NONFORCE);
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
 */
void hand_task_set_output()
{
  switch(__GET_STRUCT_MODE())
  {
    case HAND_MODE_IDLE:
      __hand_idle_ctrl();
      break;
    case HAND_MODE_RC_CTRL:
      __hand_rc_ctrl();
      break;
    case HAND_MODE_NONFORCE:
    default:
      __hand_nonforce();
  }
}

/**
 * @brief 控制输出
 * @details 根据电机种类与控制状态设定输出
 */
void hand_task_output()
{
  uint16_t index;
  /*joint map to motor state*/

  /*motor output*/
  for(index=0;index<HAND_MOTOR_COUNT;index++)
  {
    GENERAL_MOTOR_SET_OUTPUT(__GET_MOTOR_INSTANCE(index),
    __GET_MOTOR_TYPE(index),
    __GET_MOTOR_CTRL_MODE(index),
    __GET_MOTOR_CURRENT(index),
    __GET_MOTOR_SPEED(index),
    __GET_MOTOR_ANGLE(index)
    )
  }
}

void __hand_nonforce()
{

}

void __hand_idle_ctrl()
{

}

void __hand_rc_ctrl()
{

}
