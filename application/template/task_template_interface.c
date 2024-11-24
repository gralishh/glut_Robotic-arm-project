#include "task_template_interface.h"
#include "remote_control.h"

#define HANDLER task_handler
#define HANDLER_PTR task_handler_ptr

/*general handler method*/
/** 
 * macro name format:
 *  __<GET/SET>_<MOTOR/JOINT>_<ITEM>(index[,value])
 */
#define __GET_MOTOR_INSTANCE(index) (HANDLER_PTR->motor_instance[index])
#define __GET_STRUCT_MODE() (HANDLER_PTR->ctrl_mode)
#define __SET_STRUCT_MODE(value) (HANDLER_PTR->ctrl_mode=value)

#define __GET_MOTOR_TYPE(index) (HANDLER_PTR->motor_type[index])
#define __GET_MOTOR_CTRL_MODE(index) (HANDLER_PTR->motor_ctrl_mode[index])

#define __GET_MOTOR_ANGLE(index) (HANDLER_PTR->feedback_motor_angle[index])
#define __GET_MOTOR_SPEED(index) (HANDLER_PTR->feedback_motor_speed[index])
#define __GET_MOTOR_CURRENT(index) (HANDLER_PTR->feedback_motor_current[index])

#define __SET_MOTOR_ANGLE(index,value) (HANDLER_PTR->motor_angle[index]=value)
#define __SET_MOTOR_SPEED(index,value) (HANDLER_PTR->motor_speed[index]=value)
#define __SET_MOTOR_CURRENT(index,value) (HANDLER_PTR->motor_current[index]=value)

#define __GET_JOINT_ANGLE(index) (HANDLER_PTR->feedback_joint_angle[index])
#define __SET_JOINT_ANGLE(index,value) (HANDLER_PTR->joint_angle[index]=value)


void temp_task_init()
{

}

/**
 * @brief 刷新句柄的反馈值
 * @details 遍历电机控制句柄，针对点击类型刷新反馈
 * @code 
 */
void temp_task_get_feedback()
{
  uint16_t index;

  /*motor feedback*/
  for(index=0;index<MOTOR_COUNT;index++)
  {
    switch(__GET_MOTOR_TYPE(index))
    {
      case DJI_MOTOR:
        DJI_Motor_get_feedback(
          (DJI_Motor_Ctrl_t*)__GET_MOTOR_INSTANCE(index),
          &__GET_MOTOR_CURRENT(index),
          &__GET_MOTOR_SPEED(index),
          &__GET_MOTOR_ANGLE(index)
        );
        break;
      /*
      case motor_type:
        __GET_MOTOR_ANGLE(index)=
          get_motor_anlge_func(
            __GET_MOTOR_INSTANCE(index)
          )
        ...
        
        break;
      */
      default:
    }
  }

  /*joint angle map*/
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
void temp_task_mode_flash()
{
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
void temp_task_set_output()
{
  switch(__GET_STRUCT_MODE())
  {
    case first_mode:
      __first_mode_ctrl_func();
      break;
    default:
  }
}

/**
 * @brief 控制输出
 * @details 根据电机种类与控制状态设定输出
 */
void temp_task_output()
{
  uint16_t index;
  /*joint map to motor state*/

  /*motor output*/
  for(index=0;index<MOTOR_COUNT;index++)
  {
    switch(__GET_MOTOR_TYPE(index))
    {
      case first_motor_type:
        if(__GET_MOTOR_CTRL_MODE==POS_LOOP)
        {
          /*
          __GET_MOTOR_ANGLE(index,
            get_motor_anlge_func(
              __GET_MOTOR_INSTANCE(index)
            )
          )
          ...
          */
        }
        break;
      default:
    }
  }
}
