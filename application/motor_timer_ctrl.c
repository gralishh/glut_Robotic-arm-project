#include "motor_timer_ctrl.h"
#include "FreeRTOS.h"
// 提供电机反馈,pid计算以及输出函数
//#include "hand_task.h"
//#include "gimbal_task.h"
#include "cmsis_os2.h"
#include "timers.h"

#define get_feedback_statue(grp_index) \
  (motor_ctrl_handler.feedback_cmd_list[(motor_group_index_t)grp_index])

#define get_output_statue(grp_index) \
  (motor_ctrl_handler.output_cmd_list[(motor_group_index_t)grp_index])

extern osTimerId_t motor_timer_ctrlHandle;

/*电机控制句柄*/
static motor_ctrl_handler_t motor_ctrl_handler={0};

void motor_ctrl_init(void)
{
  xTimerStart(motor_timer_ctrlHandle,1);
  return;
}

void motor_ctrl_feedback_cmd(motor_group_index_t grp_index,motor_group_statue_t statue)
{
  motor_ctrl_handler.feedback_cmd_list[grp_index]=statue;
}

void motor_ctrl_output_cmd(motor_group_index_t grp_index,motor_group_statue_t statue)
{
  motor_ctrl_handler.output_cmd_list[grp_index]=statue;
}

void motor_timer_ctrl_callback(void)
{
  /*****hand_task*****/
  if(get_feedback_statue(HAND_MOTOR))
  {

    if(get_output_statue(HAND_MOTOR))
    {

    }
  }



  /*****gimbal_task*****/
  if(get_feedback_statue(GIMBAL_MOTOR))
  {

    if(get_output_statue(GIMBAL_MOTOR))
    {

    }
  }
}
