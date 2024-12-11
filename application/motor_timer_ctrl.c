#include "motor_timer_ctrl.h"
#include "FreeRTOS.h"
// 提供电机反馈,pid计算以及输出函数
//#include "hand_task.h"
//#include "gimbal_task.h"
#include "cmsis_os2.h"
#include "timers.h"
#include "M8010_motor.h"
#include "DJI_motor_canbus.h"
#include "dm4310_drv.h"

#include "detect_task.h"

/*can*/
extern FDCAN_HandleTypeDef hfdcan2;
/*specific motor handler*/
extern Joint_Motor_t DM_Motor_J2;

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
  if(toe_is_error(DBUSTOE))
  {
    /*填写失力状态使能*/
  }

  /*****standalone_motor_ctrl*****/
  __M8010_motor_control_hook();
  //__dm4310_mit_output_ctrl(&hfdcan2,&DM_Motor_J2);
  __DJI_CANBus_ctrl_loop(&DJI_CAN1_Bus_ctrl);
  __DJI_CANBus_ctrl_loop(&DJI_CAN2_Bus_ctrl);
  __DJI_CANBus_ctrl_loop(&DJI_CAN3_Bus_ctrl);

  //__dm4310_mit_output_ctrl(hfdcan?,DM_Motor_J2);
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
