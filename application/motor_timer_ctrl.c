#include "motor_timer_ctrl.h"
#include "FreeRTOS.h"
// 提供电机反馈,pid计算以及输出函数
//#include "hand_task.h"
//#include "gimbal_task.h"
#include "cmsis_os2.h"
#include "timers.h"
#include "DJI_motor_canbus.h"
#include "dm4310_drv.h"
#include "AK_series.h"
#include "servo.h"

#include "detect_task.h"
      
/*can*/
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
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
// 在FreeRTOS中，绝对不能在定时器回调函数中使用阻塞式延时（如HAL_Delay, osDelay, vTaskDelay等），这会导致定时器任务被挂起，影响其他定时器的执行
// 建议使用time = HAL_GetTick();
void motor_timer_ctrl_callback(void)
{
  if(toe_is_error(DBUSTOE))
  {
    /*填写失力状态使能*/
  }

  /*****standalone_motor_ctrl*****/

  //延时控制发送频率
    static uint32_t last_time_j2 = 0;
     static uint32_t last_time_j3 = 0;
     static uint32_t last_time_j5 = 0;
     static uint32_t last_time_gripper = 0;
     static uint32_t last_time_j1 = 0;
     static uint32_t last_time_j4 = 0;

    uint32_t current_time = HAL_GetTick();
//达妙电机为同步通信发送等于接收频率，不能发送太快
    // 达妙电机：每3ms发送一次
    if(current_time - last_time_j1 >= 2) {
        __dm4310_mit_output_ctrl(&hfdcan1, &DM_Motor_J1);
        last_time_j1 = current_time;
    }
    
     if(current_time - last_time_j2 >= 2) {
         __dm4310_mit_output_ctrl(&hfdcan1, &DM_Motor_J2);
         last_time_j2 = current_time;
     }

    if(current_time - last_time_j3 >= 3) {
        __dm4310_mit_output_ctrl(&hfdcan1, &DM_Motor_J3);
        last_time_j3 = current_time;
    }

    if(current_time - last_time_j4 >= 2) {
        __dm4310_mit_output_ctrl(&hfdcan2, &DM_Motor_J4);
        last_time_j4 = current_time;
    }

    if(current_time - last_time_j5 >= 3) {
        __dm4310_mit_output_ctrl(&hfdcan2, &DM_Motor_J5);
        last_time_j5 = current_time;
    }

    if(current_time - last_time_gripper >= 3) {
        __dm4310_mit_output_ctrl(&hfdcan2, &DM_Motor_gripper);
        last_time_gripper = current_time;
    }

    // // AK电机：每1ms发送一次
    // if(current_time - last_time_j1 >= 2) {
    //     __AK_joint_motor_ctrl_hook(&AK70_10_motor, &hfdcan2);
    //     last_time_j1 = current_time;
    // }

    // // DJI电机：每1ms发送一次
    // if(current_time - last_time_j4 >= 1) {
    //     __DJI_CANBus_ctrl_loop(&DJI_CAN2_Bus_ctrl);
    //     last_time_j4 = current_time;
    // }

    // 其他总线：每1ms发送一次
    static uint32_t last_time_others = 0;
    if(current_time - last_time_others >= 1) {
        __DJI_CANBus_ctrl_loop(&DJI_CAN1_Bus_ctrl);
        __DJI_CANBus_ctrl_loop(&DJI_CAN3_Bus_ctrl);
        __SERVO_CTRL_HOOK();
        last_time_others = current_time;
    }

  //  __AK_joint_motor_ctrl_hook(&AK70_10_motor, &hfdcan2);
  //  __dm4310_mit_output_ctrl(&hfdcan2, &DM_Motor_J2);
  // __dm4310_mit_output_ctrl(&hfdcan2, &DM_Motor_J3);
  // __dm4310_mit_output_ctrl(&hfdcan2, &DM_Motor_J5);
  // __dm4310_mit_output_ctrl(&hfdcan2, &DM_Motor_gripper);
  // __DJI_CANBus_ctrl_loop(&DJI_CAN1_Bus_ctrl);
  // __DJI_CANBus_ctrl_loop(&DJI_CAN2_Bus_ctrl);
  // __DJI_CANBus_ctrl_loop(&DJI_CAN3_Bus_ctrl);
  // __SERVO_CTRL_HOOK();

  //__dm4310_mit_output_ctrl(hfdcan?,DM_Motor_J2);

  // 添加检测层此处和检测层交互
  // 检测层检查所有功能都正常才使能
  /*****hand_task*****/
  if (get_feedback_statue(HAND_MOTOR))
  {

    if (get_output_statue(HAND_MOTOR))
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


