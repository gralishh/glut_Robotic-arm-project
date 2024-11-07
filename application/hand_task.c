#include "hand_task.h"
#include "hand_task_interface.h"
#include "motor_timer_ctrl.h"

//Readme:
//由于关节电机编码器属于增量式
//不要用下载器对单片机单独供电
//如果使用
//一定要在无力档烧写代码，重启等

// 电机反馈与输出使能控制
#define FEEDBACK_ON() (motor_ctrl_feedback_cmd(HAND_MOTOR,MOTOR_CMD_ENABLE));
#define FEEDBACK_OFF() (motor_ctrl_feedback_cmd(HAND_MOTOR,MOTOR_CMD_DISABLE));
#define OUTPUT_ON() (motor_ctrl_output_cmd(HAND_MOTOR,MOTOR_CMD_ENABLE));
#define OUTPUT_OFF() (motor_ctrl_output_cmd(HAND_MOTOR,MOTOR_CMD_DISABLE));

Hand_Control_t	hand_control;

// 圈数计量计时器
void vTimerAngleCallback(TimerHandle_t xTimer)
{
	if(hand_control.hand_x_4_motor.hand_motor_measure_all.last_ecd >30000 && hand_control.hand_x_4_motor.hand_encoder_4_measure->ecd < 7000)
	{
		hand_control.hand_x_4_motor.hand_motor_measure_all.motor_circle = 1;
	}
	else if(hand_control.hand_x_4_motor.hand_motor_measure_all.last_ecd < 7000 && hand_control.hand_x_4_motor.hand_encoder_4_measure->ecd > 30000)
	{
		hand_control.hand_x_4_motor.hand_motor_measure_all.motor_circle = 0;	
	}
	hand_control.hand_x_4_motor.hand_motor_measure_all.last_ecd = hand_control.hand_x_4_motor.hand_encoder_4_measure->ecd;
}


/**
 * @brief 机械臂主任务
 * @param pvParameters 
 */
void Hand_task(void const *pvParameters)
{
	vTaskDelay(1000);

	taskENTER_CRITICAL();
	Hand_Init(&hand_control);	
  /*启动伸缩圈数计数器*/
	TimerHandle_t xTimer = xTimerCreate("Timer", pdMS_TO_TICKS(1), pdTRUE, 0, vTimerAngleCallback);
	taskEXIT_CRITICAL();
	
  /*Main_loop*/
	do
	{
		Hand_Set_Mode(&hand_control); 	                   //机械臂遥控器设置模式
		vTaskDelay(10);
	}while( hand_control.hand_behaviour != HAND_ZERO_FORCE);
	vTaskDelay(150);//延迟一段时间之后在进行校准 目的是先让滑台进行校准

  /*启动电机反馈与输出刷新*/
  FEEDBACK_ON();
  OUTPUT_ON();

	while(1)
  {
	  Hand_Set_Mode(&hand_control); 	                   //机械臂遥控器设置模式
		Hand_Set_Contorl(&hand_control);                   //设置机械臂控制
		//Hand_Feedback_Update(&hand_control);               //机械臂数据反馈
		Hand_Set_Position(&hand_control);                  //机械臂遥控器，键鼠设置位置
		HAND_Mode_Change_Control_Transit(&hand_control);   //控制模式切换 控制数据过渡
		//Hand_Control_loop(&hand_control);                  //机械臂控制PID计算
		View_steering_engine_control(&hand_control);
		Get_Hand_Status(hand_control);//只进行值传递

    //控制器掉线判断
    Hand_Dbus_Offline_Control();

    vTaskDelay(1);
  }
}


