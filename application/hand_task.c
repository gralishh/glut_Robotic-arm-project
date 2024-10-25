//#include "hand_task.h"
#include "gimbal_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "arm_math.h" 
#include "detect_task.h"
#include "user_lib.h"
#include "chassis_task.h"
#include "GO_M8010_control.h"
#include "math.h"
#include "MCU_communicaton_task.h"
#include "cmsis_armcc.h"
#include "bsp_usart.h"
#include "motor_timer_ctrl.h"
//Readme:
//由于关节电机编码器属于增量式
//不要用下载器对单片机单独供电
//如果使用
//一定要在无力档烧写代码，重启等



/**
 * @brief 死区限制
 */
#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
//pid最大输出限幅宏函数0
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

	//PID清理宏函数
#define hand_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                        \
        PID_clear(&(gimbal_clear)->hand_roll_1_motor.hand_motor_measure_all.hand_motor_relative_pid);    			\
        PID_clear(&(gimbal_clear)->hand_roll_1_motor.hand_motor_measure_all.hand_motor_speed_pid);                    \
																								\
		PID_clear(&(gimbal_clear)->hand_pitch_2_motor.hand_motor_measure_all.hand_motor_relative_pid);    			\
        PID_clear(&(gimbal_clear)->hand_pitch_2_motor.hand_motor_measure_all.hand_motor_speed_pid);                    \
																								\
		PID_clear(&(gimbal_clear)->hand_roll_3_motor.hand_motor_measure_all.hand_motor_relative_pid);    			\
        PID_clear(&(gimbal_clear)->hand_roll_3_motor.hand_motor_measure_all.hand_motor_speed_pid);                    \
																							\
		PID_clear(&(gimbal_clear)->hand_x_4_motor.hand_motor_measure_all.hand_motor_relative_pid);    			\
        PID_clear(&(gimbal_clear)->hand_x_4_motor.hand_motor_measure_all.hand_motor_speed_pid);                    \
																							\
		PID_clear(&(gimbal_clear)->hand_yaw_5_motor.hand_motor_measure_all.hand_motor_relative_pid);    			\
        PID_clear(&(gimbal_clear)->hand_yaw_5_motor.hand_motor_measure_all.hand_motor_speed_pid);                    \
    }

// 电机反馈与输出使能控制
#define FEEDBACK_ON() (motor_ctrl_feedback_cmd(HAND_MOTOR,MOTOR_CMD_ENABLE));
#define FEEDBACK_OFF() (motor_ctrl_feedback_cmd(HAND_MOTOR,MOTOR_CMD_DISABLE));
#define OUTPUT_ON() (motor_ctrl_output_cmd(HAND_MOTOR,MOTOR_CMD_ENABLE));
#define OUTPUT_OFF() (motor_ctrl_output_cmd(HAND_MOTOR,MOTOR_CMD_DISABLE));


extern float joint_angles[JOINTS][POSITIONS_PER_JOINT][ANGLES_PER_POSITION];
Hand_Control_t	hand_control;
hand_status_t hand_status;


const hand_status_t *get_hand_status_point(void)
{
  return &hand_status;
}

//初始化电机参数
int16_t	hand_set_current1_4[4]		     = {0,0,0,0};
int16_t	hand_set_current1_4text[4]		 = {500,3000,3000,3000};
int16_t	hand_set_current5_8[4]		     = {0,0,0,0};

extern FDCAN_HandleTypeDef hfdcan2;
extern UART_HandleTypeDef huart1;



// 软件定时器回调函数
void vTimerAngleCallback(TimerHandle_t xTimer)
{
//    hand_control.hand_roll_1_motor.hand_motor_measure_all.last_encoder_angle = hand_control.hand_roll_1_motor.hand_motor_measure_all.motor_encoder_angle;
//	hand_control.hand_pitch_2_motor.hand_motor_measure_all.last_encoder_angle = hand_control.hand_pitch_2_motor.hand_motor_measure_all.motor_encoder_angle;

//	hand_motor_circle_measure(&hand_control.hand_roll_1_motor.hand_motor_measure_all);
//	hand_motor_circle_measure(&hand_control.hand_pitch_2_motor.hand_motor_measure_all);
//	hand_motor_circle_measure(&hand_control.hand_x_4_motor.hand_motor_measure_all);
	
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
	#if gimbalControlBoard
	//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1670);	//控制占空比
	//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1775);	//控制占空比
	vTaskDelay(1000);
	taskENTER_CRITICAL();
	Hand_Init(&hand_control);	
	TimerHandle_t xTimer = xTimerCreate("Timer", pdMS_TO_TICKS(1), pdTRUE, 0, vTimerAngleCallback);
//	if (xTimer != NULL)
//    {
//        // 启动定时器
//        if (xTimerStart(xTimer, 0) == pdPASS)
//        {
//            // 定时器成功启动
//        }
//    }
	taskEXIT_CRITICAL();
	
	
	do
	{
		Hand_Set_Mode(&hand_control); 	                   //机械臂遥控器设置模式
		vTaskDelay(10);
	}while( hand_control.hand_behaviour != HAND_ZERO_FORCE);
	vTaskDelay(150);//延迟一段时间之后在进行校准 目的是先让滑台进行校准

	////进行多次校准，一次有误差
	//hand_position_Init(&hand_control);                 //机械臂位置初始化
	//hand_position_Init(&hand_control);                 //机械臂位置初始化
	//hand_position_Init(&hand_control);                 //机械臂位置初始化
	//hand_position_Init(&hand_control);                 //机械臂位置初始化
	//hand_position_Init(&hand_control);                 //机械臂位置初始化
	////hand_roll_3_position_Init(&hand_control);
	//vTaskDelay(10);
	//hand_yaw_5_position_Init(&hand_control);			//先逼近位置
	//hand_control.hand_roll_3_motor.hand_motor_measure_all.relative_init_angle = 0.0f;
	//hand_control.hand_yaw_5_motor.hand_motor_measure_all.relative_init_angle = 0.0f;
	//vTaskDelay(40);
	////hand_roll_3_position_Init(&hand_control);
	//vTaskDelay(20);
	//hand_yaw_5_position_Init(&hand_control);	

		
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
// 		Hand_Temperature_control(&hand_control);
		  View_steering_engine_control(&hand_control);
		  Get_Hand_Status(hand_control);//只进行值传递

	
      if (toe_is_error(DBUSTOE))
      {
        OUTPUT_OFF();
        for(char i = 0;i<4;i++)
        {
          hand_set_current1_4[i]  = 0;
				} 
			  hand_control.hand_yaw_5_motor.GM_Send_Data.mode=0;
			  hand_control.hand_roll_3_motor.GM_Send_Data.mode=0;
        Hand_Current_Output();
      }
      else
      {
        //Hand_Set_Reverse();
        OUTPUT_ON();
			}

      //Hand_Current_Output();
			
      //SERVO1_RS485_Send(&hand_control.hand_yaw_5_motor.GM_Send_Data , hand_control.hand_yaw_5_motor.rData);
      //SERVO2_RS485_Send(&hand_control.hand_roll_3_motor.GM_Send_Data , hand_control.hand_roll_3_motor.rData);
      //CanSendMess(&hfdcan2,SEND_ID201_204,hand_set_current1_4); 
			
      vTaskDelay(1);
	}
	#endif
}
/// @brief 机械臂初始化 主要是pid初始化
/// @param hand_init 
static void Hand_Init(Hand_Control_t *hand_init)
{
  /*****PID赋值与初始化*****/
	static const fp32 roll_1_speed_pid[3]  			    = {roll_1_SPEED_PID_KP				,roll_1_SPEED_PID_KI				,roll_1_SPEED_PID_KD};
	static const fp32 roll_1_relative_pid[3] 			= {roll_1_relative_angle_PID_KP		,roll_1_relative_angle_PID_KI		,roll_1_relative_angle_PID_KD};	

	static const fp32 pitch_2_speed_pid[3]  			= {pitch_2_SPEED_PID_KP				,pitch_2_SPEED_PID_KI				,pitch_2_SPEED_PID_KD};
	static const fp32 pitch_2_relative_pid[3] 			= {pitch_2_relative_angle_PID_KP	,pitch_2_relative_angle_PID_KI		,pitch_2_relative_angle_PID_KD};

	static const fp32 roll_3_speed_pid[3]  			    = {roll_3_SPEED_PID_KP				,roll_3_SPEED_PID_KI				,roll_3_SPEED_PID_KD};
	static const fp32 roll_3_relative_pid[3] 			= {roll_3_relative_angle_PID_KP  	,roll_3_relative_angle_PID_KI		,roll_3_relative_angle_PID_KD};	
		
	static const fp32 x_4_speed_pid[3]  	           = {x_4_SPEED_PID_KP				    ,x_4_SPEED_PID_KI					,x_4_SPEED_PID_KD};
	static const fp32 x_4_relative_pid[3]              = {x_4_relative_angle_PID_KP		,x_4_relative_angle_PID_KI		        ,x_4_relative_angle_PID_KD};							
 
	static const fp32 yaw_5_speed_pid[3]  			    = {yaw_5_SPEED_PID_KP				,yaw_5_SPEED_PID_KI				    ,yaw_5_SPEED_PID_KD};
	static const fp32 yaw_5_relative_pid[3] 			= {yaw_5_relative_angle_PID_KP  	,yaw_5_relative_angle_PID_KI		,yaw_5_relative_angle_PID_KD};	

	PID_Init(&hand_init->hand_roll_1_motor.hand_motor_measure_all.hand_motor_relative_pid		,PID_POSITION		,roll_1_relative_pid	,roll_1_relative_angle_PID_MAX_OUT		,roll_1_relative_angle_PID_MAX_IOUT,0,0);
  PID_Init(&hand_init->hand_roll_1_motor.hand_motor_measure_all.hand_motor_speed_pid		,PID_POSITION		,roll_1_speed_pid		,roll_1_SPEED_PID_MAX_OUT				,roll_1_SPEED_PID_MAX_IOUT,0,0);
  
	PID_Init(&hand_init->hand_pitch_2_motor.hand_motor_measure_all.hand_motor_relative_pid	,PID_POSITION		,pitch_2_relative_pid	,pitch_2_relative_angle_PID_MAX_OUT	    ,pitch_2_relative_angle_PID_MAX_IOUT,0,0);
  PID_Init(&hand_init->hand_pitch_2_motor.hand_motor_measure_all.hand_motor_speed_pid		,PID_POSITION		,pitch_2_speed_pid		,pitch_2_SPEED_PID_MAX_OUT				,pitch_2_SPEED_PID_MAX_IOUT,0,0);

	PID_Init(&hand_init->hand_roll_3_motor.hand_motor_measure_all.hand_motor_relative_pid	    ,PID_POSITION		,roll_3_relative_pid	,roll_3_relative_angle_PID_MAX_OUT	    ,roll_3_relative_angle_PID_MAX_IOUT,0,0);
  PID_Init(&hand_init->hand_roll_3_motor.hand_motor_measure_all.hand_motor_speed_pid		,PID_POSITION		,roll_3_speed_pid		,roll_3_SPEED_PID_MAX_OUT				,roll_3_SPEED_PID_MAX_IOUT,0,0);

	PID_Init(&hand_init->hand_x_4_motor.hand_motor_measure_all.hand_motor_relative_pid		,PID_POSITION		,x_4_relative_pid		,x_4_relative_angle_PID_MAX_OUT		    ,x_4_relative_angle_PID_MAX_IOUT,0,0);
  PID_Init(&hand_init->hand_x_4_motor.hand_motor_measure_all.hand_motor_speed_pid			,PID_POSITION		,x_4_speed_pid			,x_4_SPEED_PID_MAX_OUT					,x_4_SPEED_PID_MAX_IOUT,0,0);

	PID_Init(&hand_init->hand_yaw_5_motor.hand_motor_measure_all.hand_motor_relative_pid		,PID_POSITION		,yaw_5_relative_pid		,yaw_5_relative_angle_PID_MAX_OUT	    ,yaw_5_relative_angle_PID_MAX_IOUT,0,0);
  PID_Init(&hand_init->hand_yaw_5_motor.hand_motor_measure_all.hand_motor_speed_pid			,PID_POSITION		,yaw_5_speed_pid		,yaw_5_SPEED_PID_MAX_OUT			    ,yaw_5_SPEED_PID_MAX_IOUT,0,0);

	hand_total_pid_clear(hand_init);
	

  /*****滤波器初始化*****/
	const static fp32 hand_1_order_filter[1] = {HAND_ACCEL_1_NUM};
  const static fp32 hand_2_order_filter[1] = {HAND_ACCEL_2_NUM};
	

  /*****控制|视觉|电机指针赋值*****/
	hand_init->gimbal_rc_ctrl = get_remote_control_point();
	hand_init->hand_visual_control = (visuals_rx_data_t *)get_visuals_rx_data_point();
  hand_init->hand_custom_control = (elf_measure_t *)get_self_measure_point(); //要使用视觉记得更换指针
	
	hand_init->hand_roll_1_motor.hand_motor_measure_all.hand_motor_measure  				 =  get__roll_1_Measure_Point();
  
  /*空心编码器获取指针，被弃用*/
  //hand_init->hand_roll_1_motor.hollowEncoderMotor                                =  (EncoderHollowMotor_t *)get_hand_encoder_hand_roll_1_CapVal_Point(); 
	
	hand_init->hand_pitch_2_motor.hand_motor_measure_all.hand_motor_measure   			 =  get_2006_HAND_pitch_2_Measure_Point();
  /*同上*/
  //hand_init->hand_pitch_2_motor.hollowEncoderMotor                               =  (EncoderHollowMotor_t *)get_hand_encoder_hand_pitch_2_CapVal_Point();	

	hand_init->hand_roll_3_motor.rData													                     =  (MOTOR_recv *)get_hand_roll_3_motor_rx_point();

	hand_init->hand_x_4_motor.hand_motor_measure_all.hand_motor_measure              =  get_2006_HAND_x_4_Measure_Point();
	hand_init->hand_x_4_motor.hand_encoder_4_measure                                 =  get_encoder_HAND_x_4_Measure_Point();

  hand_init->hand_yaw_5_motor.rData                                                = (MOTOR_recv *)get_hand_yaw_5_motor_rx_point();

  /*初值设置*/
	//Hand_Feedback_Update(hand_init);
  FEEDBACK_ON();
	
	hand_init->hand_yaw_5_motor.hand_motor_measure_all.relative_angle_set	                
    =hand_init->hand_yaw_5_motor.hand_motor_measure_all.lock_angle	            
    =hand_init->hand_yaw_5_motor.hand_motor_measure_all.lsat_relative_angle              
    =hand_init->hand_yaw_5_motor.hand_motor_measure_all.relative_angle;

	hand_init->hand_x_4_motor.hand_motor_measure_all.relative_angle_set	                
    =hand_init->hand_x_4_motor.hand_motor_measure_all.lock_angle	                
    =hand_init->hand_x_4_motor.hand_motor_measure_all.lsat_relative_angle                
    =hand_init->hand_x_4_motor.hand_motor_measure_all.relative_angle;

	hand_init->hand_roll_3_motor.hand_motor_measure_all.relative_angle_set				
    =hand_init->hand_roll_3_motor.hand_motor_measure_all.lock_angle				
    =hand_init->hand_roll_3_motor.hand_motor_measure_all.lsat_relative_angle             
    =hand_init->hand_roll_3_motor.hand_motor_measure_all.relative_angle;

	hand_init->hand_pitch_2_motor.hand_motor_measure_all.relative_angle_set				
    =hand_init->hand_pitch_2_motor.hand_motor_measure_all.lock_angle				
    =hand_init->hand_pitch_2_motor.hand_motor_measure_all.lsat_relative_angle            
    =hand_init->hand_pitch_2_motor.hand_motor_measure_all.relative_angle;

	hand_init->hand_roll_1_motor.hand_motor_measure_all.relative_angle_set				
    =hand_init->hand_roll_1_motor.hand_motor_measure_all.lock_angle				
    =hand_init->hand_roll_1_motor.hand_motor_measure_all.lsat_relative_angle             
    =hand_init->hand_roll_1_motor.hand_motor_measure_all.relative_angle;
	

	hand_init->hand_x_4_motor.hand_motor_measure_all.lsat_relative_angle = 0.0f;
	hand_init->hand_x_4_motor.hand_motor_measure_all.motor_encoder_angle = 0.0f;
	hand_init->hand_x_4_motor.hand_motor_measure_all.original_RM_angle = 0.0f;
	hand_init->hand_x_4_motor.hand_motor_measure_all.motor_circle = 0;

	hand_init->hand_pitch_2_motor.hand_motor_measure_all.lsat_relative_angle = 0.0f;
	hand_init->hand_pitch_2_motor.hand_motor_measure_all.motor_encoder_angle = 0.0f;
	hand_init->hand_pitch_2_motor.hand_motor_measure_all.original_RM_angle = 0.0f;
	hand_init->hand_pitch_2_motor.hand_motor_measure_all.motor_circle = 0;

	hand_init->hand_roll_1_motor.hand_motor_measure_all.lsat_relative_angle = 0.0f;
	hand_init->hand_roll_1_motor.hand_motor_measure_all.motor_encoder_angle = 0.0f;
	hand_init->hand_roll_1_motor.hand_motor_measure_all.original_RM_angle = 0.0f;
	hand_init->hand_roll_1_motor.hand_motor_measure_all.motor_circle = 0;

	hand_init->hand_yaw_5_motor.hand_motor_measure_all.lsat_relative_angle = 0.0f;
	hand_init->hand_yaw_5_motor.hand_motor_measure_all.motor_encoder_angle = 0.0f;
	hand_init->hand_yaw_5_motor.hand_motor_measure_all.original_RM_angle = 0.0f;
	hand_init->hand_yaw_5_motor.hand_motor_measure_all.motor_circle = 0;

	hand_init->hand_roll_3_motor.hand_motor_measure_all.lsat_relative_angle = 0.0f;
	hand_init->hand_roll_3_motor.hand_motor_measure_all.motor_encoder_angle = 0.0f;
	hand_init->hand_roll_3_motor.hand_motor_measure_all.original_RM_angle = 0.0f;
	hand_init->hand_roll_3_motor.hand_motor_measure_all.motor_circle = 0;


	hand_init->hand_roll_3_motor.GM_Send_Data.mode=0;  
	hand_init->hand_roll_3_motor.GM_Send_Data.id=GM_roll_3_ID;  
	hand_init->hand_yaw_5_motor.GM_Send_Data.mode=0;  
	hand_init->hand_yaw_5_motor.GM_Send_Data.id=GM_Yaw_5_ID; 

	hand_init->hand_position          =hand_position_N;
	hand_init->hand_behaviour         =HAND_INIT;
	hand_init->hand_behaviour_last    =HAND_INIT;

	hand_init->hand_roll_1_motor.hand_motor_measure_all.max_relative_angle                =roll_1_relative_angle_set_max;
	hand_init->hand_roll_1_motor.hand_motor_measure_all.min_relative_angle                =roll_1_relative_angle_set_min;
	
	hand_init->hand_x_4_motor.hand_motor_measure_all.max_relative_angle		                =x_4_relative_angle_set_max;
	hand_init->hand_x_4_motor.hand_motor_measure_all.min_relative_angle		                =x_4_relative_angle_set_min;
	
	hand_init->hand_roll_3_motor.hand_motor_measure_all.max_relative_angle                =roll_3_relative_angle_set_max;
	hand_init->hand_roll_3_motor.hand_motor_measure_all.min_relative_angle				        =roll_3_relative_angle_set_min;
	
	hand_init->hand_pitch_2_motor.hand_motor_measure_all.max_relative_angle				      	=pitch_2_relative_angle_set_max;
	hand_init->hand_pitch_2_motor.hand_motor_measure_all.min_relative_angle			       		=pitch_2_relative_angle_set_min;

	hand_init->hand_yaw_5_motor.hand_motor_measure_all.max_relative_angle		              =yaw_5_relative_angle_set_max;
	hand_init->hand_yaw_5_motor.hand_motor_measure_all.min_relative_angle	               	=yaw_5_relative_angle_set_min;
	

	first_order_filter_init(&hand_init->hand_cmd_slow_set_1, 0.1, hand_1_order_filter);
  first_order_filter_init(&hand_init->hand_cmd_slow_set_2, 0.1, hand_2_order_filter);	
	
}

/**
 * @brief 机械臂初始化位置 
 * 先校准机械臂除大臂的关节
 * 同时移动滑台到最中心完成校准
 * @param gimbal_init
 */
static void hand_position_Init(Hand_Control_t *gimbal_init)
{
	uint32_t wait_time = 0;
	
	uint8_t hand_roll_1_init = 0;
	uint8_t hand_pitch_2_init = 0;
	uint8_t hand_x_4_init = 0;
	
	float hand_roll_1_last_ecd = 0.0f;
	float hand_pitch_2_last_ecd = 0.0f;
	float hand_x_4_last_ecd = 0.0f;

	int16_t hand_current_init1[4]={4000,-4000,-4000,2000};
	int16_t hand_current_init0[4]={0,0,0,0};

  FEEDBACK_ON();
  /*****等待停止发送电流*****/
	do
	{
		//Hand_Feedback_Update(&hand_control);
		CanSendMess(&hfdcan2,SEND_ID201_204,hand_current_init1);	
		vTaskDelay(1);
		if(gimbal_init->hand_roll_1_motor.hand_motor_measure_all.hand_motor_measure->given_current != 0     ||\
				gimbal_init->hand_pitch_2_motor.hand_motor_measure_all.hand_motor_measure->given_current != 0  ||\
					gimbal_init->hand_x_4_motor.hand_motor_measure_all.hand_motor_measure->given_current != 0)
		{
			wait_time++;
		}
		if(wait_time > 100)
		{
			break;
		}
	}

  while(gimbal_init->hand_roll_1_motor.hand_motor_measure_all.hand_motor_measure->given_current == 0
        || gimbal_init->hand_pitch_2_motor.hand_motor_measure_all.hand_motor_measure->given_current == 0
				|| gimbal_init->hand_x_4_motor.hand_motor_measure_all.hand_motor_measure->given_current == 0				
	 				);
		vTaskDelay(50);
  FEEDBACK_ON();

  /*等待编码值稳定(机械结构卡在限位处)*/
	for(;;)
	{			
		//这里不用can的last_ecd是因为要通过限制频率来使last_ecd相对变大
		if (fabs(hand_roll_1_last_ecd - gimbal_init->hand_roll_1_motor.hand_motor_measure_all.hand_motor_measure->ecd)<5)
		{
			CanSendMess(&hfdcan2,CAN_2006_roll_1_ID,hand_current_init0);
			gimbal_init->hand_roll_1_motor.hand_motor_measure_all.relative_init_angle += - gimbal_init->hand_roll_1_motor.hand_motor_measure_all.relative_angle;
			hand_roll_1_init = 1;
		}
		if (fabs(hand_pitch_2_last_ecd - gimbal_init->hand_pitch_2_motor.hand_motor_measure_all.hand_motor_measure->ecd)<5)
		{
			CanSendMess(&hfdcan2,CAN_2006_HAND_pitch_2_ID,hand_current_init0);
			gimbal_init->hand_pitch_2_motor.hand_motor_measure_all.relative_init_angle += gimbal_init->hand_pitch_2_motor.hand_motor_measure_all.relative_angle;
			hand_pitch_2_init = 1;
		}	
		if (fabs(hand_x_4_last_ecd - gimbal_init->hand_x_4_motor.hand_motor_measure_all.hand_motor_measure->ecd)<5)
		{
			CanSendMess(&hfdcan2,CAN_2006_HAND_x_4_ID,hand_current_init0);
			hand_x_4_init = 1;
		}

		

		
		hand_roll_1_last_ecd   = gimbal_init->hand_roll_1_motor.hand_motor_measure_all.hand_motor_measure->ecd;
		hand_pitch_2_last_ecd  = gimbal_init->hand_pitch_2_motor.hand_motor_measure_all.hand_motor_measure->ecd;
		hand_x_4_last_ecd      = gimbal_init->hand_x_4_motor.hand_motor_measure_all.hand_motor_measure->ecd;
	
		if(hand_roll_1_init && hand_pitch_2_init && hand_x_4_init)
		{
			break;
		}
		vTaskDelay(10);
	}
}		
static void hand_roll_3_position_Init(Hand_Control_t *gimbal_init)
{		
	uint8_t hand_roll_3_init = 0;
	float hand_roll_3_last_pos = 0.0f;
	uint32_t wait_time = 0;

	MOTOR_send hand_GoM8010_init1;
	hand_GoM8010_init1.mode = 1;
	MOTOR_send hand_GoM8010_init0;
	hand_GoM8010_init0.mode = 0;
	hand_GoM8010_init0.GM_Send_Effort = 0;

  FEEDBACK_ON();
	do
	{
		hand_GoM8010_init1.id = 1;
		hand_GoM8010_init1.GM_Send_Effort = 0.6;//3.0
		modify_data(&hand_GoM8010_init1);
	    USART1_Send((uint8_t *)&hand_GoM8010_init1,sizeof(hand_GoM8010_init1.motor_send_data));
		vTaskDelay(15);
		wait_time++;
		if(wait_time > 100)
		{
			break;
		}
	}
	while(gimbal_init->hand_roll_3_motor.rData->Temp == 0);
		vTaskDelay(50);

	while(1)//待前端以及滑台完成校准 不固定找不到校准点
	{
		if (fabs(hand_roll_3_last_pos - gimbal_init->hand_roll_3_motor.rData->Pos < 0.1f))
		{
			hand_GoM8010_init0.id = 1;
			vTaskDelay(2);
			Hand_Feedback_Update();
			gimbal_init->hand_roll_3_motor.hand_motor_measure_all.relative_init_angle += gimbal_init->hand_roll_3_motor.rData->Pos;
			modify_data(&hand_GoM8010_init0);
			USART1_Send((uint8_t *)&hand_GoM8010_init0,sizeof(hand_GoM8010_init0.motor_send_data));				
			hand_roll_3_init = 1;
		}
		vTaskDelay(40);
		hand_roll_3_last_pos   = gimbal_init->hand_roll_3_motor.rData->Pos;

		if(hand_roll_3_init)
		{
			break;
		}
		vTaskDelay(10);
	}

}

/**
 * @brief 大臂yaw初始化
 * 超级甩臂
 */
static void hand_yaw_5_position_Init(Hand_Control_t *gimbal_init)
{
	uint8_t hand_yaw_5_init = 0;
	float hand_yaw_5_last_pos = 0.0f;
	uint32_t wait_time = 0;


	MOTOR_send hand_GoM8010_init1;
	hand_GoM8010_init1.mode = 1;
	MOTOR_send hand_GoM8010_init0;
	hand_GoM8010_init0.mode = 0;
	hand_GoM8010_init0.GM_Send_Effort = 0;
		
	do
		{
			Hand_Feedback_Update();
			hand_GoM8010_init1.id = 3;
			hand_GoM8010_init1.GM_Send_Effort = -1.5;//3.0
			modify_data(&hand_GoM8010_init1);
	        //USART6_Send((uint8_t *)&hand_GoM8010_init1,sizeof(hand_GoM8010_init1.motor_send_data));
			SERVO2_RS485_Send((uint8_t *)&hand_GoM8010_init1,sizeof(hand_GoM8010_init1.motor_send_data));

			vTaskDelay(15);
			wait_time++;
			if(wait_time > 100)
			{
				break;
			}
			
		}
		while(gimbal_init->hand_yaw_5_motor.rData->motor_id == 0);
				vTaskDelay(50);

		while(1)//待前端以及滑台完成校准 不固定找不到校准点
		{
			if (fabs(hand_yaw_5_last_pos - gimbal_init->hand_yaw_5_motor.rData->Pos < 0.1f))
				{
					hand_GoM8010_init0.id = 3;					
					vTaskDelay(2);
					Hand_Feedback_Update();
					gimbal_init->hand_yaw_5_motor.hand_motor_measure_all.relative_init_angle += gimbal_init->hand_yaw_5_motor.rData->Pos;
					modify_data(&hand_GoM8010_init0);
          SERVO2_RS485_Send((uint8_t *)&hand_GoM8010_init0,sizeof(hand_GoM8010_init0.motor_send_data));
					
					hand_yaw_5_init = 1;
				}
			vTaskDelay(40);
			Hand_Feedback_Update();
			hand_yaw_5_last_pos = gimbal_init->hand_yaw_5_motor.rData->Pos;

			if (hand_yaw_5_init )
			{
				break;
			}
			vTaskDelay(10);
		}
}

/**
 * @brief 云台遥控器设置模式与控制
 * @param hand_motor_get 
 */
static void Hand_Set_Mode(Hand_Control_t *hand_motor_get)
{
	
	if(hand_motor_get->hand_behaviour==HAND_CUSTOM)
	{
		return;
	}
	if(hand_motor_get->hand_behaviour==HAND_INIT)
	{
		
	}
	if(switch_is_mid(hand_motor_get->gimbal_rc_ctrl->rc.s[1])&&switch_is_mid(hand_motor_get->gimbal_rc_ctrl->rc.s[0]))
	{
	  hand_motor_get->hand_behaviour		= HAND_all_Operation;
	}
	else if(switch_is_down(hand_motor_get->gimbal_rc_ctrl->rc.s[1])&&switch_is_down(hand_motor_get->gimbal_rc_ctrl->rc.s[0]))
	{
	  hand_motor_get->hand_behaviour		= HAND_ZERO_FORCE;
	}
	else hand_motor_get->hand_behaviour	= HAND_MOTIONLESS;
	
}

/**
 * @brief 机械臂遥控器控制
 * @param hand_set_control
 */
static void Hand_Set_Contorl( Hand_Control_t *hand_set_control)
{

  /*控制器增量*/
  static fp32 rc_add_channel_0, rc_add_channel_1, rc_add_channel_2, rc_add_channel_3, rc_add_channel_4;

	static fp32 KEY_add_1, KEY_add_2,	KEY_add_3, KEY_add_4, KEY_add_5;
  static int16_t 	channel_0 = 0, 		channel_1 = 0, 			channel_2 = 0, 		channel_3 = 0, 		channel_4 = 0;

  //将遥控器的数据处理死区 int16_t yaw_channel,pitch_channel
  rc_deadline_limit(hand_set_control->gimbal_rc_ctrl->rc.ch[Channel_0], channel_0, RC_deadband);
  rc_deadline_limit(hand_set_control->gimbal_rc_ctrl->rc.ch[Channel_1], channel_1, RC_deadband);
  rc_deadline_limit(hand_set_control->gimbal_rc_ctrl->rc.ch[Channel_2], channel_2, RC_deadband);
  rc_deadline_limit(hand_set_control->gimbal_rc_ctrl->rc.ch[Channel_3], channel_3, RC_deadband);
  rc_deadline_limit(hand_set_control->gimbal_rc_ctrl->rc.ch[Channel_4], channel_4, RC_deadband);

	if(hand_set_control->hand_behaviour==HAND_all_Operation)
	{
//		rc_add_channel_0 = 	 channel_0 * hand_5_channel_RC_SEN 	;
//		rc_add_channel_1 =   channel_1 * hand_2_channel_RC_SEN 	;
//		
//		rc_add_channel_2 = 	 channel_2 * hand_4_channel_RC_SEN 	;
//		rc_add_channel_3 =   channel_3 * hand_3_channel_RC_SEN 	;
//		
//		rc_add_channel_4 = 	 channel_4 * hand_1_channel_RC_SEN 	;
		
		rc_add_channel_0 = 	 channel_0 * hand_1_channel_RC_SEN 	;
		rc_add_channel_1 =   channel_1 * hand_2_channel_RC_SEN 	;
		rc_add_channel_2 = 	 channel_2 * hand_3_channel_RC_SEN 	;
		rc_add_channel_3 =   channel_3 * hand_4_channel_RC_SEN 	;
		rc_add_channel_4 = 	 channel_4 * hand_5_channel_RC_SEN 	;
			
		//HAND_relative_angle_limit(&hand_set_control->hand_yaw_5_motor.hand_motor_measure_all	,/**/rc_add_channel_0	,yaw_5_relative_angle_set_max	,yaw_5_relative_angle_set_min);
		HAND_relative_angle_limit(&hand_set_control->hand_yaw_5_motor.hand_motor_measure_all	,/**/rc_add_channel_0	,0.0	,0.0);
		HAND_relative_angle_limit(&hand_set_control->hand_x_4_motor.hand_motor_measure_all	  ,/**/rc_add_channel_1	,0.0	,0.0);
		HAND_relative_angle_limit(&hand_set_control->hand_roll_3_motor.hand_motor_measure_all	,/**/rc_add_channel_2	,0.0	,0.0);
		
		hand_set_control->pitch_2_angle = fangle_limit(hand_set_control->roll_1_angle , rc_add_channel_3 , 0.0 , 0.0); //只用调试
		
		//根据电机角度值计算出的高度值进行限位，旋转后不精确
			if((hand_set_control->pitch_2_angle_fact > -95.0f && hand_set_control->pitch_2_angle_fact < -2.0f) ||
					(hand_set_control->pitch_2_angle_fact < -2.0f && rc_add_channel_3 > 0)||
						(hand_set_control->pitch_2_angle_fact > -95.0f && rc_add_channel_3 < 0))
			{
				HAND_relative_angle_limit(&hand_set_control->hand_pitch_2_motor.hand_motor_measure_all	,/**/rc_add_channel_3	,0.0	,0.0);
				HAND_relative_angle_limit(&hand_set_control->hand_roll_1_motor.hand_motor_measure_all	,/**/rc_add_channel_3	,0.0	,0.0);
			}
			
			HAND_relative_angle_limit(&hand_set_control->hand_pitch_2_motor.hand_motor_measure_all	,/**/rc_add_channel_4	,0.0	,0.0);
			HAND_relative_angle_limit(&hand_set_control->hand_roll_1_motor.hand_motor_measure_all	,/**/-1.0*rc_add_channel_4	,0.0	,0.0);
			
	}	
}

/**
 * @brief 电机角度限制
 * @param hand_Motor_t 
 * @param add 角度增量
 * @param max_limit 
 * @param min_limit 
 * @note max_limit与min_limit为0时为无限制
 */
static void HAND_relative_angle_limit(Hand_Motor_t *hand_Motor_t,fp32 add,fp32 max_limit,fp32 min_limit)
{
	if (hand_Motor_t == NULL)
  {
    return;
  }
	
  hand_Motor_t->relative_angle_set += add;
	if(max_limit==min_limit&&min_limit==0.0f)
	{
		return ;
	}
	if(hand_Motor_t->relative_angle_set - add <= max_limit && hand_Motor_t->relative_angle_set - add >= min_limit )
	{
    //是否超过最大 最小
		if (hand_Motor_t->relative_angle_set > max_limit)
		{
			hand_Motor_t->relative_angle_set = max_limit;
		}
		else if (hand_Motor_t->relative_angle_set < min_limit)
		{
			hand_Motor_t->relative_angle_set = min_limit;
		}
	}
	else if(hand_Motor_t->relative_angle_set - add >= max_limit)
	{
		if(add >0)  hand_Motor_t->relative_angle_set -= add;
	}
	else if(hand_Motor_t->relative_angle_set - add <= min_limit)
	{
		if(add <0)  hand_Motor_t->relative_angle_set -= add;
	}
}

/**
 * @brief 浮点数角度限制
 */
static fp32 fangle_limit(fp32 angle_mount,fp32 add,fp32 max_limit,fp32 min_limit)
{
  angle_mount += add;
	if(max_limit==min_limit&&min_limit==0.0f)
	{
		return angle_mount;
	}
	if(angle_mount - add <= max_limit && angle_mount - add >= min_limit )
	{
    //是否超过最大 最小
		if (angle_mount > max_limit)
		{
			angle_mount = max_limit;
		}
		else if (angle_mount < min_limit)
		{
			angle_mount = min_limit;
		}
	}
	else if(angle_mount - add >= max_limit)
	{
		if(add >0)  angle_mount -= add;
	}
	else if(angle_mount - add <= min_limit)
	{
		if(add <0)  angle_mount -= add;
	}
		return angle_mount;
}

/**
 * @brief 控制模式切换 控制数据过渡
 * @param hand_mode_change 
 */
static void HAND_Mode_Change_Control_Transit(Hand_Control_t *hand_mode_change)
{

  if (hand_mode_change->hand_behaviour_last != HAND_ZERO_FORCE && hand_mode_change-> hand_behaviour == HAND_ZERO_FORCE)
  {	
		hand_mode_change->hand_yaw_5_motor.hand_motor_measure_all.current_set 	            = hand_mode_change->hand_yaw_5_motor.hand_motor_measure_all.given_current;	
		hand_mode_change->hand_x_4_motor.hand_motor_measure_all.current_set	                = hand_mode_change->hand_x_4_motor.hand_motor_measure_all.given_current;
		hand_mode_change->hand_roll_3_motor.hand_motor_measure_all.current_set 				= hand_mode_change->hand_roll_3_motor.hand_motor_measure_all.given_current;
		hand_mode_change->hand_pitch_2_motor.hand_motor_measure_all.current_set 		    = hand_mode_change->hand_pitch_2_motor.hand_motor_measure_all.given_current;
		hand_mode_change->hand_roll_1_motor.hand_motor_measure_all.current_set 				= hand_mode_change->hand_roll_1_motor.hand_motor_measure_all.given_current;
    
		hand_mode_change->hand_behaviour_last = hand_mode_change->hand_behaviour;
	}
	else if (hand_mode_change->hand_behaviour_last != HAND_all_Operation && hand_mode_change-> hand_behaviour == HAND_all_Operation) 
	{
		hand_mode_change->hand_yaw_5_motor.hand_motor_measure_all.relative_angle_set             	= hand_mode_change->hand_yaw_5_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_x_4_motor.hand_motor_measure_all.relative_angle_set                  = hand_mode_change->hand_x_4_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_roll_3_motor.hand_motor_measure_all.relative_angle_set 				= hand_mode_change->hand_roll_3_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_pitch_2_motor.hand_motor_measure_all.relative_angle_set 				= hand_mode_change->hand_pitch_2_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_roll_1_motor.hand_motor_measure_all.relative_angle_set 				= hand_mode_change->hand_roll_1_motor.hand_motor_measure_all.relative_angle; 				


    hand_mode_change->hand_yaw_5_motor.hand_motor_measure_all.lock_angle 	                    = hand_mode_change->hand_yaw_5_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_x_4_motor.hand_motor_measure_all.lock_angle                          = hand_mode_change->hand_x_4_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_roll_3_motor.hand_motor_measure_all.lock_angle 			         	= hand_mode_change->hand_roll_3_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_pitch_2_motor.hand_motor_measure_all.lock_angle 			        	= hand_mode_change->hand_pitch_2_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_roll_1_motor.hand_motor_measure_all.lock_angle 					    = hand_mode_change->hand_roll_1_motor.hand_motor_measure_all.relative_angle; 					
		
		hand_mode_change->hand_behaviour_last = hand_mode_change->hand_behaviour;
	}
	else if (hand_mode_change->hand_behaviour_last != HAND_MOTIONLESS && hand_mode_change-> hand_behaviour == HAND_MOTIONLESS)
    {
		hand_mode_change->hand_yaw_5_motor.hand_motor_measure_all.relative_angle_set 	            = hand_mode_change->hand_yaw_5_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_x_4_motor.hand_motor_measure_all.relative_angle_set                  = hand_mode_change->hand_x_4_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_roll_3_motor.hand_motor_measure_all.relative_angle_set 				= hand_mode_change->hand_roll_3_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_pitch_2_motor.hand_motor_measure_all.relative_angle_set 				= hand_mode_change->hand_pitch_2_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_roll_1_motor.hand_motor_measure_all.relative_angle_set 			    = hand_mode_change->hand_roll_1_motor.hand_motor_measure_all.relative_angle; 				


    hand_mode_change->hand_yaw_5_motor.hand_motor_measure_all.lock_angle                      	= hand_mode_change->hand_yaw_5_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_x_4_motor.hand_motor_measure_all.lock_angle                          = hand_mode_change->hand_x_4_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_roll_3_motor.hand_motor_measure_all.lock_angle 			         	= hand_mode_change->hand_roll_3_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_pitch_2_motor.hand_motor_measure_all.lock_angle 			         	= hand_mode_change->hand_pitch_2_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_roll_1_motor.hand_motor_measure_all.lock_angle 					    = hand_mode_change->hand_roll_1_motor.hand_motor_measure_all.relative_angle;				
	

		hand_mode_change->hand_behaviour_last = hand_mode_change->hand_behaviour;
	}
	else if (hand_mode_change->hand_behaviour_last != HAND_CUSTOM && hand_mode_change-> hand_behaviour == HAND_CUSTOM) 
	{
		hand_mode_change->hand_yaw_5_motor.hand_motor_measure_all.relative_angle_set         	= hand_mode_change->hand_yaw_5_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_x_4_motor.hand_motor_measure_all.relative_angle_set            = hand_mode_change->hand_x_4_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_roll_3_motor.hand_motor_measure_all.relative_angle_set 				= hand_mode_change->hand_roll_3_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_pitch_2_motor.hand_motor_measure_all.relative_angle_set 				= hand_mode_change->hand_pitch_2_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_roll_1_motor.hand_motor_measure_all.relative_angle_set 				= hand_mode_change->hand_roll_1_motor.hand_motor_measure_all.relative_angle; 				


    hand_mode_change->hand_yaw_5_motor.hand_motor_measure_all.lock_angle 	                    = hand_mode_change->hand_yaw_5_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_x_4_motor.hand_motor_measure_all.lock_angle                          = hand_mode_change->hand_x_4_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_roll_3_motor.hand_motor_measure_all.lock_angle 			         	= hand_mode_change->hand_roll_3_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_pitch_2_motor.hand_motor_measure_all.lock_angle 			        	= hand_mode_change->hand_pitch_2_motor.hand_motor_measure_all.relative_angle;
		hand_mode_change->hand_roll_1_motor.hand_motor_measure_all.lock_angle 					    = hand_mode_change->hand_roll_1_motor.hand_motor_measure_all.relative_angle; 					
		
		hand_mode_change->hand_behaviour_last = hand_mode_change->hand_behaviour;
	}
}

/// @brief 机械臂遥控器，键鼠设置位置
/// @param  
static void	Hand_Set_Position(Hand_Control_t *hand_set_position)
{
	if (hand_set_position == NULL)
    {
        return;
    }

	if (hand_set_position->gimbal_rc_ctrl->mouse.press_l)//下一步操作
	{
		hand_set_position->hand_behaviour = HAND_CUSTOM;
		hand_set_position->hand_point += 1 ;
	}
	else if (hand_set_position->gimbal_rc_ctrl->mouse.press_r)//上一步操作
	{
		hand_set_position->hand_behaviour = HAND_CUSTOM;
		if (hand_set_position->hand_behaviour != hand_point_1)
		{
			hand_set_position->hand_point -= 1 ;
		}		
	}	
	switch(hand_set_position->gimbal_rc_ctrl->key.v)    
	{	
		case KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_Z ://自定义控制器模式
			hand_set_position->hand_behaviour = HAND_CUSTOM;
			hand_set_position->hand_point = hand_point_1;
			hand_set_position->hand_position = hand_position_N;
			break;
		case KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_B ://切回传统控制模式(会中断自定义控制器，视觉，一键运动)
			hand_set_position->hand_behaviour = HAND_all_Operation;
			hand_set_position->hand_point = hand_point_1;
			hand_set_position->hand_position = hand_position_N;
			break;

		case KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_Z : 
			hand_set_position->hand_position  = hand_position_1;
			hand_set_position->hand_point = hand_point_1;
			hand_set_position->hand_behaviour = HAND_CUSTOM;
			break;
		case KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_X : 
			hand_set_position->hand_position  = hand_position_2;
			hand_set_position->hand_point = hand_point_1;
			hand_set_position->hand_behaviour = HAND_CUSTOM;
			break;
		case KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_C : 
			hand_set_position->hand_position  = hand_position_3;		
			hand_set_position->hand_point = hand_point_1;	
			hand_set_position->hand_behaviour = HAND_CUSTOM;
			break;
		case KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_V : 
			hand_set_position->hand_position  = hand_position_4;
			hand_set_position->hand_point = hand_point_1;
			hand_set_position->hand_behaviour = HAND_CUSTOM;
			break;
		case KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_B: 
			hand_position_Init(&hand_control);                 //机械臂位置初始化
			hand_position_Init(&hand_control);                 //机械臂位置初始化
			hand_position_Init(&hand_control);                 //机械臂位置初始化
			hand_position_Init(&hand_control);                 //机械臂位置初始化
			hand_position_Init(&hand_control);                 //机械臂位置初始化
		    hand_set_position->hand_behaviour = HAND_ZERO_FORCE;

			break;
		case KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_G: 
			__set_FAULTMASK(1); //关闭所有中断
			NVIC_SystemReset(); //进行软件复位
			break;	
	  default:  break;
	}
	
	if(hand_set_position->hand_behaviour==HAND_CUSTOM)
	{
		if(hand_set_position->hand_position == hand_position_N)      
		{
			//Hand_Custom_Set(hand_set_position);//自定义控制器 
		}
		else if (hand_set_position->hand_position != hand_position_N)
		{
			HAND_combination_control(hand_set_position, 
									 joint_angles[4][hand_set_position->hand_position][hand_set_position->hand_point],
									 joint_angles[3][hand_set_position->hand_position][hand_set_position->hand_point],
									 joint_angles[2][hand_set_position->hand_position][hand_set_position->hand_point],
									 joint_angles[1][hand_set_position->hand_position][hand_set_position->hand_point],
									 joint_angles[0][hand_set_position->hand_position][hand_set_position->hand_point]);
		}
	}
}

static void Hand_Custom_Set(Hand_Control_t *hand_set_position)
{
	if (hand_set_position == NULL)
    {
        return;
    }
	
	
    // HAND_relative_angle_limit(&hand_set_position->hand_yaw_5_motor.hand_motor_measure_all	,/**/processValue(hand_set_position->hand_custom_control->motor_position4 ,0.5,yaw_5_custom_rate)       ,yaw_5_relative_angle_set_max	   	     ,yaw_5_custom_angle_set_min);
	// HAND_relative_angle_limit(&hand_set_position->hand_x_4_motor.hand_motor_measure_all	    ,/**/hand_set_position->hand_custom_control->motor_position3   *x_4_custom_rate         ,x_4_relative_angle_set_max	                 ,x_4_relative_angle_set_min);
	// HAND_relative_angle_limit(&hand_set_position->hand_roll_3_motor.hand_motor_measure_all	,/**/hand_set_position->hand_custom_control->motor_position2   *roll_3_custom_rate      ,roll_3_relative_angle_set_max	       ,roll_3_relative_angle_set_min);
	// if(hand_set_position->pitch_2_angle_fact > -100.0f && hand_set_position->pitch_2_angle_fact < -10.0f) 
	// {
	// 	HAND_relative_angle_limit(&hand_set_position->hand_pitch_2_motor.hand_motor_measure_all	,/**/processValue(hand_set_position->hand_custom_control->motor_position1 ,0.5,pitch_2_custom_rate)     ,pitch_2_relative_angle_set_max	         ,pitch_2_relative_angle_set_min);
	// 	HAND_relative_angle_limit(&hand_set_position->hand_roll_1_motor.hand_motor_measure_all	,/**/ processValue(hand_set_position->hand_custom_control->motor_position1 ,0.5,pitch_2_custom_rate)       ,roll_1_relative_angle_set_max	      ,roll_1_relative_angle_set_min);
	// }
}

/// @brief 电机移动到一个指定位置 可用于轨迹规划，运动分解
/// @param hand_combination_control 
/// @param relative_angle_set_5 
/// @param relative_angle_set_4 
/// @param relative_angle_set_3 
/// @param relative_angle_set_2 
static void HAND_combination_control(Hand_Control_t *hand_combination_control,fp32 relative_angle_set_5,fp32 relative_angle_set_4,fp32 relative_angle_set_3,fp32 relative_angle_set_2 , fp32 relative_angle_set_1 )
{

	static fp32 add_1 , add_2 , add_3 , add_4, add_5 ;
	double epsilon = 1e-9;
    if (fabs(relative_angle_set_5) < epsilon && fabs(relative_angle_set_4) < epsilon && fabs(relative_angle_set_3) < epsilon && fabs(relative_angle_set_2) < epsilon && fabs(relative_angle_set_1) < epsilon) 
		{
			hand_combination_control->hand_yaw_5_motor.hand_motor_measure_all.lock_angle	            =hand_combination_control->hand_yaw_5_motor.hand_motor_measure_all.relative_angle;
			hand_combination_control->hand_x_4_motor.hand_motor_measure_all.lock_angle	                =hand_combination_control->hand_x_4_motor.hand_motor_measure_all.relative_angle;
			hand_combination_control->hand_roll_3_motor.hand_motor_measure_all.lock_angle				=hand_combination_control->hand_roll_3_motor.hand_motor_measure_all.relative_angle;
			hand_combination_control->hand_pitch_2_motor.hand_motor_measure_all.lock_angle				=hand_combination_control->hand_pitch_2_motor.hand_motor_measure_all.relative_angle;

			hand_combination_control->hand_behaviour = HAND_MOTIONLESS;
			hand_combination_control->hand_position  = hand_position_N;
		}

		if(fabs(hand_combination_control->hand_yaw_5_motor.hand_motor_measure_all.relative_angle-relative_angle_set_5)>HAND_ANGLE_ERROR5)
		{
			if((relative_angle_set_5-hand_combination_control->hand_yaw_5_motor.hand_motor_measure_all.relative_angle)>=0.0f)
			{
			add_5 = Keyboard_set_Angle_increment_hand_5;
			}
			else add_5 = -Keyboard_set_Angle_increment_hand_5;
			
			HAND_relative_angle_limit(&hand_combination_control->hand_yaw_5_motor.hand_motor_measure_all	,/**/add_5,yaw_5_relative_angle_set_max		,yaw_5_relative_angle_set_min);
			
		}
		
		// if(fabs(hand_combination_control->hand_x_4_motor.hand_motor_measure_all.relative_angle-relative_angle_set_4)>HAND_ANGLE_ERROR4)
		// {
		// 	if((relative_angle_set_4-hand_combination_control->hand_x_4_motor.hand_motor_measure_all.relative_angle)>=0.0f)
		// 	{
		// 	add_4 = Keyboard_set_Angle_increment_hand_4;
		// 	}
		// 	else add_4 = -Keyboard_set_Angle_increment_hand_4;
			
		// 	HAND_relative_angle_limit(&hand_combination_control->hand_x_4_motor.hand_motor_measure_all	,/**/add_4,x_4_relative_angle_set_max	,x_4_relative_angle_set_min);
		// }
		
		if(fabs(hand_combination_control->hand_roll_3_motor.hand_motor_measure_all.relative_angle-relative_angle_set_3)>HAND_ANGLE_ERROR3)
		{
			if((relative_angle_set_3-hand_combination_control->hand_roll_3_motor.hand_motor_measure_all.relative_angle)>=0.0f)
			{
			add_3 = Keyboard_set_Angle_increment_hand_3;
			}
			else add_3 = -Keyboard_set_Angle_increment_hand_3;
			
			HAND_relative_angle_limit(&hand_combination_control->hand_roll_3_motor.hand_motor_measure_all	,/**/add_3	,roll_3_relative_angle_set_max	,roll_3_relative_angle_set_min);
		
		}
	
//		if(fabs(hand_combination_control->hand_pitch_2_motor.hand_motor_measure_all.relative_angle-relative_angle_set_2)>HAND_ANGLE_ERROR2)
//		{
//			if((relative_angle_set_2-hand_combination_control->hand_pitch_2_motor.hand_motor_measure_all.relative_angle)>=0.0f)
//			{
//				add_2 = Keyboard_set_Angle_increment_hand_2;
//			}
//			else add_2 = -Keyboard_set_Angle_increment_hand_2;
//			
//			HAND_relative_angle_limit(&hand_combination_control->hand_pitch_2_motor.hand_motor_measure_all	,/**/add_2	,pitch_2_relative_angle_set_max	,pitch_2_relative_angle_set_min);
//		
//		}
//		if(fabs(hand_combination_control->hand_roll_1_motor.hand_motor_measure_all.relative_angle-relative_angle_set_1)>HAND_ANGLE_ERROR1)
//		{
//			if((relative_angle_set_1-hand_combination_control->hand_roll_1_motor.hand_motor_measure_all.relative_angle)>=0.0f)
//			{
//				add_1 = Keyboard_set_Angle_increment_hand_1;
//			}
//			else add_1 = -Keyboard_set_Angle_increment_hand_1;
//			
//			HAND_relative_angle_limit(&hand_combination_control->hand_roll_1_motor.hand_motor_measure_all	,/**/add_1	,roll_1_relative_angle_set_max	,roll_1_relative_angle_set_min);
//		
//		}
		
		if((fabs(hand_combination_control->hand_yaw_5_motor.hand_motor_measure_all.relative_angle-relative_angle_set_5)<=HAND_ANGLE_ERROR5)\
			&&(fabs(hand_combination_control->hand_x_4_motor.hand_motor_measure_all.relative_angle-relative_angle_set_4)<=HAND_ANGLE_ERROR4)\
			&&(fabs(hand_combination_control->hand_roll_3_motor.hand_motor_measure_all.relative_angle-relative_angle_set_3)<=HAND_ANGLE_ERROR3))
//			&&(fabs(hand_combination_control->hand_pitch_2_motor.hand_motor_measure_all.relative_angle-relative_angle_set_2)<=HAND_ANGLE_ERROR2)\
//			&&(fabs(hand_combination_control->hand_roll_1_motor.hand_motor_measure_all.relative_angle-relative_angle_set_1)<=HAND_ANGLE_ERROR1))
		{
			hand_combination_control->hand_yaw_5_motor.hand_motor_measure_all.lock_angle	            =hand_combination_control->hand_yaw_5_motor.hand_motor_measure_all.relative_angle;
			hand_combination_control->hand_x_4_motor.hand_motor_measure_all.lock_angle	                =hand_combination_control->hand_x_4_motor.hand_motor_measure_all.relative_angle;
			hand_combination_control->hand_roll_3_motor.hand_motor_measure_all.lock_angle				=hand_combination_control->hand_roll_3_motor.hand_motor_measure_all.relative_angle;
			hand_combination_control->hand_pitch_2_motor.hand_motor_measure_all.lock_angle				=hand_combination_control->hand_pitch_2_motor.hand_motor_measure_all.relative_angle;
			hand_combination_control->hand_roll_1_motor.hand_motor_measure_all.lock_angle				=hand_combination_control->hand_roll_1_motor.hand_motor_measure_all.relative_angle;
			hand_combination_control->hand_behaviour = HAND_MOTIONLESS;
		}
}

/**
 * @brief 机械臂控制PID计算
 * @param hand_control_loop 
 */
void Hand_Control_loop(void)
{			
  Hand_Control_t *hand_control_loop=&hand_control;

	if(hand_control_loop == NULL)   return; 

	if(hand_control_loop->hand_behaviour==HAND_CUSTOM)
	{	

		hand_motor_GM_relative_angle_control        (&hand_control_loop->hand_yaw_5_motor.GM_Send_Data                      ,&hand_control_loop->hand_yaw_5_motor.hand_motor_measure_all);
		hand_motor_relative_angle_control			(&hand_control_loop->hand_x_4_motor.hand_motor_measure_all	            ,x_4_3508_SPEED_RATE);
    hand_motor_GM_relative_angle_control        (&hand_control_loop->hand_roll_3_motor.GM_Send_Data                     ,&hand_control_loop->hand_roll_3_motor.hand_motor_measure_all);
		hand_motor_relative_angle_control			(&hand_control_loop->hand_pitch_2_motor.hand_motor_measure_all	        ,pitch_2_3508_SPEED_RATE);
		hand_motor_relative_angle_control			(&hand_control_loop->hand_roll_1_motor.hand_motor_measure_all	        ,roll_1_2006_SPEED_RATE);

	}
    else if(hand_control_loop-> hand_behaviour == HAND_ZERO_FORCE)
    {
		hand_motor_GM_raw_angle_control       		(&hand_control_loop->hand_yaw_5_motor.GM_Send_Data);
		hand_motor_raw_angle_control        	    (&hand_control_loop->hand_x_4_motor.hand_motor_measure_all);
		hand_motor_GM_raw_angle_control             (&hand_control_loop->hand_roll_3_motor.GM_Send_Data);
		hand_motor_raw_angle_control				(&hand_control_loop->hand_pitch_2_motor.hand_motor_measure_all);
		hand_motor_raw_angle_control				(&hand_control_loop->hand_roll_1_motor.hand_motor_measure_all);

    }
	else if (hand_control_loop-> hand_behaviour == HAND_MOTIONLESS)//不用lock_angle是为了方便接入图传，在遥控器其他挡位也可以通过改变relaitive_angle来控制
    {
		hand_motor_GM_relative_angle_control        (&hand_control_loop->hand_yaw_5_motor.GM_Send_Data                      ,&hand_control_loop->hand_yaw_5_motor.hand_motor_measure_all);
		hand_motor_relative_angle_control			(&hand_control_loop->hand_x_4_motor.hand_motor_measure_all	            ,x_4_3508_SPEED_RATE);
    hand_motor_GM_relative_angle_control        (&hand_control_loop->hand_roll_3_motor.GM_Send_Data                     ,&hand_control_loop->hand_roll_3_motor.hand_motor_measure_all);
		hand_motor_relative_angle_control			(&hand_control_loop->hand_pitch_2_motor.hand_motor_measure_all			,pitch_2_3508_SPEED_RATE);
		hand_motor_relative_angle_control			(&hand_control_loop->hand_roll_1_motor.hand_motor_measure_all			,roll_1_2006_SPEED_RATE);

    }
	else if(hand_control_loop-> hand_behaviour == HAND_all_Operation)	
	{
		hand_motor_GM_relative_angle_control        (&hand_control_loop->hand_yaw_5_motor.GM_Send_Data                      ,&hand_control_loop->hand_yaw_5_motor.hand_motor_measure_all);
		hand_motor_relative_angle_control			(&hand_control_loop->hand_x_4_motor.hand_motor_measure_all	            ,x_4_3508_SPEED_RATE);
    hand_motor_GM_relative_angle_control        (&hand_control_loop->hand_roll_3_motor.GM_Send_Data                     ,&hand_control_loop->hand_roll_3_motor.hand_motor_measure_all);
		hand_motor_relative_angle_control			(&hand_control_loop->hand_pitch_2_motor.hand_motor_measure_all			,pitch_2_3508_SPEED_RATE);
		hand_motor_relative_angle_control			(&hand_control_loop->hand_roll_1_motor.hand_motor_measure_all	        ,roll_1_2006_SPEED_RATE);
	}
	
}

/**
 * @brief RM电机速度环
 * @param hand_motor 
 * @param SPEED_RATE 速度比例
 */
static void hand_motor_speed_control(Hand_Motor_t *hand_motor,fp32 SPEED_RATE)
{
    if (hand_motor == NULL)
    {
        return;
    }
	hand_motor->current_set = PID_Calc(&hand_motor->hand_motor_speed_pid,
	hand_motor->hand_motor_measure->speed_rpm*SPEED_RATE, hand_motor->speed_set);
    hand_motor->given_current = (int16_t)(hand_motor->current_set);
}
/// @brief RM电机停止
/// @param hand_motor 
static void hand_motor_raw_angle_control(Hand_Motor_t *hand_motor)
{
    if (hand_motor == NULL)
    {
        return;
    }
    hand_motor->current_set = 0;
    hand_motor->given_current = (int16_t)(hand_motor->current_set);
}
/// @brief GM电机停止
/// @param motor_send 
static void hand_motor_GM_raw_angle_control(MOTOR_send *motor_send)
{
	if (motor_send == NULL) return ;
	motor_send->mode = 0;

	motor_send->GM_Send_Effort = 0;
	motor_send->GM_Send_Pos = 0;
	motor_send->GM_Send_speed = 0;
	motor_send->GM_Send_Kd_Speed=0;
	motor_send->GM_Send_Kp_Pos = 0;

}
/// @brief RM电机相对角度串级控制
/// @param hand_motor 
/// @param SPEED_RATE 
static void hand_motor_relative_angle_control(Hand_Motor_t *hand_motor,fp32 SPEED_RATE)
{
     if (hand_motor == NULL)
    {
        return;
    }
    hand_motor->motor_position_set =  PID_Calc(&hand_motor->hand_motor_relative_pid , hand_motor->relative_angle, hand_motor->relative_angle_set);
	  hand_motor->current_set        =  PID_Calc(&hand_motor->hand_motor_speed_pid, hand_motor->hand_motor_measure->speed_rpm*SPEED_RATE, hand_motor->motor_position_set);
    hand_motor->given_current      =  (int16_t)(hand_motor->current_set);
}

static int same_sign(float num1, float num2) {
    if ((num1 >= 0 && num2 >= 0) || (num1 < 0 && num2 < 0)) {
        return 1; // 返回1表示具有相同的符号
    } else {
        return 0; // 返回0表示具有不同的符号
    }
}
/// @brief GM电机相对角度串级控制
/// @param motor_send 
/// @param relative_angle_set 
static void hand_motor_GM_relative_angle_control(MOTOR_send *motor_send , Hand_Motor_t *hand_motor)
{
	if (motor_send == NULL) return ;
	
	motor_send->mode = 1;
	motor_send->GM_Send_Pos = 0;
	motor_send->GM_Send_speed = 0;
	motor_send->GM_Send_Kd_Speed=0;
	motor_send->GM_Send_Kp_Pos=0;

		
	hand_motor->motor_position_set =  PID_Calc(&hand_motor->hand_motor_relative_pid , hand_motor->relative_angle, hand_motor->relative_angle_set);

	motor_send->GM_Send_speed = hand_motor->motor_position_set;
	
	if(motor_send->id == GM_Yaw_5_ID)
	{
		motor_send->GM_Send_Kd_Speed = yaw_5_SPEED_PID_KP; //0.120000996
	}

	else if(motor_send->id == GM_roll_3_ID)
	{
		motor_send->GM_Send_Kd_Speed = roll_3_SPEED_PID_KP; //0.120000996	
	}
	
}
/// @brief RM电机自锁角度串级控制
/// @param hand_motor 
/// @param SPEED_RATE 
static void hand_motor_lock_angle_angle_control(Hand_Motor_t *hand_motor,fp32 SPEED_RATE)
{
	     if (hand_motor == NULL)
    {
        return;
    }
    hand_motor->motor_position_set =  PID_Calc(&hand_motor->hand_motor_relative_pid , hand_motor->relative_angle, hand_motor->lock_angle);
	hand_motor->current_set = PID_Calc(&hand_motor->hand_motor_speed_pid, hand_motor->hand_motor_measure->speed_rpm*SPEED_RATE, hand_motor->motor_position_set);
    hand_motor->given_current = (int16_t)(hand_motor->current_set);
}
/// @brief GM电机自锁角度串级控制
/// @param motor_send 
/// @param lock_angle 
static void hand_motor_GM_lock_angle_control(MOTOR_send *motor_send , Hand_Motor_t *hand_motor)
{
	if (motor_send == NULL) return ;
	
	motor_send->mode = 1;
	motor_send->GM_Send_Pos = 0;
	motor_send->GM_Send_speed = 0;
	motor_send->GM_Send_Kd_Speed=0;
	motor_send->GM_Send_Kp_Pos=0;

	hand_motor->motor_position_set =  PID_Calc(&hand_motor->hand_motor_relative_pid , hand_motor->relative_angle, hand_motor->lock_angle);
	hand_motor->current_set        =  PID_Calc(&hand_motor->hand_motor_speed_pid, hand_motor->speed_ref , hand_motor->motor_position_set);
        
	motor_send->GM_Send_Effort = hand_motor->current_set;

}

/**
 * @brief 机械臂数据反馈
 * @param hand_motor_feedback_update 
 */
void Hand_Feedback_Update(void) //0.0000271314538043478
{
  Hand_Control_t *hand_motor_feedback_update=&hand_control;

  if (hand_motor_feedback_update == NULL)
  {
      return;
  }

  /*****保留上一个反馈数据*****/
  hand_motor_feedback_update->hand_roll_1_motor.hand_motor_measure_all.lsat_relative_angle          = hand_motor_feedback_update->hand_roll_1_motor.hand_motor_measure_all.relative_angle;
  hand_motor_feedback_update->hand_pitch_2_motor.hand_motor_measure_all.lsat_relative_angle         = hand_motor_feedback_update->hand_pitch_2_motor.hand_motor_measure_all.relative_angle;
  hand_motor_feedback_update->hand_roll_3_motor.hand_motor_measure_all.lsat_relative_angle          = hand_motor_feedback_update->hand_roll_3_motor.hand_motor_measure_all.relative_angle;
  hand_motor_feedback_update->hand_x_4_motor.hand_motor_measure_all.lsat_relative_angle             = hand_motor_feedback_update->hand_x_4_motor.hand_motor_measure_all.last_encoder_angle              = hand_motor_feedback_update->hand_x_4_motor.hand_motor_measure_all.relative_angle;
  hand_motor_feedback_update->hand_yaw_5_motor.hand_motor_measure_all.lsat_relative_angle           = hand_motor_feedback_update->hand_yaw_5_motor.hand_motor_measure_all.relative_angle;
	
	//roll1 hollow
//    hand_motor_feedback_update->hand_roll_1_motor.hand_motor_measure_all.motor_encoder_angle          = hand_motor_hollow_encoder_to_relative_angle_change(hand_motor_feedback_update->hand_roll_1_motor.hollowEncoderMotor , roll_1_encoder_ecd_offset , roll_1_encoder_angle_offset , hand_motor_feedback_update->hand_roll_1_motor.hand_motor_measure_all.motor_encoder_angle);
  hand_motor_feedback_update->hand_roll_1_motor.hand_motor_measure_all.motor_encoder_angle           = hand_motor_ecd_to_angle_change(hand_motor_feedback_update->hand_roll_1_motor.hand_motor_measure_all.hand_motor_measure->ecd , roll_1_relative_ecd_offset , roll_1_relative_angle_offset );
  hand_motor_feedback_update->hand_roll_1_motor.hand_motor_measure_all.relative_angle                = hand_motor_cic_relative_angle(hand_motor_feedback_update->hand_roll_1_motor.hand_motor_measure_all.motor_encoder_angle ,hand_motor_feedback_update->hand_roll_1_motor.hand_motor_measure_all.hand_motor_measure->crc , hand_motor_feedback_update->hand_roll_1_motor.hand_motor_measure_all.relative_init_angle);

	//pitch2 hollow
  hand_motor_feedback_update->hand_pitch_2_motor.hand_motor_measure_all.motor_encoder_angle          = hand_motor_ecd_to_angle_change(hand_motor_feedback_update->hand_pitch_2_motor.hand_motor_measure_all.hand_motor_measure->ecd , pitch_2_relative_ecd_offset , pitch_2_relative_angle_offset);
	hand_motor_feedback_update->hand_pitch_2_motor.hand_motor_measure_all.relative_angle               = -hand_motor_cic_relative_angle(hand_motor_feedback_update->hand_pitch_2_motor.hand_motor_measure_all.motor_encoder_angle ,hand_motor_feedback_update->hand_pitch_2_motor.hand_motor_measure_all.hand_motor_measure->crc , hand_motor_feedback_update->hand_pitch_2_motor.hand_motor_measure_all.relative_init_angle);

  hand_motor_feedback_update->pitch_2_angle_fact = (hand_motor_feedback_update->hand_roll_1_motor.hand_motor_measure_all.relative_angle + hand_motor_feedback_update->hand_pitch_2_motor.hand_motor_measure_all.relative_angle)/2;
  hand_motor_feedback_update->roll_1_angle_fact = hand_motor_feedback_update->hand_roll_1_motor.hand_motor_measure_all.relative_angle - hand_motor_feedback_update->pitch_2_angle_fact;
	
	
//	hand_motor_feedback_update->hand_pitch_2_motor.hand_motor_measure_all.original_RM_angle           = hand_motor_ecd_to_angle_2PI(hand_motor_feedback_update->hand_pitch_2_motor.hand_motor_measure_all.hand_motor_measure->ecd , pitch_2_relative_ecd_offset , pitch_2_relative_angle_offset);
//    hand_motor_feedback_update->hand_pitch_2_motor.hand_motor_measure_all.relative_angle              = hand_motor_hollow_encoder_to_circle_angle (hand_motor_feedback_update->hand_pitch_2_motor.hand_motor_measure_all , hand_motor_feedback_update->hand_pitch_2_motor.hand_motor_measure_all.original_RM_angle , pitch_2_reduction_radio);

	//yaw3 关节电机
	hand_motor_feedback_update->hand_roll_3_motor.hand_motor_measure_all.relative_angle               = hand_motor_feedback_update->hand_roll_3_motor.hand_motor_measure_all.motor_encoder_angle        = hand_motor_feedback_update->hand_roll_3_motor.rData->Pos - hand_motor_feedback_update->hand_roll_3_motor.hand_motor_measure_all.relative_init_angle;
    hand_motor_feedback_update->hand_roll_3_motor.hand_motor_measure_all.speed_ref                    = hand_motor_feedback_update->hand_roll_3_motor.rData->W;

	//x4 3508 
    hand_motor_feedback_update->hand_x_4_motor.hand_motor_measure_all.original_RM_angle               = hand_motor_ecd_to_angle_change(hand_motor_feedback_update->hand_x_4_motor.hand_motor_measure_all.hand_motor_measure->ecd , x_4_encoder_ecd_offset , x_4_encoder_angle_offset);
    hand_motor_feedback_update->hand_x_4_motor.hand_motor_measure_all.motor_encoder_angle             = hand_motor_feedback_update->hand_x_4_motor.hand_motor_measure_all.relative_angle                = encoder_ecd_to_relative_expansion(hand_motor_feedback_update->hand_x_4_motor.hand_encoder_4_measure->ecd , hand_motor_feedback_update->hand_x_4_motor.hand_motor_measure_all.motor_circle , hand_motor_feedback_update->hand_x_4_motor.hand_motor_measure_all.relative_angle , x_4_ecd_offset , x_4_expansion_offset);//
	
	//yaw5 关节电机
	hand_motor_feedback_update->hand_yaw_5_motor.hand_motor_measure_all.relative_angle                = hand_motor_feedback_update->hand_yaw_5_motor.rData->Pos - hand_motor_feedback_update->hand_yaw_5_motor.hand_motor_measure_all.relative_init_angle;	
	hand_motor_feedback_update->hand_yaw_5_motor.hand_motor_measure_all.speed_ref                     = hand_motor_feedback_update->hand_yaw_5_motor.rData->W;

}

static double abs(double x) 
{
  if (x < 0) {
    return -x;
  } else {
    return x;
  }
}  
//第一圈从4096开始，防止编码器溢出
//行程一共两圈 4096 32768 和 0 32768
static fp32 encoder_ecd_to_relative_expansion(uint16_t ecd, int32_t circle_measure ,float last_relative_angle , int32_t offset_ecd, float offect_height)
{
	int32_t relative_ecd = ecd + offset_ecd + circle_measure * (32768 - 4096);
	float relative_expansion = relative_ecd * Motor_Ecd_to_expansion + offect_height; 
	if(relative_expansion < 0)
	{
		relative_expansion = 0;
	}
	if(abs(relative_expansion - last_relative_angle) > 0.07f)
	{
		return relative_expansion;
	}
	return relative_expansion;
}

/// @brief RM电机角度值报文转换相对角度  -3.14至3.14
/// @param ecd 
/// @param offset_ecd 角度偏移值
/// @return 
static fp32 hand_motor_ecd_to_angle_change(int16_t ecd, int32_t offset_ecd , fp32 offset_angle)
{
    int32_t relative_ecd = ecd + offset_ecd;
	static fp32 relative_angle;

	relative_angle = (relative_ecd*1.0/ecd_range - 0.5) *2* hollow_encoder_pi + offset_angle;

    if (relative_angle > hollow_encoder_pi)
	{
		relative_angle = relative_angle - 2*hollow_encoder_pi;
	}
	else if (relative_angle < -hollow_encoder_pi)
	{
		relative_angle = relative_angle + 2*hollow_encoder_pi;
	}
	
	return relative_angle;

}

/// @brief RM电机角度值报文转换相对角度  0至6.28
/// @param ecd 
/// @param offset_ecd 角度偏移值
/// @return 
static fp32 hand_motor_ecd_to_angle_2PI(int16_t ecd, int32_t offset_ecd , fp32 offset_angle)
{
    int32_t relative_ecd = ecd + offset_ecd;
	static fp32 relative_angle;

	relative_angle = (relative_ecd*1.0/ecd_range) *2* hollow_encoder_pi + offset_angle;

    if (relative_angle > 2 * hollow_encoder_pi)
	{
		relative_angle = relative_angle - 2*hollow_encoder_pi;
	}
	else if (relative_angle < 0)
	{
		relative_angle = relative_angle + 2*hollow_encoder_pi;
	}
	
	return relative_angle;

}

static fp32 hand_motor_cic_relative_angle(float encoder_one_ecd, int64_t cic_mount , float offset_relative_ecd )
{
	return cic_mount* 2 * PI + offset_relative_ecd + encoder_one_ecd;
}

	
/**
 * @brief 中空编码器占空比转换相对角度
 * @param hand_hollow_encoder_change 
 * @param TIM_channel 对应通道
 * @param offset_ecd  角度偏移值
 * @param last_relative_angle 
 * @return HollowEncodermotor 返回值	 
 * @note 24/10/19 原差速器使用中空编码器，未被使用
 */
static  fp32  hand_motor_hollow_encoder_to_relative_angle_change(const EncoderHollowMotor_t* hand_hollow_encoder_change,int32_t offset_ecd, fp32 offset_angle , fp32 last_relative_angle)
{
  static fp32 HollowEncodermotor=0;int32_t hollowEncoderEcd=0;

	if (hand_hollow_encoder_change == NULL)
    {
         return last_relative_angle;
    }

	if (hand_hollow_encoder_change->hollowEncoderOriginaEcd<0)//定时器在两次捕获中溢出 做处理
	{
	    hollowEncoderEcd = hand_hollow_encoder_change->hollowEncoderOriginaEcd + hand_hollow_encoder_change->hollowEncoderSum + offset_ecd ;
	}
	else hollowEncoderEcd = hand_hollow_encoder_change->hollowEncoderOriginaEcd + offset_ecd;//OFFSET=SUM/2-ECD

	HollowEncodermotor = (hollowEncoderEcd*1.0/hand_hollow_encoder_change->hollowEncoderSum - 0.5) *2* hollow_encoder_pi + offset_angle;

    if (HollowEncodermotor > hollow_encoder_pi)
	{
		HollowEncodermotor = HollowEncodermotor - 2*hollow_encoder_pi;
	}
	else if (HollowEncodermotor < -hollow_encoder_pi)
	{
		HollowEncodermotor = HollowEncodermotor + 2*hollow_encoder_pi;
	}	
	
	
	if (HollowEncodermotor == 0) 
	{
		return last_relative_angle;
	}
		if (last_relative_angle == 0) 
	{
		return HollowEncodermotor;
	}
	if (HollowEncodermotor - last_relative_angle > 0.05f || HollowEncodermotor - last_relative_angle < -0.05f)
	{
		return last_relative_angle;
	}
		
	if (HollowEncodermotor > -hollow_encoder_pi && HollowEncodermotor < hollow_encoder_pi)
	{
		return HollowEncodermotor;
	}
	
	return last_relative_angle;	
}	


  static fp32 circle_angle;//多圈角度值
/// @brief 由中空编码器值 计算电机多圈角度值
/// @param hollow_encoder_angle      中空编码器角度值
/// @param relative_angle_mid        电机单圈值 传入0到2PI的值
/// @param reduction_ratio           电机减速比
/// @return 

static fp32 hand_motor_hollow_encoder_to_circle_angle(Hand_Motor_t hand_motor_measure , fp32 relative_angle_mid , uint8_t reduction_ratio)
{
	int attempt = 3;	
	fp32 relative_angle_measure; //电机单圈值转化为减速值
	fp32 circle_motor;      //电机转动一圈，末端运动角度
	fp32 circle_count;      //中空编码器值计算电机圈数

	circle_motor = 2*PI/reduction_ratio;
	circle_count=(int)(hand_motor_measure.motor_encoder_angle/circle_motor);
	if(hand_motor_measure.motor_encoder_angle < 0)
	{
		circle_count--;
	}
	relative_angle_measure = relative_angle_mid/reduction_ratio;
	circle_angle=circle_count*circle_motor+relative_angle_measure;
	
//	do {
//    if (circle_angle > hand_motor_measure.motor_encoder_angle + circle_motor*1/2)
//    {
//        circle_angle -= circle_motor;
//    }
//    else if (circle_angle < hand_motor_measure.motor_encoder_angle - circle_motor*1/2)
//    {
//        circle_angle += circle_motor;
//    }
//    attempt--;
//	} while ((circle_angle > hand_motor_measure.motor_encoder_angle + circle_motor*1/2 || circle_angle < hand_motor_measure.motor_encoder_angle - circle_motor*1/2) && attempt > 0);
//	if (attempt==0)
//	{
//		circle_angle = hand_motor_measure.motor_encoder_angle;
//	}
	return circle_angle;
}	


/// @brief 计量圈数
/// @param hand_motor 
/// @return 
static void hand_motor_circle_measure(Hand_Motor_t *hand_motor)
{
    if (hand_motor == NULL) // 空指针检查
    {
        return;
    }
    float angle_change = hand_motor->last_encoder_angle - hand_motor->motor_encoder_angle;

    if (angle_change > -0.15f && hand_motor->last_encoder_angle < -0.1f && hand_motor->motor_encoder_angle > 0.1f)
    {
        hand_motor->motor_circle++;
    }
	if (angle_change > 3.00f && hand_motor->last_encoder_angle > 1.47f && hand_motor->motor_encoder_angle < -1.47f)
    {
        hand_motor->motor_circle++;
    }
    if (angle_change > 0.15f && hand_motor->last_encoder_angle > 0.1f && hand_motor->motor_encoder_angle < -0.1f)
    {
        hand_motor->motor_circle--;
    }
	if (angle_change > -3.00f && hand_motor->last_encoder_angle < -1.47f && hand_motor->motor_encoder_angle > 1.47f )
    {
        hand_motor->motor_circle--;
    }
}
/// @brief 用于调试控温
/// @param  
static void Hand_Temperature_control(Hand_Control_t *hand_control)
{
	if(hand_control->hand_roll_1_motor.hand_motor_measure_all.hand_motor_measure->temperate>=60
		||hand_control->hand_pitch_2_motor.hand_motor_measure_all.hand_motor_measure->temperate>=60
			||hand_control->hand_roll_3_motor.rData->Temp>=60
				||hand_control->hand_x_4_motor.hand_motor_measure_all.hand_motor_measure->temperate>=60
					||hand_control->hand_yaw_5_motor.rData->Temp>=60)
	{
		while(1)
		{
			hand_control->hand_roll_3_motor.GM_Send_Data.GM_Send_Kp_Pos=0;
			hand_control->hand_roll_3_motor.GM_Send_Data.GM_Send_Kd_Speed=0;
			hand_control->hand_roll_3_motor.GM_Send_Data.GM_Send_Effort=0;
			hand_control->hand_yaw_5_motor.GM_Send_Data.GM_Send_Kp_Pos=0;
			hand_control->hand_yaw_5_motor.GM_Send_Data.GM_Send_Kd_Speed=0;
			hand_control->hand_yaw_5_motor.GM_Send_Data.GM_Send_Effort=0;
			for(char i = 0;i<4;i++)
				{
					hand_set_current1_4[i]  = 0;
					hand_set_current5_8[i]  = 0;

				} 
			buzzer_on(95, 10000);
		}

	}
}
static float calculateResult(float inputNumber, float k, float b) {
    return inputNumber * k + b;
}

/**
 * @brief 对电机数据变量赋值
 * @note 赋值的变量用于传递给其它MCU或微机
 */
void Get_Hand_Status(const Hand_Control_t hand_control)//只进行值传递
{
	hand_status.motor_position3 = calculateResult(hand_control.hand_yaw_5_motor.hand_motor_measure_all.relative_angle , kkkk3 , bbbb3);
	hand_status.motor_position4 = calculateResult(hand_control.hand_x_4_motor.hand_motor_measure_all.relative_angle , kkkk4 , bbbb4);
	hand_status.motor_position5 = calculateResult(hand_control.hand_roll_3_motor.hand_motor_measure_all.relative_angle , kkkk5 , bbbb5);
	hand_status.motor_position6 = calculateResult(hand_control.hand_pitch_2_motor.hand_motor_measure_all.relative_angle , kkkk6 , bbbb6);
	hand_status.motor_position7 = calculateResult(hand_control.hand_roll_1_motor.hand_motor_measure_all.relative_angle , kkkk7 , bbbb7);
}

/**
 * @brief 视角舵机控制 舵机角度值的值等于初值 + 操作手调整值 + 自动角度值(kx+b)
 * @param gimbal_control_loop 
 */
static void View_steering_engine_control(Hand_Control_t *gimbal_control_loop )
{
	/*
	X：左负右正  160以下
	Y：上负下正
	*/
	//1400+++
	static int32_t arr_X_auto;       static int32_t arr_X_offect = 0;       static uint16_t arr_x;
	static uint16_t arr_y = 1700;
	
	arr_X_auto = gimbal_control_loop->hand_yaw_5_motor.hand_motor_measure_all.relative_angle * k_yaw + b_yaw;
	if((switch_is_down(gimbal_control_loop->gimbal_rc_ctrl->rc.s[1]) && switch_is_mid(gimbal_control_loop->gimbal_rc_ctrl->rc.s[0]))\
		|| (gimbal_control_loop->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q)\
		|| (gimbal_control_loop->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E)\
	    || (gimbal_control_loop->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_R)\
	)\
	{
		if((gimbal_control_loop->gimbal_rc_ctrl->rc.ch[2] < -10) || (gimbal_control_loop->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q))
		{
			arr_X_offect += Mouse_control_sensitivity_X;/*需要测试*/
		}
		else if((gimbal_control_loop->gimbal_rc_ctrl->rc.ch[2] > 10) || (gimbal_control_loop->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E))
		{
			arr_X_offect -= Mouse_control_sensitivity_X;
		}
		if(gimbal_control_loop->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_R)
		{
			arr_X_offect = 0;
		}

		 if(gimbal_control_loop->gimbal_rc_ctrl->rc.ch[3] < -10)
		 {
		 	arr_y += Mouse_control_sensitivity_Y;
			
		 	if( arr_y >= arr_Y_limit_max )
		 	{ arr_y = arr_Y_limit_max;}
		 }
		 else if(gimbal_control_loop->gimbal_rc_ctrl->rc.ch[3] > 10)
		 {
		 	arr_y -= Mouse_control_sensitivity_Y;
			
		 	if( arr_y <= arr_Y_limit_min)
		 	{ arr_y = arr_Y_limit_min;}
		 }
	}
	
		arr_x = arr_X_chu + arr_X_auto + arr_X_offect;
		if(arr_x >2500){arr_x = 2500;}
		if(arr_x <500){arr_x = 500;}
		//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, arr_x);	//控制占空比
		//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, arr_y);	//控制占空比

}

void Hand_Set_Reverse(void)
{
	#if roll_1_TURN
	  hand_set_current1_4[0] = - hand_control.hand_roll_1_motor.hand_motor_measure_all.given_current;
	#else    
		hand_set_current1_4[0] =   hand_control.hand_roll_1_motor.hand_motor_measure_all.given_current;
	#endif					
	#if pitch_2_TURN
		hand_set_current1_4[1] = - hand_control.hand_pitch_2_motor.hand_motor_measure_all.given_current;
	#else 
		hand_set_current1_4[1] =   hand_control.hand_pitch_2_motor.hand_motor_measure_all.given_current;
	#endif
	  #if    x_4_TURN
		hand_set_current1_4[2] = - hand_control.hand_x_4_motor.hand_motor_measure_all.given_current;
	#else 
		hand_set_current1_4[2] =   hand_control.hand_x_4_motor.hand_motor_measure_all.given_current;
	#endif
}

void Hand_Current_Output(void)
{
  SERVO1_RS485_Send(&hand_control.hand_yaw_5_motor.GM_Send_Data , hand_control.hand_yaw_5_motor.rData);
  SERVO2_RS485_Send(&hand_control.hand_roll_3_motor.GM_Send_Data , hand_control.hand_roll_3_motor.rData);
  CanSendMess(&hfdcan2,SEND_ID201_204,hand_set_current1_4); 
}
