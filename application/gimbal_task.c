#include "gimbal_task.h"
#include "main.h"

#include "math.h"
#include "bsp_buzzer.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "pid.h"
#include "hand_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "queue.h"
#include "arm_math.h" 
#include "cmsis_os.h"
#include "MCU_communicaton_task.h"
#include "hand_task.h"
#include "motor_timer_ctrl.h"
int16_t gimbal_set_current1_4[4]  = {0,0,0,0};
int16_t gimbal_set_current5_8[4]  = {0,0,0,0};

extern float joint_angles[JOINTS][POSITIONS_PER_JOINT][ANGLES_PER_POSITION];
extern FDCAN_HandleTypeDef hfdcan3;
Gimbal_Control_t gimbal_control;		//云台控制所有相关数据

	
//这段代码定义了一个无符号32位整形变量 gimbal_high_water，它的作用是保存云台任务堆栈的最高水位线。INCLUDE_uxTaskGetStackHighWaterMark 是一个宏定义，用于编译时判断是否包含此函数的宏（本函数为 FreeRTOS 内置函数），如果存在该宏定义，则在代码中定义一个 uint32_t 类型的变量 gimbal_high_water 用于保存云台任务堆栈的最高水位线。在调试程序时可以通过查询这个变量的值来了解云台任务运行时的堆栈空间使用情况，以判断是否会出现堆栈溢出等问题。		
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif
		
		
//电机编码值规整 0—8191
#define ECD_Format(ecd)         \
    {                           \
        if ((ecd) > ecd_range)  \
            (ecd) -= ecd_range; \
        else if ((ecd) < 0)     \
            (ecd) += ecd_range; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        PID_clear(&(gimbal_clear)->_3508_RISE1_height_L_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->_3508_RISE1_height_L_motor.gimbal_motor_speed_pid);                    \
		PID_clear(&(gimbal_clear)->_3508_RISE1_height_L_motor.gimbal_motor_relative_speed_pid);     \
		PID_clear(&(gimbal_clear)->_3508_RISE1_height_L_motor.gimbal_motor_defference_angle_pid);                                                                                            \
                                                                                               \
        PID_clear(&(gimbal_clear)->_3508_RISE2_height_R_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->_3508_RISE2_height_R_motor.gimbal_motor_speed_pid);                  \
		PID_clear(&(gimbal_clear)->_3508_RISE2_height_R_motor.gimbal_motor_relative_speed_pid);           \
		                                                                                     \
        PID_clear(&(gimbal_clear)->_3508_LeftRight_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->_3508_LeftRight_motor.gimbal_motor_speed_pid);                  \
		PID_clear(&(gimbal_clear)->_3508_LeftRight_motor.gimbal_motor_relative_speed_pid);                  \
																										\
            																					\
    }																										

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

// 电机反馈与输出使能控制
#define FEEDBACK_ON() (motor_ctrl_feedback_cmd(GIMBAL_MOTOR,MOTOR_CMD_ENABLE));
#define FEEDBACK_OFF() (motor_ctrl_feedback_cmd(GIMBAL_MOTOR,MOTOR_CMD_DISABLE));
#define OUTPUT_ON() (motor_ctrl_output_cmd(GIMBAL_MOTOR,MOTOR_CMD_ENABLE));
#define OUTPUT_OFF() (motor_ctrl_output_cmd(GIMBAL_MOTOR,MOTOR_CMD_DISABLE));

int aaaaaa;

extern TIM_HandleTypeDef htim8;//servo
extern Hand_Control_t	hand_control;

void vTimerStorageCallback(TimerHandle_t xTimerStorage)
{
  //HAL_GPIO_WritePin(GPIOF, Storage_mechanism_Open_Pin, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOF, Storage_mechanism_Close_Pin, GPIO_PIN_SET);
  gimbal_control.Hand_Vacuum_set = Vacuum_Pump_NONE;

}	
		
uint8_t	gimbal_set[4]	;				
static gimbal_status_t gimbal_status;
const gimbal_status_t *get_gimbal_status_point(void)
{
return &gimbal_status;
}
TimerHandle_t xTimerStorage;	

/// @brief 云台主任务
/// @param pvParameters 
void gimbal_task(void const *pvParameters)
{

  gimbal_set[0]=0x04;
  gimbal_set[1]=0x01;
  gimbal_set[2]=0x03;
  gimbal_set[3]=0x04;
  /**********************vTaskDelayUntil()所用参数********************/	
  //	TickType_t PreviousWakeTime;
  //	const TickType_t TimerIncrement =  pdMS_TO_TICKS(gimbal_CONTROL_TIME);
  //	PreviousWakeTime = xTaskGetTickCount();
  /*******************************************************************/	
  	taskENTER_CRITICAL();//用于初始化避免中断
  //test		IO_Read_Init();//初始化高度传感器io口
  //test		IO_Out_Init();//初始化io引脚输出


		//等待IO口任务任务更新io数据
	taskEXIT_CRITICAL();
		vTaskDelay(gimbal_TASK_INIT_TIME);
	//云台初始化
	taskENTER_CRITICAL();//用于初始化避免中断
  FEEDBACK_OFF();
	gimbal_Init(&gimbal_control);
	xTimerStorage = xTimerCreate("TimerStorage", pdMS_TO_TICKS(3000), pdFALSE, (void *)1, vTimerStorageCallback);	
  taskEXIT_CRITICAL();
	do
	{
		gimbal_Set_Mode(&gimbal_control); 	                   //机械臂遥控器设置模式
		vTaskDelay(10);
	}while( gimbal_control.gimbal_behaviour != gimbal_ZERO_FORCE);

  gimbal_position_Init(&gimbal_control); 
	/*判断电机是否都上线*/
//	if(		toe_is_error(TOE_3508_RISE1_height_L_5_ID)			|| toe_is_error(TOE_3508_RISE2_height_R_6_ID)\
//		||	toe_is_error(TOE_3508_LeftRight_7_ID)			|| toe_is_error(TOE_2006_Storage_mechanism_7_ID)\
//		||	toe_is_error(TOE_3508_vacuum_pump_hand_8_ID)			|| toe_is_error(TOE_2006_HAND_pitch_2_ID)\
//		||	toe_is_error(TOE_2006_HAND_roll_3_ID)				|| toe_is_error(TOE_2006_HAND_x_4_ID)\
//		||	toe_is_error(TOE_2006_HAND_yaw_5_ID)	|| toe_is_error(RefereeSystemTOE)\
//		||	toe_is_error(TOE_3508_M1_ID)						|| toe_is_error(TOE_3508_M2_ID)\
//		||	toe_is_error(TOE_3508_M3_ID)						|| toe_is_error(TOE_3508_M4_ID)\
//		||	toe_is_error(TOE_EXTER_ENCODER_pitch_2)				|| toe_is_error(TOE_EXTER_ENCODER_roll_3)\
//		||	toe_is_error(TOE_EXTER_ENCODER_1)					|| toe_is_error(TOE_EXTER_ENCODER_2))
////	{buzzer_on(50,20000);}
//	else buzzer_off();
//    while (toe_is_error(TOE_3508_RISE1_height_L_5_ID)|| toe_is_error(TOE_3508_RISE2_height_R_6_ID)||toe_is_error(TOE_3508_LeftRight_7_ID)|| toe_is_error(TOE_2006_HAND_roll_3_ID)|| toe_is_error(LeftPitchMotorTOE))
//    {
//        vTaskDelay(gimbal_CONTROL_TIME);
//        gimbal_Feedback_Update(&gimbal_control);             //云台数据反馈
//    }						
  FEEDBACK_ON();
	while(1)  
	{	 
		/********************************************************************************************/		 
		gimbal_Set_Mode(&gimbal_control);    				 //云台遥控器设置模式
		gimbal_Set_Control(&gimbal_control);                 //云台遥控器以及键鼠设置控制量
		//gimbal_Feedback_Update();             //云台数据反馈
	  gimbal_Mode_Change_Control_Transit(&gimbal_control); //控制模式切换 控制数据过渡
		gimbal_Set_Position(&gimbal_control);                 //设置云台位置
		//gimbal_Control_loop(&gimbal_control);                //云台控制PID计算				
		Get_Gimbal_Status(gimbal_control);//只进行值传递
		
		aaaaaa++;
//		gimbal_temperature_control(&gimbal_control);
 

//		Steering_engine_init();
		//云台在遥控器掉线状态即relax 状态，can指令为0，不使用current设置为零的方法，是保证遥控器掉线一定使得云台停止
    if (toe_is_error(DBUSTOE))
    {			
      OUTPUT_OFF();
      for(char i = 0;i<4;i++)
      {
        gimbal_set_current1_4[i]  = 0;
      } 
    }
    else
    {
      OUTPUT_ON();
    } 
    //gimbal_current_out();

		vTaskDelay(2);
//      vTaskDelayUntil(&PreviousWakeTime,TimerIncrement);
			
  #if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
	#endif	
    }	
}


/// @brief 云台初始化
/// @param gimbal_init 
static void gimbal_Init(Gimbal_Control_t *gimbal_init)
{	
	/***************************************************速度单环pid初始化**********************************************************/
		static const fp32 _3508_RISE1_height_L_speed_pid[3] 	= {_3508_RISE1_height_L_SPEED_PID_KP		, _3508_RISE1_height_L_SPEED_PID_KI		, _3508_RISE1_height_L_SPEED_PID_KD};
		static const fp32 _3508_RISE2_height_R_speed_pid[3] 	= {_3508_RISE2_height_R_SPEED_PID_KP		, _3508_RISE2_height_R_SPEED_PID_KI		, _3508_RISE2_height_R_SPEED_PID_KD};
		static const fp32 _3508_LeftRight_speed_pid[3] 	        = {_3508_LeftRight_SPEED_PID_KP		        , _3508_LeftRight_SPEED_PID_KI		    , _3508_LeftRight_SPEED_PID_KD};

		PID_Init(&gimbal_init->_3508_RISE1_height_L_motor.gimbal_motor_speed_pid	, PID_POSITION, _3508_RISE1_height_L_speed_pid     	, _3508_RISE1_height_L_SPEED_PID_MAX_OUT		, _3508_RISE1_height_L_SPEED_PID_MAX_IOUT,0,0);
		PID_Init(&gimbal_init->_3508_RISE2_height_R_motor.gimbal_motor_speed_pid	, PID_POSITION, _3508_RISE2_height_R_speed_pid	    , _3508_RISE2_height_R_SPEED_PID_MAX_OUT		, _3508_RISE2_height_R_SPEED_PID_MAX_IOUT,0,0);
		PID_Init(&gimbal_init->_3508_LeftRight_motor.gimbal_motor_speed_pid	        , PID_POSITION, _3508_LeftRight_speed_pid	        , _3508_LeftRight_SPEED_PID_MAX_OUT				, _3508_LeftRight_SPEED_PID_MAX_IOUT,0,0);

		/***********************************************位置单环pid初始化***********************************************************/
	    static const fp32 _height_difference_relative_speed_pid[3] 		= {_height_differeence_balance_angle_kP		, _height_differeence_balance_angle_kI		, _height_differeence_balance_angle_kD};
		PID_Init(&gimbal_init->_3508_RISE1_height_L_motor.gimbal_motor_defference_angle_pid	, PID_POSITION, _height_difference_relative_speed_pid	, _height_differeence_balance_angle_MAX_OUT		, _height_differeence_balance_angle_MAX_IOUT,0,0);

		/***************************************************双环速度pid初始化**********************************************************/
		static const fp32 _3508_RISE1_height_L_relative_speed_pid[3] 		= {_3508_RISE1_height_L_relative_speed_PID_KP		, _3508_RISE1_height_L_relative_speed_PID_KI		, _3508_RISE1_height_L_relative_speed_PID_KD};
		static const fp32 _3508_RISE2_height_R_relative_speed_pid[3] 		= {_3508_RISE2_height_R_relative_speed_PID_KP		, _3508_RISE2_height_R_relative_speed_PID_KI		, _3508_RISE2_height_R_relative_speed_PID_KD};
		static const fp32 _3508_LeftRight_relative_speed_pid[3] 		    = {_3508_LeftRight_relative_speed_PID_KP		    , _3508_LeftRight_relative_speed_PID_KI	         	, _3508_LeftRight_relative_speed_PID_KD};

		PID_Init(&gimbal_init->_3508_RISE1_height_L_motor.gimbal_motor_relative_speed_pid	, PID_POSITION, _3508_RISE1_height_L_relative_speed_pid	, _3508_RISE1_height_L_relative_speed_PID_MAX_OUT		, _3508_RISE1_height_L_relative_speed_PID_MAX_IOUT,0,0);
		PID_Init(&gimbal_init->_3508_RISE2_height_R_motor.gimbal_motor_relative_speed_pid	, PID_POSITION, _3508_RISE2_height_R_relative_speed_pid	, _3508_RISE2_height_R_relative_speed_PID_MAX_OUT		, _3508_RISE2_height_R_relative_speed_PID_MAX_IOUT,0,0);
		PID_Init(&gimbal_init->_3508_LeftRight_motor.gimbal_motor_relative_speed_pid	    , PID_POSITION, _3508_LeftRight_relative_speed_pid	    , _3508_LeftRight_relative_speed_PID_MAX_OUT	        	, _3508_LeftRight_relative_speed_PID_MAX_IOUT,0,0);
		/***************************************************双环角度pid初始化**********************************************************/
		static const fp32 _3508_RISE1_height_L_relative_angle_pid[3] 		= {_3508_RISE1_height_L_relative_angle_PID_KP		, _3508_RISE1_height_L_relative_angle_PID_KI		, _3508_RISE1_height_L_relative_angle_PID_KD};
		static const fp32 _3508_RISE2_height_R_relative_angle_pid[3] 		= {_3508_RISE2_height_R_relative_angle_PID_KP		, _3508_RISE2_height_R_relative_angle_PID_KI		, _3508_RISE2_height_R_relative_angle_PID_KD};
		static const fp32 _3508_LeftRight_relative_angle_pid[3] 		    = { _3508_LeftRight_relative_angle_PID_KP           , _3508_LeftRight_relative_angle_PID_KI		        , _3508_LeftRight_relative_angle_PID_KD};

		PID_Init(&gimbal_init->_3508_RISE1_height_L_motor.gimbal_motor_relative_angle_pid	, PID_POSITION, _3508_RISE1_height_L_relative_angle_pid	   , _3508_RISE1_height_L_relative_angle_PID_MAX_OUT		, _3508_RISE1_height_L_relative_angle_PID_MAX_IOUT,0,0);
		PID_Init(&gimbal_init->_3508_RISE2_height_R_motor.gimbal_motor_relative_angle_pid	, PID_POSITION, _3508_RISE2_height_R_relative_angle_pid	   , _3508_RISE2_height_R_relative_angle_PID_MAX_OUT		, _3508_RISE2_height_R_relative_angle_PID_MAX_IOUT,0,0);
		PID_Init(&gimbal_init->_3508_LeftRight_motor.gimbal_motor_relative_angle_pid	    , PID_POSITION, _3508_LeftRight_relative_angle_pid	       , _3508_LeftRight_relative_angle_PID_MAX_OUT		        , _3508_LeftRight_relative_angle_PID_MAX_IOUT,0,0);


		//清除pid数据
		gimbal_total_pid_clear(gimbal_init);
		
		gimbal_init->_3508_RISE1_height_L_motor.gimbal_motor_measure	=	get_3508_RISE1_height_L_Measure_Point();
		gimbal_init->_3508_RISE1_height_L_motor.gimbal_encoder_measure	=	get_gimbal_encoder_height_L_Point();

		gimbal_init->_3508_RISE2_height_R_motor.gimbal_motor_measure	=	get_3508_RISE2_height_R_Measure_Point();
		gimbal_init->_3508_RISE2_height_R_motor.gimbal_encoder_measure	=	get_gimbal_encoder_height_R_Point();

		gimbal_init->_3508_LeftRight_motor.gimbal_motor_measure	        =	get_3508_LeftRight_Measure_Point();
		gimbal_init->_3508_LeftRight_motor.gimbal_encoder_measure	    =	get_gimbal_encoder_leftRight_Point();		




		gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
		gimbal_init->hand_custom_control = (elf_measure_t *)get_self_measure_point();

		//初始化电机中值，模仿步兵云台手动设定中值位置

		gimbal_init->_3508_RISE1_height_L_motor.offset_ecd 	= _3508_RISE1_offset;
		gimbal_init->_3508_RISE2_height_R_motor.offset_ecd 	= _3508_RISE2_offset;
		gimbal_init->_3508_LeftRight_motor.offset_ecd 	    = _3508_LeftRight_offset;


		gimbal_init->gimbal_position                        = gimbal_position_N;
		gimbal_init->gimbal_behaviour                       = gimbal_INIT;
		gimbal_init->gimbal_behaviour_last                  = gimbal_INIT;

		gimbal_init->Gimbal_Storage_mechanism_ANGLE         = Storage_mechanism_NONE;
		gimbal_init->Gimbal_Storage_mechanism_ANGLE_last 	= Storage_mechanism_NONE;
		gimbal_init->Gimbal_Storage_mechanism_ANGLE_set 	= Storage_mechanism_NONE;

		gimbal_init->Hand_Vacuum_set                        = Vacuum_Pump_NONE;

		gimbal_Feedback_Update();
		
    /*气泵*/
    //gimbal_init->IO_value.Storage_mechanism_Open_Value  =   !HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_6);
    //gimbal_init->IO_value.Storage_mechanism_Close_Value =   !HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_7);

		/*******************************************************************/
		//记录高度自锁编码值
		gimbal_init->_3508_RISE1_height_L_motor.relative_angle_set     = gimbal_init->_3508_RISE1_height_L_motor.lock_angle 	= gimbal_init->_3508_RISE1_height_L_motor.relative_angle;
		gimbal_init->_3508_RISE2_height_R_motor.relative_angle_set     = gimbal_init->_3508_RISE2_height_R_motor.lock_angle 	= gimbal_init->_3508_RISE2_height_R_motor.relative_angle;
		gimbal_init->_3508_LeftRight_motor.relative_angle_set          = gimbal_init->_3508_LeftRight_motor.lock_angle 	        = gimbal_init->_3508_LeftRight_motor.relative_angle;

		gimbal_init->_3508_RISE1_height_L_motor.current_set_difference=0;
		gimbal_init->_3508_RISE2_height_R_motor.current_set_difference=0;
		gimbal_init->_3508_LeftRight_motor.current_set_difference=0;
}
/// @brief 云台遥控器以及键鼠设置模式与控制量
/// @param gimbal_set_mode 
static void gimbal_Set_Mode(Gimbal_Control_t *gimbal_set_mode)
{
	/************************************************升降，滑台***************************************************/
	if(gimbal_set_mode->gimbal_behaviour==gimbal_CUSTIM)
	{
		return;
	}
	if(gimbal_set_mode->gimbal_behaviour==gimbal_INIT)
	{

	}
	if(switch_is_up(gimbal_set_mode->gimbal_rc_ctrl->rc.s[1])&&switch_is_up(gimbal_set_mode->gimbal_rc_ctrl->rc.s[0]))
	{
	gimbal_set_mode->gimbal_behaviour		= gimbal_all_Operation;
	}
	else if(switch_is_down(gimbal_set_mode->gimbal_rc_ctrl->rc.s[1])&&switch_is_down(gimbal_set_mode->gimbal_rc_ctrl->rc.s[0]))
	{
	gimbal_set_mode->gimbal_behaviour		= gimbal_ZERO_FORCE;
	}
	else gimbal_set_mode->gimbal_behaviour  = gimbal_motionless;
	/***************************************************储矿******************************************************/
 	if(gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE_set == Storage_mechanism_Close)
	{
		if (gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE == Storage_mechanism_Close)
		{
		}
		if (gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE == Storage_mechanism_Open)
		{
		}		
		if (gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE == Storage_mechanism_Running)
		{
		}
	}
 	else if(gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE_set == Storage_mechanism_Open)
	{
		if (gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE == Storage_mechanism_Close)
		{
		}
		if (gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE == Storage_mechanism_Open)
		{
		}		
		if (gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE == Storage_mechanism_Running)
		{
		}
	}
/*************************************************吸矿*************************************************************/
//	if(gimbal_set_mode->Hand_Vacuum_set == Vacuum_Pump_Open)
//	{
//		HAL_GPIO_WritePin(GPIOF, Storage_mechanism_Open_Pin, GPIO_PIN_RESET);
//		gimbal_set_mode->Hand_Vacuum_set = Vacuum_Pump_Runing;
//
//	}
//	else if (gimbal_set_mode->Hand_Vacuum_set == Vacuum_Pump_Close)
//	{
//		HAL_GPIO_WritePin(GPIOF, Storage_mechanism_Open_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOF, Storage_mechanism_Close_Pin, GPIO_PIN_RESET);
//
//		
//		if (xTimerStorage != NULL)
//		{
//			// 启动定时器
//			if (xTimerStart(xTimerStorage, 0) == pdPASS)
//			{
//				// 定时器成功启动
//			    gimbal_set_mode->Hand_Vacuum_set = Vacuum_Pump_Runing;
//				
//
//			}
//		}
//
//	}
}
static void gimbal_position_Init(Gimbal_Control_t *gimbal_init)
{

	uint8_t gimbal_liftright_init = 0;
	
	float gimbal_liftright_ecd;


	int16_t gimbal_current_init1[4]={0,0,-4000,0};
	int16_t gimbal_current_init0[4]={0,0,0,0};
	do
		{
			gimbal_Feedback_Update();
			CanSendMess(&hfdcan3,SEND_ID201_204,gimbal_current_init1);	
			vTaskDelay(1);
		}
  while(gimbal_init->_3508_LeftRight_motor.gimbal_motor_measure->given_current == 0);
		vTaskDelay(50);

		while(1)
		{			
			//这里不用can的last_ecd是因为要通过限制频率来使last_ecd相对变大
			if (fabs(gimbal_liftright_ecd - gimbal_init->_3508_LeftRight_motor.gimbal_motor_measure->ecd)<5)
			{
				CanSendMess(&hfdcan3,SEND_ID201_204,gimbal_current_init0);
				gimbal_liftright_init = 1;
			}
			gimbal_Feedback_Update();
			

			gimbal_liftright_ecd   = gimbal_init->_3508_LeftRight_motor.gimbal_motor_measure->ecd;

		
			if(gimbal_liftright_init)
			{
				break;
			}
			vTaskDelay(10);
		}
	
}
/// @brief 云台遥控器以及键鼠设置控制量
/// @param gimbal_set_control 
static void gimbal_Set_Control(Gimbal_Control_t *gimbal_set_control)
{
    if (gimbal_set_control == NULL)
    {
        return;
    }
	static fp32 	rc_add_channel_0, rc_add_channel_1,rc_add_channel_2,rc_add_channel_3,rc_add_channel_4;
    static int16_t 	channel_0 = 0,channel_1 = 0,channel_2 = 0,channel_3 = 0,channel_4 = 0;
	
	rc_deadline_limit(gimbal_set_control->gimbal_rc_ctrl->rc.ch[Channel_0], channel_0, RC_deadband);
	rc_deadline_limit(gimbal_set_control->gimbal_rc_ctrl->rc.ch[Channel_1], channel_1, RC_deadband);
	rc_deadline_limit(gimbal_set_control->gimbal_rc_ctrl->rc.ch[Channel_2], channel_2, RC_deadband);
	rc_deadline_limit(gimbal_set_control->gimbal_rc_ctrl->rc.ch[Channel_3], channel_3, RC_deadband);
	rc_deadline_limit(gimbal_set_control->gimbal_rc_ctrl->rc.ch[Channel_4], channel_4, RC_deadband);

	/************************************************升降，滑台***************************************************/
	if(gimbal_set_control->gimbal_behaviour==gimbal_all_Operation)
	{
		rc_add_channel_0 = 	 channel_0 * gimbal_1_channel_RC_SEN 	;
		rc_add_channel_1 =   channel_1 * gimbal_2_channel_RC_SEN 	;
		
		rc_add_channel_2 = 	 channel_2 * gimbal_3_channel_RC_SEN 	;
		rc_add_channel_3 =   channel_3 * gimbal_4_channel_RC_SEN 	;
		
		rc_add_channel_4 = 	 channel_4 * gimbal_5_channel_RC_SEN 	;


		gimbal_relative_angle_limit(&gimbal_set_control->_3508_RISE1_height_L_motor  ,/**/rc_add_channel_3	,_3508_RISE1_relative_angle_set_max	    ,_3508_RISE1_relative_angle_set_min);
		gimbal_relative_angle_limit(&gimbal_set_control->_3508_RISE2_height_R_motor  ,/**/rc_add_channel_3	,_3508_RISE2_relative_angle_set_max	    ,_3508_RISE2_relative_angle_set_min);		
		gimbal_relative_angle_limit(&gimbal_set_control->_3508_LeftRight_motor	      ,/**/rc_add_channel_2	,_3508_LeftRight_relative_angle_set_max	,_3508_LeftRight_relative_angle_set_min);
	
	}	
	/************************************************存矿***************************************************/
	if(switch_is_up(gimbal_set_control->gimbal_rc_ctrl->rc.s[1])&&switch_is_up(gimbal_set_control->gimbal_rc_ctrl->rc.s[0]))
	{
		if(channel_0 >= 600)
			gimbal_set_control->Gimbal_Storage_mechanism_ANGLE_set = Storage_mechanism_Open;
		else if(channel_0 <= -600)
			gimbal_set_control->Gimbal_Storage_mechanism_ANGLE_set = Storage_mechanism_Close;		
	}
	/************************************************取矿***************************************************/
	if(switch_is_up(gimbal_set_control->gimbal_rc_ctrl->rc.s[1])&&switch_is_up(gimbal_set_control->gimbal_rc_ctrl->rc.s[0]))
	{
		if(channel_1 >= 600)
			gimbal_set_control->Hand_Vacuum_set = Vacuum_Pump_Open;
		else if(channel_1 <= -600)
			gimbal_set_control->Hand_Vacuum_set = Vacuum_Pump_Close;		
	}
	
	/************************************************图传***************************************************/	
	switch(gimbal_set_control->gimbal_rc_ctrl->key.v) //660是遥控器最大输出，方便量化
	{
		case KEY_PRESSED_OFFSET_G ://一级升降向上动
		gimbal_relative_angle_limit(&gimbal_set_control->_3508_RISE1_height_L_motor  , 660*gimbal_4_channel_RC_SEN	,_3508_RISE1_relative_angle_set_max	    ,_3508_RISE1_relative_angle_set_min);
		gimbal_relative_angle_limit(&gimbal_set_control->_3508_RISE2_height_R_motor  , 660*gimbal_4_channel_RC_SEN	,_3508_RISE2_relative_angle_set_max	    ,_3508_RISE2_relative_angle_set_min);		
		break;
		case KEY_PRESSED_OFFSET_B ://一级升降向下动
		gimbal_relative_angle_limit(&gimbal_set_control->_3508_RISE1_height_L_motor  , -660*gimbal_4_channel_RC_SEN	,_3508_RISE1_relative_angle_set_max	    ,_3508_RISE1_relative_angle_set_min);
		gimbal_relative_angle_limit(&gimbal_set_control->_3508_RISE2_height_R_motor  , -660*gimbal_4_channel_RC_SEN	,_3508_RISE2_relative_angle_set_max	    ,_3508_RISE2_relative_angle_set_min);		
		break;
		case KEY_PRESSED_OFFSET_C ://滑台左
		gimbal_relative_angle_limit(&gimbal_set_control->_3508_LeftRight_motor	      , 660*gimbal_3_channel_RC_SEN	,_3508_LeftRight_relative_angle_set_max	,_3508_LeftRight_relative_angle_set_min);	
		break;
		case KEY_PRESSED_OFFSET_V ://滑台左
		gimbal_relative_angle_limit(&gimbal_set_control->_3508_LeftRight_motor	      , -660*gimbal_3_channel_RC_SEN	,_3508_LeftRight_relative_angle_set_max	,_3508_LeftRight_relative_angle_set_min);	
		break;
	}
	
}
/// @brief 云台相对角度限制
/// @param gimbal_Motor 
/// @param add 
/// @param max_limit 
/// @param min_limit 
static void gimbal_relative_angle_limit(Gimbal_Motor_t *gimbal_Motor,fp32 add,fp32 max_limit,fp32 min_limit)
{
	if (gimbal_Motor == NULL)
    {
        return;
    }
    gimbal_Motor->relative_angle_set += add;
	if(max_limit==min_limit&&min_limit==0.0f)
	{
		return ;
	}
	else if(gimbal_Motor->relative_angle_set - add <= max_limit && gimbal_Motor->relative_angle_set - add >= min_limit )
	{
    //是否超过最大 最小
		if (gimbal_Motor->relative_angle_set > max_limit)
		{
			gimbal_Motor->relative_angle_set = max_limit;
		}
		else if (gimbal_Motor->relative_angle_set < min_limit)
		{
			gimbal_Motor->relative_angle_set = min_limit;
		}
	}
	else if(gimbal_Motor->relative_angle_set - add >= max_limit)
	{
		if(add >0)  gimbal_Motor->relative_angle_set -= add;
	}
	else if(gimbal_Motor->relative_angle_set - add <= min_limit)
	{
		if(add <0)  gimbal_Motor->relative_angle_set -= add;
	}
}


/// @brief 云台数据反馈
/// @param gimbal_feedback_update 
void gimbal_Feedback_Update(void)
{
  Gimbal_Control_t *gimbal_feedback_update=&gimbal_control;

  if (gimbal_feedback_update == NULL)
  {
      return;
  }

	gimbal_feedback_update->_3508_RISE1_height_L_motor.encoder_angle	=	motor_ecd_to_angle_change(gimbal_feedback_update->_3508_RISE1_height_L_motor.gimbal_motor_measure->ecd,4096);
	gimbal_feedback_update->_3508_RISE1_height_L_motor.relative_angle   =   encoder_ecd_to_relative_height(gimbal_feedback_update->_3508_RISE1_height_L_motor.gimbal_encoder_measure->ecd , height_L_encoder_offset_ecd , height_L_encoder_offset_height);


	gimbal_feedback_update->_3508_RISE2_height_R_motor.encoder_angle	=	motor_ecd_to_angle_change(gimbal_feedback_update->_3508_RISE2_height_R_motor.gimbal_motor_measure->ecd,4096);
	gimbal_feedback_update->_3508_RISE2_height_R_motor.relative_angle   =   encoder_ecd_to_relative_height(gimbal_feedback_update->_3508_RISE2_height_R_motor.gimbal_encoder_measure->ecd , height_R_encoder_offset_ecd , height_R_encoder_offset_height);
	
	gimbal_feedback_update->_3508_LeftRight_motor.encoder_angle	        =	motor_ecd_to_angle_change(gimbal_feedback_update->_3508_LeftRight_motor.gimbal_motor_measure->ecd,4096);
	gimbal_feedback_update->_3508_LeftRight_motor.relative_angle        = 	encoder_ecd_to_relative_distance(gimbal_feedback_update->_3508_LeftRight_motor.gimbal_encoder_measure->ecd , heightright_encoder_offset_ecd , heightright_encoder_offset_height);																	  
	
																						     
  //gimbal_feedback_update->IO_value.Storage_mechanism_Open_Value       =   !HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_6);
  //gimbal_feedback_update->IO_value.Storage_mechanism_Close_Value      =   !HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_7);


	if (gimbal_feedback_update->IO_value.Storage_mechanism_Open_Value)
	{
		gimbal_feedback_update->Gimbal_Storage_mechanism_ANGLE                        	 =      Storage_mechanism_Open;
	}
	if (gimbal_feedback_update->IO_value.Storage_mechanism_Close_Value)	
	{
		gimbal_feedback_update->Gimbal_Storage_mechanism_ANGLE                         	 =      Storage_mechanism_Close;
	}
	if (gimbal_feedback_update->IO_value.Storage_mechanism_Open_Value== 0 && gimbal_feedback_update->IO_value.Storage_mechanism_Close_Value== 0)
	{
		gimbal_feedback_update->Gimbal_Storage_mechanism_ANGLE                         	 =      Storage_mechanism_Running;
	}

}


/// @brief RM电机角度值转换为相对角度值
/// @param ecd 
/// @param offset_ecd 
/// @return 
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd; //计算编码器当前值与起始时刻的差值，并将其保存到 relative_ecd 变量中
    if (relative_ecd > Half_ecd_range)
    {
        relative_ecd -= ecd_range;
    }
    else if (relative_ecd < -Half_ecd_range)
    {
        relative_ecd += ecd_range;
    }

    return relative_ecd * Motor_Ecd_to_Rad;
}

static fp32 encoder_ecd_to_relative_height(uint16_t ecd, int32_t offset_ecd, float offect_height)
{

	int32_t relative_ecd = ecd + offset_ecd;
	float relative_height = relative_ecd * Motor_Ecd_to_HEIGHT + offect_height; 
	return relative_height;
	
	
}
static fp32 encoder_ecd_to_relative_distance(uint16_t ecd, int32_t offset_ecd, float offect_height)
{

	int32_t relative_ecd = ecd + offset_ecd;
	float relative_height = relative_ecd * Motor_Ecd_to_DISTANCE + offect_height; 
	return relative_height;
	
	
}
/// @brief 控制模式切换 控制数据过渡
/// @param gimbal_mode_change 
static void gimbal_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change)
{
	
if (gimbal_mode_change->gimbal_behaviour_last != gimbal_ZERO_FORCE && gimbal_mode_change-> gimbal_behaviour == gimbal_ZERO_FORCE)
    {	

		gimbal_mode_change->_3508_RISE1_height_L_motor.raw_cmd_current		   =gimbal_mode_change->_3508_RISE1_height_L_motor.current_set						    =gimbal_mode_change->_3508_RISE1_height_L_motor.given_current;
		gimbal_mode_change->_3508_RISE2_height_R_motor.raw_cmd_current		   =gimbal_mode_change->_3508_RISE2_height_R_motor.current_set						    =gimbal_mode_change->_3508_RISE2_height_R_motor.given_current;
		gimbal_mode_change->_3508_LeftRight_motor.raw_cmd_current			   =gimbal_mode_change->_3508_LeftRight_motor.current_set								=gimbal_mode_change->_3508_LeftRight_motor.given_current;

		gimbal_mode_change->gimbal_behaviour_last = gimbal_mode_change->gimbal_behaviour;
	}
	else if (gimbal_mode_change->gimbal_behaviour_last != gimbal_all_Operation && gimbal_mode_change-> gimbal_behaviour == gimbal_all_Operation) 
	{
		gimbal_mode_change->_3508_RISE1_height_L_motor.relative_angle_set     = gimbal_mode_change->_3508_RISE1_height_L_motor.relative_angle;
		gimbal_mode_change->_3508_RISE2_height_R_motor.relative_angle_set     = gimbal_mode_change->_3508_RISE2_height_R_motor.relative_angle;
		gimbal_mode_change->_3508_LeftRight_motor.relative_angle_set          = gimbal_mode_change->_3508_LeftRight_motor.relative_angle;

		gimbal_mode_change->_3508_RISE1_height_L_motor.lock_angle             = gimbal_mode_change->_3508_RISE1_height_L_motor.relative_angle;
		gimbal_mode_change->_3508_RISE2_height_R_motor.lock_angle     	      = gimbal_mode_change->_3508_RISE2_height_R_motor.relative_angle;
		gimbal_mode_change->_3508_LeftRight_motor.lock_angle         		  = gimbal_mode_change->_3508_LeftRight_motor.relative_angle;

		gimbal_mode_change->gimbal_behaviour_last = gimbal_mode_change->gimbal_behaviour;
	}
	else if (gimbal_mode_change->gimbal_behaviour_last != gimbal_motionless && gimbal_mode_change-> gimbal_behaviour == gimbal_motionless) 
	{
		gimbal_mode_change->_3508_RISE1_height_L_motor.relative_angle_set     = gimbal_mode_change->_3508_RISE1_height_L_motor.relative_angle;
		gimbal_mode_change->_3508_RISE2_height_R_motor.relative_angle_set     = gimbal_mode_change->_3508_RISE2_height_R_motor.relative_angle;
		gimbal_mode_change->_3508_LeftRight_motor.relative_angle_set          = gimbal_mode_change->_3508_LeftRight_motor.relative_angle;

		gimbal_mode_change->_3508_RISE1_height_L_motor.lock_angle             = gimbal_mode_change->_3508_RISE1_height_L_motor.relative_angle;
		gimbal_mode_change->_3508_RISE2_height_R_motor.lock_angle     	      = gimbal_mode_change->_3508_RISE2_height_R_motor.relative_angle;
		gimbal_mode_change->_3508_LeftRight_motor.lock_angle         		  = gimbal_mode_change->_3508_LeftRight_motor.relative_angle;

		gimbal_mode_change->gimbal_behaviour_last = gimbal_mode_change->gimbal_behaviour;
	}

	if((gimbal_mode_change->Gimbal_Storage_mechanism_ANGLE_last != Storage_mechanism_Close) && gimbal_mode_change->Gimbal_Storage_mechanism_ANGLE == Storage_mechanism_Close)
	{
		gimbal_mode_change->Gimbal_Storage_mechanism_ANGLE_last = gimbal_mode_change->Gimbal_Storage_mechanism_ANGLE;
	}
	else if((gimbal_mode_change->Gimbal_Storage_mechanism_ANGLE_last != Storage_mechanism_Open) && gimbal_mode_change->Gimbal_Storage_mechanism_ANGLE == Storage_mechanism_Open)
	{
		gimbal_mode_change->Gimbal_Storage_mechanism_ANGLE_last = gimbal_mode_change->Gimbal_Storage_mechanism_ANGLE;
	}
	
	//gimbal_mode_change->gimbal_behaviour_last = gimbal_mode_change-> gimbal_behaviour;
}

/// @brief 设置云台位置
/// @param gimbal_set_control 
static void gimbal_Set_Position(Gimbal_Control_t *gimbal_set_position)
{
	if (gimbal_set_position == NULL)
    {
        return;
    }

	if (gimbal_set_position->gimbal_rc_ctrl->mouse.press_l)//下一步操作
	{
		gimbal_set_position->gimbal_behaviour = gimbal_CUSTIM;
		gimbal_set_position->gimbal_point += 1 ;
	}
	else if (gimbal_set_position->gimbal_rc_ctrl->mouse.press_r)//上一步操作
	{
		gimbal_set_position->gimbal_behaviour = gimbal_CUSTIM;
		if (gimbal_set_position->gimbal_behaviour != gimbal_point_1)
		{
			gimbal_set_position->gimbal_point -= 1 ;
		}		
	}

	switch(gimbal_set_position->gimbal_rc_ctrl->key.v)    
	{	
		case KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_Z ://自定义控制器模式
			Gimbal_Custom_Set(gimbal_set_position);
			gimbal_set_position->gimbal_behaviour = gimbal_CUSTIM;
			gimbal_set_position->gimbal_point = gimbal_point_1;
			gimbal_set_position->gimbal_position = gimbal_position_N;

			break;
		case KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_B ://切回传统控制模式(会中断自定义控制器，视觉，一键运动)
			gimbal_set_position->gimbal_behaviour = gimbal_all_Operation;
			gimbal_set_position->gimbal_point = gimbal_point_1;
			gimbal_set_position->gimbal_position = gimbal_position_N;
			break;

		case KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_Z : 
			gimbal_set_position->gimbal_position  = gimbal_position_1;
			gimbal_set_position->gimbal_point = gimbal_point_1;
			gimbal_set_position->gimbal_behaviour = gimbal_CUSTIM;
			break;
		case KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_X : 
			gimbal_set_position->gimbal_position  = gimbal_position_2;
			gimbal_set_position->gimbal_point = gimbal_point_1;
			gimbal_set_position->gimbal_behaviour = gimbal_CUSTIM;
			break;
		case KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_C : 
			gimbal_set_position->gimbal_position  = gimbal_position_3;		
			gimbal_set_position->gimbal_point = gimbal_point_1;	
			gimbal_set_position->gimbal_behaviour = gimbal_CUSTIM;
			break;
		case KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_V : 
			gimbal_set_position->gimbal_position  = gimbal_position_4;
			gimbal_set_position->gimbal_point = gimbal_point_1;
			gimbal_set_position->gimbal_behaviour = gimbal_CUSTIM;
			break;
		case KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_B: 
			break;
		case KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_G: 
			__set_FAULTMASK(1); //关闭所有中断
			NVIC_SystemReset(); //进行软件复位
			break;	
	}
	
	if(gimbal_set_position->gimbal_behaviour==gimbal_CUSTIM)
	{
		if(gimbal_set_position->gimbal_position == gimbal_position_N)      
		{
			//Gimbal_Custom_Set(gimbal_set_position);		
		}
		else if(gimbal_set_position->gimbal_position  == gimbal_position_1)
		{
			GIMBAL_combination_control(gimbal_set_position,joint_angles[6][gimbal_set_position->gimbal_position][gimbal_set_position->gimbal_point],joint_angles[6][gimbal_set_position->gimbal_position][gimbal_set_position->gimbal_point] ,joint_angles[7][gimbal_set_position->gimbal_position][gimbal_set_position->gimbal_point]  );
		}
		else if(gimbal_set_position->gimbal_position  == gimbal_position_2)
		{
			GIMBAL_combination_control(gimbal_set_position,joint_angles[6][gimbal_set_position->gimbal_position][gimbal_set_position->gimbal_point],joint_angles[6][gimbal_set_position->gimbal_position][gimbal_set_position->gimbal_point] ,joint_angles[7][gimbal_set_position->gimbal_position][gimbal_set_position->gimbal_point] );
		}
		else if(gimbal_set_position->gimbal_position  == gimbal_position_3)
		{
			GIMBAL_combination_control(gimbal_set_position,joint_angles[6][gimbal_set_position->gimbal_position][gimbal_set_position->gimbal_point],joint_angles[7][gimbal_set_position->gimbal_position][gimbal_set_position->gimbal_point] ,joint_angles[7][gimbal_set_position->gimbal_position][gimbal_set_position->gimbal_point] );
		}
		else if(gimbal_set_position->gimbal_position  == gimbal_position_4)
		{
			GIMBAL_combination_control(gimbal_set_position,joint_angles[6][gimbal_set_position->gimbal_position][gimbal_set_position->gimbal_point],joint_angles[7][gimbal_set_position->gimbal_position][gimbal_set_position->gimbal_point] ,joint_angles[7][gimbal_set_position->gimbal_position][gimbal_set_position->gimbal_point] );
		}
		else if(gimbal_set_position->gimbal_position  == gimbal_position_5)
		{
			GIMBAL_combination_control(gimbal_set_position,joint_angles[6][gimbal_set_position->gimbal_position][gimbal_set_position->gimbal_point],joint_angles[7][gimbal_set_position->gimbal_position][gimbal_set_position->gimbal_point] ,joint_angles[7][gimbal_set_position->gimbal_position][gimbal_set_position->gimbal_point] );
		}
	}
}

static void Gimbal_Custom_Set(Gimbal_Control_t *gimbal_set_position)
{
	if (gimbal_set_position == NULL)
    {
        return;
    }
	
	gimbal_relative_angle_limit(&gimbal_set_position->_3508_RISE1_height_L_motor	,/**/gimbal_set_position->hand_custom_control->motor_position6    ,_3508_RISE1_relative_angle_set_max		 ,_3508_RISE1_relative_angle_set_min);
	gimbal_relative_angle_limit(&gimbal_set_position->_3508_RISE2_height_R_motor	,/**/gimbal_set_position->hand_custom_control->motor_position6    ,_3508_RISE2_relative_angle_set_max	     ,_3508_RISE2_relative_angle_set_min);
	gimbal_relative_angle_limit(&gimbal_set_position->_3508_LeftRight_motor	        ,/**/gimbal_set_position->hand_custom_control->motor_position5    ,_3508_LeftRight_relative_angle_set_max    ,_3508_LeftRight_relative_angle_set_min);
}

static void GIMBAL_combination_control(Gimbal_Control_t *gimbal_combination_control,fp32 relative_angle_set_5,fp32 relative_angle_set_4,fp32 relative_angle_set_3)
{

	static fp32 add_1,	add_2,	add_3;
	
		if(fabs(gimbal_combination_control->_3508_RISE1_height_L_motor.relative_angle-relative_angle_set_5)>HAND_ANGLE_ERROR6)
		{
			if((relative_angle_set_5-gimbal_combination_control->_3508_RISE1_height_L_motor.relative_angle)>=0.0f)
			{
			add_1 = Keyboard_set_Angle_increment_hand_6;
			}
			else add_1 = -Keyboard_set_Angle_increment_hand_5;
			
			gimbal_relative_angle_limit(&gimbal_combination_control->_3508_RISE1_height_L_motor , add_1 ,_3508_RISE1_relative_angle_set_max	,_3508_RISE1_relative_angle_set_min);
			
		}
		
		if(fabs(gimbal_combination_control->_3508_RISE2_height_R_motor.relative_angle-relative_angle_set_4)>HAND_ANGLE_ERROR6)
		{
			if((relative_angle_set_4-gimbal_combination_control->_3508_RISE2_height_R_motor.relative_angle)>=0.0f)
			{
			add_2 = Keyboard_set_Angle_increment_hand_6;
			}
			else add_2 = -Keyboard_set_Angle_increment_hand_6;
			
			gimbal_relative_angle_limit(&gimbal_combination_control->_3508_RISE2_height_R_motor	,add_2,_3508_RISE2_relative_angle_set_max	,_3508_RISE2_relative_angle_set_min);
		}
		
		if(fabs(gimbal_combination_control->_3508_LeftRight_motor.relative_angle-relative_angle_set_3)>HAND_ANGLE_ERROR7)
		{
			if((relative_angle_set_3-gimbal_combination_control->_3508_LeftRight_motor.relative_angle)>=0.0f)
			{
			add_3 = Keyboard_set_Angle_increment_hand_7;
			}
			else add_3 = -Keyboard_set_Angle_increment_hand_7;
			
			gimbal_relative_angle_limit(&gimbal_combination_control->_3508_LeftRight_motor	,add_3	,_3508_LeftRight_relative_angle_set_max	,_3508_LeftRight_relative_angle_set_min);
		}
		
		if((fabs(gimbal_combination_control->_3508_RISE1_height_L_motor.relative_angle-relative_angle_set_5)<=HAND_ANGLE_ERROR6)\
			&&(fabs(gimbal_combination_control->_3508_RISE2_height_R_motor.relative_angle-relative_angle_set_4)<=HAND_ANGLE_ERROR6)\
				&&(fabs(gimbal_combination_control->_3508_LeftRight_motor.relative_angle-relative_angle_set_3)<=HAND_ANGLE_ERROR7))
		{
			gimbal_combination_control->_3508_RISE1_height_L_motor.lock_angle	            =gimbal_combination_control->_3508_RISE1_height_L_motor.relative_angle;
			gimbal_combination_control->_3508_RISE2_height_R_motor.lock_angle	            =gimbal_combination_control->_3508_RISE2_height_R_motor.relative_angle;
			gimbal_combination_control->_3508_LeftRight_motor.lock_angle				    =gimbal_combination_control->_3508_LeftRight_motor.relative_angle;


				gimbal_combination_control->gimbal_behaviour = gimbal_motionless;
		}
}



/// @brief 云台控制PID计算
/// @param gimbal_control_loop 
void gimbal_Control_loop(void)
{
  Gimbal_Control_t *gimbal_control_loop=&gimbal_control;
	if(gimbal_control_loop == NULL)   return; 
/***************************************************升降，滑台************************************************/
	if(gimbal_control_loop->gimbal_behaviour==gimbal_CUSTIM)
	{	
		//gimbal_motor_difference_relative_angle_control(gimbal_control_loop);
		gimbal_motor_relative_angle_control    (&gimbal_control_loop->_3508_RISE1_height_L_motor  , _3508_RISE1_height_L_SPEED_Rate );
		gimbal_motor_relative_angle_control    (&gimbal_control_loop->_3508_RISE2_height_R_motor  , _3508_RISE2_height_R_SPEED_Rate );
		gimbal_motor_relative_angle_control    (&gimbal_control_loop->_3508_LeftRight_motor       , _3508_LeftRight1_height_L_SPEED_Rate );	
	}
    else if(gimbal_control_loop-> gimbal_behaviour == gimbal_ZERO_FORCE)
    {
		gimbal_motor_raw_angle_control          (&gimbal_control_loop->_3508_RISE1_height_L_motor);
		gimbal_motor_raw_angle_control        	(&gimbal_control_loop->_3508_RISE2_height_R_motor);
		gimbal_motor_raw_angle_control			(&gimbal_control_loop->_3508_LeftRight_motor);
    }
	else if (gimbal_control_loop-> gimbal_behaviour == gimbal_motionless)//不用lock_angle是为了方便接入图传，在遥控器其他挡位也可以通过改变relaitive_angle来控制
    {
		//gimbal_motor_difference_relative_angle_control(gimbal_control_loop);
		gimbal_motor_relative_angle_control    (&gimbal_control_loop->_3508_RISE1_height_L_motor  , _3508_RISE1_height_L_SPEED_Rate );
		gimbal_motor_relative_angle_control    (&gimbal_control_loop->_3508_RISE2_height_R_motor  , _3508_RISE2_height_R_SPEED_Rate );
		gimbal_motor_relative_angle_control    (&gimbal_control_loop->_3508_LeftRight_motor       , _3508_LeftRight1_height_L_SPEED_Rate );	
		
    }
	else if(gimbal_control_loop-> gimbal_behaviour == gimbal_all_Operation)	
	{
		//gimbal_motor_difference_relative_angle_control(gimbal_control_loop);
		gimbal_motor_relative_angle_control    (&gimbal_control_loop->_3508_RISE1_height_L_motor  , _3508_RISE1_height_L_SPEED_Rate );
		gimbal_motor_relative_angle_control    (&gimbal_control_loop->_3508_RISE2_height_R_motor  , _3508_RISE2_height_R_SPEED_Rate );
		gimbal_motor_relative_angle_control    (&gimbal_control_loop->_3508_LeftRight_motor       , _3508_LeftRight1_height_L_SPEED_Rate );	
	}
}
/// @brief RM电机相对角度串级pid
/// @param gimbal_motor 
/// @param SPEED_RATE 
static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor,fp32 SPEED_RATE)
{
     if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->motor_position_set =  PID_Calc(&gimbal_motor->gimbal_motor_relative_angle_pid , gimbal_motor->relative_angle, gimbal_motor->relative_angle_set);
	gimbal_motor->current_set        =  PID_Calc(&gimbal_motor->gimbal_motor_relative_speed_pid , gimbal_motor->gimbal_motor_measure->speed_rpm*SPEED_RATE, gimbal_motor->motor_position_set);
    gimbal_motor->given_current      =  (int16_t)(gimbal_motor->current_set);
}
/// @brief RM电机停止
/// @param gimbal_motor 
static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current=0;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
/// @brief RM电机自锁角度串级pid
/// @param gimbal_motor 
/// @param SPEED_RATE 
static void gimbal_motor_lock_angle_angle_control(Gimbal_Motor_t *gimbal_motor,fp32 SPEED_RATE)
{
	if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->motor_position_set  =  PID_Calc(&gimbal_motor->gimbal_motor_relative_angle_pid , gimbal_motor->relative_angle, gimbal_motor->lock_angle);
	gimbal_motor->current_set         = PID_Calc(&gimbal_motor->gimbal_motor_relative_speed_pid, gimbal_motor->gimbal_motor_measure->speed_rpm*SPEED_RATE, gimbal_motor->motor_position_set);
    gimbal_motor->given_current       = (int16_t)(gimbal_motor->current_set)+(int16_t)(gimbal_motor->current_set_difference);
}
/// @brief 升降差速控制
/// @param gimbal_motor 
static void gimbal_motor_difference_relative_angle_control(Gimbal_Control_t *gimbal_motor)
{
	if (gimbal_motor == NULL)
    {
        return;
    }
	gimbal_motor->_3508_RISE1_height_L_motor.current_set_difference   =  PID_Calc(&gimbal_motor->_3508_RISE1_height_L_motor.gimbal_motor_defference_angle_pid ,gimbal_motor->_3508_RISE2_height_R_motor.relative_angle - gimbal_motor->_3508_RISE1_height_L_motor.relative_angle, 0  );
}

    // Gimbal_Motor_t _3508_RISE1_height_L_motor;
    // Gimbal_Motor_t _3508_RISE2_height_R_motor;
	// Gimbal_Motor_t _3508_LeftRight_motor;
	
	// Gimbal_Motor_t _2006_Storage_mechanism;
	// Gimbal_Motor_t _3508_vacuum_pump_hand;

/// @brief 用于调试控温
/// @param  
static void gimbal_temperature_control(Gimbal_Control_t *gimbal_temperature_control)
{
	if (gimbal_temperature_control->_3508_RISE1_height_L_motor.gimbal_motor_measure->temperate>=50
		||gimbal_temperature_control->_3508_RISE1_height_L_motor.gimbal_motor_measure->temperate>=50
			||gimbal_temperature_control->_3508_RISE1_height_L_motor.gimbal_motor_measure->temperate>=50
				||gimbal_temperature_control->_3508_RISE1_height_L_motor.gimbal_motor_measure->temperate>=50
					||gimbal_temperature_control->_3508_RISE1_height_L_motor.gimbal_motor_measure->temperate>=50
					)
	{
		while (1)
		{
			for(char i = 0;i<4;i++)
				{
					gimbal_set_current1_4[i]  = 0;
					gimbal_set_current5_8[i]  = 0;
					

				} 
			buzzer_on(70,10000);
		}	
	}	
	
}

static float calculateResult(float inputNumber, float k, float b) {
    return inputNumber * k + b;
}
void Get_Gimbal_Status(const Gimbal_Control_t gimbal_control)//只进行值传递
{
	gimbal_status.motor_position1 = calculateResult(gimbal_control._3508_RISE1_height_L_motor.relative_angle , kkkk1 , bbbb1);
	gimbal_status.motor_position2 = calculateResult(gimbal_control._3508_RISE1_height_L_motor.relative_angle , kkkk2 , bbbb2);
}

//8.79874325 到  7.14568329
float joint_angles[JOINTS][POSITIONS_PER_JOINT][ANGLES_PER_POSITION] = {
     // 第一个关节的位置
     {
         // 位置1的角度值
         {-14.20755f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         // 位置2的角度值
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         // 位置3的角度值
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         // 位置4的角度值
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         // 位置5的角度值
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         // 位置6的角度值
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
     },
     // 第二个关节
     {
         {-122.20755f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
     },
     // 3
	{
         {-17.4216518f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
     },   
	// 4
	 {
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
     },     
	// 5
	 {
         {7.7f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
     },    
	//6
	  {
         {0.130930564f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
     },
	//7
	  {
         {0.416891992f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
         {0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f,	  0.0f},
     },
};
//以下

	
	// switch(gimbal_set_mode->gimbal_rc_ctrl->key.v)
	// {
	// 	case KEY_PRESSED_OFFSET_SHIFT+KEY_PRESSED_OFFSET_Z :
	// 	if((gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE == Storage_mechanism_Open || gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE == Storage_mechanism_Initial_position)&& gimbal_set_mode->Gimbal_Height == HEIGHT_Min)
	// 	{
	// 		gimbal_set_mode->Gimbal_LeftRight_set 		=LEFTRIGHT_second;
	// 		gimbal_set_mode->Gimbal_LeftRight_mode_Transit = LEFTRIGHT_mode_const;
	// 	}
	// 	else
	// 	{
	// 		gimbal_set_mode->Gimbal_LeftRight_set 		=LEFTRIGHT_Min;
	// 		gimbal_set_mode->Gimbal_LeftRight_mode_Transit = LEFTRIGHT_mode_const;
	// 	}break;
	// 	case KEY_PRESSED_OFFSET_SHIFT+KEY_PRESSED_OFFSET_X : 
	// 		gimbal_set_mode->Gimbal_LeftRight_set 		=LEFTRIGHT_second; 	
	// 		gimbal_set_mode->Gimbal_LeftRight_mode_Transit = LEFTRIGHT_mode_const;break;
	// 	case KEY_PRESSED_OFFSET_SHIFT+KEY_PRESSED_OFFSET_C : 
	// 		gimbal_set_mode->Gimbal_LeftRight_set 		=LEFTRIGHT_thirdly;
	// 		gimbal_set_mode->Gimbal_LeftRight_mode_Transit = LEFTRIGHT_mode_const;
	// 		break;
	// 	case KEY_PRESSED_OFFSET_SHIFT+KEY_PRESSED_OFFSET_V : 
	// 		gimbal_set_mode->Gimbal_LeftRight_set 		=LEFTRIGHT_Max; 		
	// 		gimbal_set_mode->Gimbal_LeftRight_mode_Transit = LEFTRIGHT_mode_const;
	// 		break;

	// 	case KEY_PRESSED_OFFSET_CTRL+KEY_PRESSED_OFFSET_Z  : 
	// 		if(gimbal_set_mode->Gimbal_LeftRight == LEFTRIGHT_Min && (gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE == Storage_mechanism_Open || gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE == Storage_mechanism_Initial_position))
	// 		{
	// 			gimbal_set_mode->Gimbal_Height_set 			= HEIGHT_Max; 		
	// 			gimbal_set_mode->Gimbal_Height_mode_Transit   = HEIGHT_mode_const;
	// 		}			
	// 		else 
	// 		{
	// 			gimbal_set_mode->Gimbal_Height_set 			= HEIGHT_Min;
	// 			gimbal_set_mode->Gimbal_Height_mode_Transit   = HEIGHT_mode_const;
	// 		}break;	
	// 	case KEY_PRESSED_OFFSET_CTRL+KEY_PRESSED_OFFSET_X  : 
	// 		{
	// 			gimbal_set_mode->Gimbal_Height_set 			= HEIGHT_Max; 		
	// 			gimbal_set_mode->Gimbal_Height_mode_Transit   = HEIGHT_mode_const;
	// 		}break;
		
	// 	case KEY_PRESSED_OFFSET_CTRL+KEY_PRESSED_OFFSET_C  : 
	// 		gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE_set 	=Storage_mechanism_Close;	break;
		
	// 	case KEY_PRESSED_OFFSET_CTRL+KEY_PRESSED_OFFSET_V  :	
	// 	if(gimbal_set_mode->Gimbal_LeftRight == LEFTRIGHT_Min && gimbal_set_mode->Gimbal_Height == HEIGHT_Min)
	// 	{
	// 	 gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE_set 		= Storage_mechanism_Close; 		
	// 	gimbal_set_mode->Gimbal_Storage_mechanism_arrival_time_flag = 0;
	// 	}
	// 	else
	// 	{
	// 	 gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE_set 		= Storage_mechanism_Open;
	// 	gimbal_set_mode->Gimbal_Storage_mechanism_arrival_time_flag = 0;
	// 	}break;
		
	//   default:  break;
	// }
	
	//test
		// if(hand_control.gimbal_position == gimbal_position_1)
		// {
		// 	gimbal_set_mode->Gimbal_Height_set 			= HEIGHT_Max; 		
		// 	gimbal_set_mode->Gimbal_Height_mode_Transit   = HEIGHT_mode_const;
			
		// 	gimbal_set_mode->Gimbal_LeftRight_set 			= LEFTRIGHT_Max; 		
		// 	gimbal_set_mode->Gimbal_LeftRight_mode_Transit = LEFTRIGHT_mode_const;
			
		// 	gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE_set     = Storage_mechanism_Open;
		// }
		// else if(hand_control.gimbal_position == gimbal_position_2)
		// {
		// 	gimbal_set_mode->Gimbal_Height_set 			= HEIGHT_Min; 		
		// 	gimbal_set_mode->Gimbal_Height_mode_Transit   = HEIGHT_mode_const;
			
		// 	gimbal_set_mode->Gimbal_LeftRight_set 			= LEFTRIGHT_second; 		
		// 	gimbal_set_mode->Gimbal_LeftRight_mode_Transit = LEFTRIGHT_mode_const;
			
		// 	gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE_set     = Storage_mechanism_Open;
			
		// 	TIM_SetCompare4(TIM8,2100);/*CH4——前端吸盘*/
		// 	arr_hand_transition_flag = 2;
		// }
		// else if(hand_control.gimbal_position == gimbal_position_3)
		// {
			
		// 	gimbal_set_mode->Gimbal_LeftRight_set 			= LEFTRIGHT_thirdly;
		// 	gimbal_set_mode->Gimbal_LeftRight_mode_Transit = LEFTRIGHT_mode_const;
			
		// 	gimbal_set_mode->Gimbal_Height_set 			= HEIGHT_Min; 		
		// 	gimbal_set_mode->Gimbal_Height_mode_Transit   = HEIGHT_mode_const;
			
		// 	gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE_set     = Storage_mechanism_Open;
			
		// 	TIM_SetCompare4(TIM8,2100);/*CH4——前端吸盘*/
		// 	arr_hand_transition_flag = 2;
		// }
		// else if(hand_control.gimbal_position == gimbal_position_4)
		// {
		// 	gimbal_set_mode->Gimbal_Height_set 			= HEIGHT_Min; 		
		// 	gimbal_set_mode->Gimbal_Height_mode_Transit   = HEIGHT_mode_const;
			
		// 	gimbal_set_mode->Gimbal_LeftRight_set 			= LEFTRIGHT_Max; 		
		// 	gimbal_set_mode->Gimbal_LeftRight_mode_Transit = LEFTRIGHT_mode_const;
			
		// 	gimbal_set_mode->Gimbal_Storage_mechanism_ANGLE_set     = Storage_mechanism_Open;
		// }
		/***********************************************************云台气泵开关*******************************************************************/
/***********************************************************云台气泵开关*******************************************************************/
/***********************************************************云台气泵开关*******************************************************************/
// 	if(gimbal_set_control->gimbal_rc_ctrl->mouse.press_l == 1 && gimbal_set_control->Single_press_limit.Mouse_L)
// 	{
// 		gimbal_set_control->Single_press_limit.Mouse_L = 0;
		
// 		if(gimbal_set_control->Peripheral_status.vacuum_pump_hand_state == 0)
// 		{
// //	test		vacuum_pump_hand_1_OPEN;
			
// //			vacuum_pump_hand_2_OPEN;
// 		}
// 		else if(gimbal_set_control->Peripheral_status.vacuum_pump_hand_state == 1)
// 		{

// //			vacuum_pump_hand_1_CLOSE;
			
// 	//		vacuum_pump_hand_2_CLOSE;
// 		}
		
// 		if(gimbal_set_control->Peripheral_status.vacuum_pump_hand_state == 1)
// 		{
// 			gimbal_set_control->Peripheral_status.vacuum_pump_hand_state=0;
// 		}
// 		else if(gimbal_set_control->Peripheral_status.vacuum_pump_hand_state == 0)
// 		{
// 			gimbal_set_control->Peripheral_status.vacuum_pump_hand_state=1;
// 		}
// 	}
// 	else if(gimbal_set_control->gimbal_rc_ctrl->mouse.press_l == 0)
// 	{
// 		gimbal_set_control->Single_press_limit.Mouse_L = 1;
// 	}
	
// // 	if(hand_control.gimbal_position == gimbal_position_1)
// // 	{
// // //	test	vacuum_pump_hand_1_OPEN;
			
// // //		vacuum_pump_hand_2_OPEN;
// // 		gimbal_set_control->Peripheral_status.vacuum_pump_hand_state=1;
// // 	}
// /***********************************************************底盘气泵开关*******************************************************************/
// /***********************************************************底盘气泵开关*******************************************************************/
// /***********************************************************底盘气泵开关*******************************************************************/
// 	if(gimbal_set_control->gimbal_rc_ctrl->mouse.press_r == 1 && gimbal_set_control->Single_press_limit.Mouse_R)
// 	{
// 		gimbal_set_control->Single_press_limit.Mouse_R = 0;
		
// 		if(gimbal_set_control->Peripheral_status.vacuum_pump_Chassis_state == 0)
// 		{
// //	test		vacuum_pump_Chassis_OPEN;
// 		}
// 		else if(gimbal_set_control->Peripheral_status.vacuum_pump_Chassis_state == 1)
// 		{
// //	test		vacuum_pump_Chassis_CLOSE;
// 		}
		
// 		if(gimbal_set_control->Peripheral_status.vacuum_pump_Chassis_state == 1)
// 		{
// 			gimbal_set_control->Peripheral_status.vacuum_pump_Chassis_state = 0;
// 		}
// 		else if(gimbal_set_control->Peripheral_status.vacuum_pump_Chassis_state == 0)
// 		{
// 			gimbal_set_control->Peripheral_status.vacuum_pump_Chassis_state = 1;
// 		}
// 	}
// 	else if(gimbal_set_control->gimbal_rc_ctrl->mouse.press_r == 0)
// 	{
// 		gimbal_set_control->Single_press_limit.Mouse_R = 1;
// 	}
// /***********************************************************储矿开关*******************************************************************/
// /***********************************************************储矿开关*******************************************************************/
// /***********************************************************储矿开关*******************************************************************/
// 	if((gimbal_set_control->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT) == 0 && (gimbal_set_control->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL) == 0)
// 	{
// 		if((gimbal_set_control->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C) && gimbal_set_control->Single_press_limit.Keyboard_C)
// 		{/*900，2200*//*700~2500*/
// 					gimbal_set_control->Single_press_limit.Keyboard_C = 0;
// 					if(gimbal_set_control->Peripheral_status.Keyboard_C_state == 0)
// 					{
// 	//test					TIM_SetCompare1(TIM8,2200);
// 					}
// 					else if(gimbal_set_control->Peripheral_status.Keyboard_C_state == 1)
// 					{
// 	//test					TIM_SetCompare1(TIM8,700);
// 					}
					
// 					if(gimbal_set_control->Peripheral_status.Keyboard_C_state == 0)
// 					{
// 						gimbal_set_control->Peripheral_status.Keyboard_C_state = 1;
// 					}
// 					else if(gimbal_set_control->Peripheral_status.Keyboard_C_state == 1)
// 					{
// 						gimbal_set_control->Peripheral_status.Keyboard_C_state = 0;
// 					}
// 		}
// 		else if((gimbal_set_control->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C) == 0)
// 		{
// 			gimbal_set_control->Single_press_limit.Keyboard_C = 1;
// 		}
// 	}
// /***********************************************************前端吸盘旋转*******************************************************************/
// /***********************************************************前端吸盘旋转*******************************************************************/
// /***********************************************************前端吸盘旋转*******************************************************************/
// 	if((gimbal_set_control->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT) == 0 && (gimbal_set_control->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL) == 0)
// 	{
// 		if((gimbal_set_control->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V) && gimbal_set_control->Single_press_limit.Keyboard_V)
// 		{/*CH4——前端吸盘*//*1400,2100*//*500~2400*/
// 					gimbal_set_control->Single_press_limit.Keyboard_V = 0;
// 					if(gimbal_set_control->Peripheral_status.Keyboard_V_state == 0)
// 					{
// 		//test				TIM_SetCompare4(TIM8,1400);/*CH4——前端吸盘*/
// 						arr_hand_transition_flag = 1;
// 					}
// 					else if(gimbal_set_control->Peripheral_status.Keyboard_V_state == 1)
// 					{
// 		//test				TIM_SetCompare4(TIM8,2100);/*CH4——前端吸盘*/
// 						arr_hand_transition_flag = 2;
// 					}
					
// 					if(gimbal_set_control->Peripheral_status.Keyboard_V_state == 0)
// 					{
// 						gimbal_set_control->Peripheral_status.Keyboard_V_state = 1;
// 					}
// 					else if(gimbal_set_control->Peripheral_status.Keyboard_V_state == 1)
// 					{
// 						gimbal_set_control->Peripheral_status.Keyboard_V_state = 0;
// 					}
// 		}
// 		else if((gimbal_set_control->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V) == 0)
// 		{
// 			gimbal_set_control->Single_press_limit.Keyboard_V = 1;
// 			arr_hand_transition_flag = 0;
// 		}
// 	}
	
// 	//test
// 	// if(hand_control.gimbal_pitch_2_motor.hand_motor_measure_all.relative_angle_set >= 2.3f)
// 	// {
// 	// 	TIM_SetCompare4(TIM8,2100);/*CH4——前端吸盘*/
// 	// 	arr_hand_transition_flag = 2;
// 	// }

void gimbal_current_out(void)
{

	#if _3508_RISE1_height_L_TURN
	   gimbal_set_current1_4[0] = - gimbal_control._3508_RISE1_height_L_motor.given_current; 
	#else
	   gimbal_set_current1_4[0] = 600 + gimbal_control._3508_RISE1_height_L_motor.given_current; 
	#endif
	
	#if _3508_RISE2_height_R_TURN
	  gimbal_set_current1_4[1] = -600 - gimbal_control._3508_RISE2_height_R_motor.given_current;
	#else 
	  gimbal_set_current1_4[1] =   gimbal_control._3508_RISE2_height_R_motor.given_current;
	#endif
				
	#if _3508_LeftRight_TURN	
	  gimbal_set_current1_4[2] = - gimbal_control._3508_LeftRight_motor.given_current;
	#else 
	  gimbal_set_current1_4[2] =   gimbal_control._3508_LeftRight_motor.given_current;
	#endif
	
  if (gimbal_set_current1_4[0] < 200) 
	  gimbal_set_current1_4[0] = 200; 
	if (gimbal_set_current1_4[1] > -200) 
	   gimbal_set_current1_4[1] = 200; 

  CanSendMess(&hfdcan3,SEND_ID201_204,gimbal_set_current1_4); 
}

