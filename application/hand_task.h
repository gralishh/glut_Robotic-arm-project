#ifndef _HAND_TASK_H
#define _HAND_TASK_H
#include "main.h"

#include "bsp_buzzer.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "pid.h"
#include "gimbal_task.h"
#include "External_encoder.h"
#include "referee.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "queue.h"
#include "arm_math.h" 
#include "cmsis_os.h"
#include "motor_control.h"
#include "gimbal_task.h"
#include "MCU_communicaton_task.h"

#define	roll_1_2006_SPEED_RATE  1.0f

#define roll_1_SPEED_PID_KP 18.0f //600
#define roll_1_SPEED_PID_KI 0.0f
#define roll_1_SPEED_PID_KD 0.0f
#define roll_1_SPEED_PID_MAX_OUT   8000.0f
#define roll_1_SPEED_PID_MAX_IOUT 16000.0f

#define roll_1_relative_angle_PID_KP 200.0f
#define roll_1_relative_angle_PID_KI 0.00f
#define roll_1_relative_angle_PID_KD 0.0f
#define roll_1_relative_angle_PID_MAX_OUT 500.0f
#define roll_1_relative_angle_PID_MAX_IOUT 0.0f

#define	pitch_2_3508_SPEED_RATE  -1.0f

#define pitch_2_SPEED_PID_KP 18.0f 
#define pitch_2_SPEED_PID_KI 0.0f
#define pitch_2_SPEED_PID_KD 0.0f
#define pitch_2_SPEED_PID_MAX_OUT   8000.0f
#define pitch_2_SPEED_PID_MAX_IOUT 16000.0f

#define pitch_2_relative_angle_PID_KP 200.0f
#define pitch_2_relative_angle_PID_KI 0.00f
#define pitch_2_relative_angle_PID_KD 0.0f
#define pitch_2_relative_angle_PID_MAX_OUT 500.0f
#define pitch_2_relative_angle_PID_MAX_IOUT 0.0f



#define	roll_3_3508_SPEED_RATE  1.0f

#define roll_3_SPEED_PID_KP 0.10f//0.13 
#define roll_3_SPEED_PID_KI 0.0f
#define roll_3_SPEED_PID_KD 0.0f
#define roll_3_SPEED_PID_MAX_OUT   16000.0f
#define roll_3_SPEED_PID_MAX_IOUT 16000.0f

#define roll_3_relative_angle_PID_KP 13.0f //15
#define roll_3_relative_angle_PID_KI 0.0f
#define roll_3_relative_angle_PID_KD 0.0f    //?
#define roll_3_relative_angle_PID_MAX_OUT 500.0f
#define roll_3_relative_angle_PID_MAX_IOUT 2.0f

#define  hand_roll_3_GM_Send_Kp_Pos         0.315f
#define  hand_roll_3_GM_Send_Kd_Speed       0
#define  hand_roll_3_GM_Send_Effort         0
#define  GM_roll_3_ID						1

#define	x_4_3508_SPEED_RATE  0.003f

#define x_4_SPEED_PID_KP  3000.0f //800
#define x_4_SPEED_PID_KI 0.0f
#define x_4_SPEED_PID_KD 0.0f
#define x_4_SPEED_PID_MAX_OUT   28000.0f
#define x_4_SPEED_PID_MAX_IOUT 28000.0f

#define x_4_relative_angle_PID_KP 500.0F  //200
#define x_4_relative_angle_PID_KI 0.00f
#define x_4_relative_angle_PID_KD 0.0f																				
#define x_4_relative_angle_PID_MAX_OUT 500.0f
#define x_4_relative_angle_PID_MAX_IOUT 0.0f


#define	yaw_5_3508_SPEED_RATE 1.0f

#define yaw_5_SPEED_PID_KP  0.075f //0.085f`
#define yaw_5_SPEED_PID_KI  0.0f
#define yaw_5_SPEED_PID_KD  0.0f
#define yaw_5_SPEED_PID_MAX_OUT   28000.0f
#define yaw_5_SPEED_PID_MAX_IOUT 28000.0f

#define yaw_5_relative_angle_PID_KP 10.0f  //16
#define yaw_5_relative_angle_PID_KI  0.0f //0.012f 
#define yaw_5_relative_angle_PID_KD  0.0f	//90.0f																			
#define yaw_5_relative_angle_PID_MAX_OUT 500.0f
#define yaw_5_relative_angle_PID_MAX_IOUT 2.0f

#define hand_Yaw_5_GM_Send_Kp_Pos         0.8f
#define hand_Yaw_5_GM_Send_Kd_Speed       0
#define hand_Yaw_5_GM_Send_Effort         0
#define GM_Yaw_5_ID					   3

/***********************************************************
**********************************************************/

#define HAND_ACCEL_1_NUM 0.9f
#define HAND_ACCEL_2_NUM 0.9f

/*************************************************************/

//电机是否反装
#define roll_1_TURN     0   
#define pitch_2_TURN    1
#define roll_3_TURN      0
#define x_4_TURN        0
#define yaw_5_TURN      0

/**************************************************************/

//电机码盘值最大以及中值
#define Half_ecd_range 4096
#define ecd_range 8191


#define roll_1_encoder_ecd_offset        0
#define roll_1_encoder_angle_offset      0
#define roll_1_relative_ecd_offset       0  
#define roll_1_relative_angle_offset     0
#define roll_1_relative_angle_set_max    0.0f
#define roll_1_relative_angle_set_min    0.0f


#define pitch_2_encoder_ecd_offset        0
#define pitch_2_encoder_angle_offset      0 
#define pitch_2_relative_ecd_offset       0  
#define pitch_2_relative_angle_offset     0
#define pitch_2_relative_angle_set_max    0.0f
#define pitch_2_relative_angle_set_min    0.0f


#define roll_3_encoder_offset             0  
#define roll_3_relative_offset            0  
#define roll_3_relative_angle_set_max    -3.0f
#define roll_3_relative_angle_set_min     -30.0f


#define x_4_encoder_ecd_offset            0
#define x_4_encoder_angle_offset          0.0f
#define x_4_relative_offset               0 
#define x_4_relative_angle_set_max        0.16f
#define x_4_relative_angle_set_min        0.0f


#define yaw_5_encoder_offset  0
#define yaw_5_relative_offset  0 
#define yaw_5_relative_angle_set_max 14.0f
#define yaw_5_relative_angle_set_min 1.0f
#define yaw_5_custom_angle_set_max   0.0f
#define yaw_5_custom_angle_set_min   0.0f

/***************************************************************/
#define roll_1_reduction_radio     36
#define pitch_2_reduction_radio    36
#define x_4_reduction_radio        19

/**************************************************************/
#define Motor_3508_Ecd_to_ANGLE     3.14151f

#define Motor_Ecd_to_expansion      0.00000434894f
#define x_4_ecd_offset              -9024
#define x_4_expansion_offset        0

#define arr_X_chu 1600  //1670
#define arr_X_limit_max 2500
#define arr_X_limit_min 500
#define k_yaw -67.44f
#define b_yaw 622.5f

#define arr_Y_chu 1700
#define arr_Y_limit_max 2500
#define arr_Y_limit_min 500

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} Hand_PID_t;
/*******************************************************************/
#define HAND_CONTROL_TIME 10
/*******************************************************************/
//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define Channel_0 0
#define Channel_1 1
#define Channel_2 2
#define Channel_3 3
#define Channel_4 4
#define ModeChannel 0


//#define	hand_1_channel_RC_SEN  -0.00000030f   //-0.00000008f
//#define	hand_2_channel_RC_SEN   0.000005f
//#define	hand_3_channel_RC_SEN   0.0000001f
//#define	hand_4_channel_RC_SEN  -0.000001f
//#define	hand_5_channel_RC_SEN  -0.0000030f

#define	hand_1_channel_RC_SEN  -0.0000030f   //-0.00000008f
#define	hand_2_channel_RC_SEN   0.0000001f
#define	hand_3_channel_RC_SEN  -0.000005f
#define	hand_4_channel_RC_SEN  0.00006f
#define	hand_5_channel_RC_SEN  -0.00003f

//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间 6s后解除初始化状态，
#define HAND_ANGLE_ERROR1 0.2f
#define HAND_ANGLE_ERROR2 0.2f
#define HAND_ANGLE_ERROR3 0.09f
#define HAND_ANGLE_ERROR4 0.05f
#define HAND_ANGLE_ERROR5 0.09f
#define HAND_ANGLE_ERROR6 0.03f
#define HAND_ANGLE_ERROR7 0.03f

#define HAND_CUSTOM_STOP_TIME 100
#define HAND_CUSTOM_TIME 6000


#define Keyboard_set_Angle_increment_hand_1 0.03f
#define Keyboard_set_Angle_increment_hand_2 0.03f
#define Keyboard_set_Angle_increment_hand_3 0.06f
#define Keyboard_set_Angle_increment_hand_4 0.00006f
#define Keyboard_set_Angle_increment_hand_5 0.006f
#define Keyboard_set_Angle_increment_hand_6 0.00006f
#define Keyboard_set_Angle_increment_hand_7 0.00006f


#define hollow_encoder_pi 3.14151f
#define holoow_TIM_value  65536

/*用于修改传递给微机或另一个主控电机状态的数值*/
/*公式为:raw_value*kkkX+bbbX*/
#define kkkk3 1.0f
#define bbbb3 0.0f
#define kkkk4 1.0f
#define bbbb4 0.0f
#define kkkk5 1.0f
#define bbbb5 0.0f
#define kkkk6 1.0f
#define bbbb6 0.0f
#define kkkk7 1.0f
#define bbbb7 0.0f

typedef enum
{
    HAND_MOTOR_RAW = 0, //电机原始值控制
    HAND_MOTOR_ENCONDE, //电机编码值角度控制
} hand_motor_mode_e;

typedef enum
{
	
  HAND_ZERO_FORCE = 0, 	//无力
  HAND_INIT,            //初始化
  HAND_all_Operation,	//正常运转
  HAND_MOTIONLESS,     	//长时间无信号自锁
  HAND_CUSTOM,          //自定义控制 可以用于指定位置，自定义控制器和运动规划时不接受遥控器
} hand_behaviour_e;



typedef enum
{
	
  TIM1Channel1 = 0, 	
  TIM1Channel2,       
  TIM1Channel3, 
  TIM1Channel4, 
  TIM1Channel5, 
	
}HollowChannel;





/******************************************************************/
/**
 * @brief 关节电机句柄，包括反馈信息与反馈报文结构体
 */
typedef struct
{
		const motor_measure_t *hand_motor_measure;
		
		PidTypeDef hand_motor_speed_pid;
		PidTypeDef hand_motor_relative_pid;
	
		fp32 max_relative_angle; 
		fp32 min_relative_angle; 
	
		fp32 lsat_relative_angle;
		fp32 relative_angle; 
	  fp32 relative_angle_set;    
		fp32 motor_encoder_angle;  
	  fp32 last_encoder_angle;
		fp32 last_ecd;   
		fp32 speed_ref; 
		fp32 relative_init_angle;
		  
		fp32 original_RM_angle;
		int32_t motor_circle;

		fp32 lock_angle;					
		
		fp32 speed_set;
		fp32 motor_position_set;
		fp32 current_set;            
		int16_t given_current;      
} Hand_Motor_t;


typedef struct
{
	int32_t hollowEncoderOriginalUp;
	int32_t hollowEncoderOriginalDown;
	int32_t hollowEncoderOriginaEcd;
	int32_t hollowEncoderSum;

} EncoderHollowMotor_t;



typedef struct
{

	Hand_Motor_t hand_motor_measure_all; 
	EncoderHollowMotor_t *hollowEncoderMotor;
	
} Hand_roll_1_Motor_t;

typedef struct
{
		// const ENCODER_uart_t *hand_encoder_2_measure;
		EncoderHollowMotor_t *hollowEncoderMotor;
		Hand_Motor_t hand_motor_measure_all;
} Hand_pitch_2_Motor_t;

typedef struct
{
		// const ENCODER_uart_t *hand_encoder_3_measure;
		MOTOR_recv *rData;
	  MOTOR_send GM_Send_Data;
		Hand_Motor_t hand_motor_measure_all;
} Hand_roll_3_Motor_t;

typedef struct
{
		const external_encoder_t *hand_encoder_4_measure;
		Hand_Motor_t hand_motor_measure_all;
} Hand_x_4_Motor_t;

typedef struct
{
		Hand_Motor_t hand_motor_measure_all;
		MOTOR_recv *rData;
		MOTOR_send GM_Send_Data;
} Hand_yaw_5_Motor_t;

typedef enum
{
	
  hand_position_1 = 0, //逃跑	
  hand_position_2,     //取矿     
  hand_position_3,     //兑矿
  hand_position_4,     //地面矿
  hand_position_5,     //储矿
  hand_position_N      //初始化位置 逃跑
	 
} hand_position_e;

typedef enum
{
	
  hand_point_1 = 0, 
  hand_point_2,      
  hand_point_3,    
  hand_point_4,     
  hand_point_5,     
  hand_point_6      
	 
} hand_point_e;

typedef struct
{
		const RC_ctrl_t    *gimbal_rc_ctrl;
		elf_measure_t*           hand_custom_control;//自定义控制器
		visuals_rx_data_t*		   hand_visual_control;//

		hand_behaviour_e  hand_behaviour;//表示当前机械手的行为状态
		hand_behaviour_e  hand_behaviour_last;//表示上一次机械手的行为状态
		hand_position_e	  hand_position;//表示当前机械手的位置
		hand_point_e      hand_point;//表示路点

	  float roll_1_angle;
		float pitch_2_angle;
	
		float roll_1_angle_fact;
		float pitch_2_angle_fact;

    /*机械臂电机结构体,表示机械手的五个电机*/
		Hand_roll_1_Motor_t 		hand_roll_1_motor;
		Hand_pitch_2_Motor_t 		hand_pitch_2_motor;
		Hand_roll_3_Motor_t 		hand_roll_3_motor;
		Hand_x_4_Motor_t          	hand_x_4_motor;
		Hand_yaw_5_Motor_t      	hand_yaw_5_motor;	
	
    /*两个一阶滤波器类型的变量。它们用于实现机械手指令的缓慢更新*/
		first_order_filter_type_t hand_cmd_slow_set_1;
		first_order_filter_type_t hand_cmd_slow_set_2;
} Hand_Control_t;

/*机械臂主任务*/
void Hand_task(void const *pvParameters);//机械臂主任务

/*机械臂结构体初始化*/
static void Hand_Init(Hand_Control_t *gimbal_init);//机械臂初始化 主要是pid初始化

/*机械臂位置初始化*/
static void hand_position_Init(Hand_Control_t *gimbal_init);//机械臂初始化位置
static void hand_yaw_5_position_Init(Hand_Control_t *gimbal_init);
static void hand_roll_3_position_Init(Hand_Control_t *gimbal_init);

static void Hand_Set_Mode(Hand_Control_t *hand_motor_get);//云台遥控器设置模式与控制

/*更新电机设定值*/
static void Hand_Set_Contorl( Hand_Control_t *hand_set_control);//机械臂遥控器控制

/*设置机械臂位置*/
static void	Hand_Set_Position(Hand_Control_t *hand_set_position);//机械臂遥控器，键鼠设置位置
static void Hand_Custom_Set(Hand_Control_t *hand_set_position);

static void HAND_Mode_Change_Control_Transit(Hand_Control_t *hand_mode_change);//控制模式切换 控制数据过渡
static void HAND_combination_control(Hand_Control_t *hand_combination_control,fp32 relative_angle_set_5,fp32 relative_angle_set_4,fp32 relative_angle_set_3,fp32 relative_angle_set_2 , fp32 relative_angle_set_1 );

static void HAND_relative_angle_limit(Hand_Motor_t *hand_Motor_t,fp32 add,fp32 max_limit,fp32 min_limit);//电机角度限制
static fp32 fangle_limit(fp32 angle_mount,fp32 add,fp32 max_limit,fp32 min_limit);

/*机械臂PID计算(获取电流输出)*/
void Hand_Control_loop(void);//机械臂控制PID计算
static void hand_motor_speed_control(Hand_Motor_t *hand_motor,fp32 SPEED_RATE);//RM电机速度环(note:abandon)
/*电机停止(电流设为0)*/
static void hand_motor_raw_angle_control(Hand_Motor_t *hand_motor);//RM电机停止
static void hand_motor_GM_raw_angle_control(MOTOR_send *motor_send);//GM电机停止
/*电机'角度'串级pid计算*/
static void hand_motor_relative_angle_control(Hand_Motor_t *hand_motor,fp32 SPEED_RATE);//RM电机相对角度串级控制
static void hand_motor_GM_relative_angle_control(MOTOR_send *motor_send , Hand_Motor_t *hand_motor);
/*电机'自锁'串级pid计算*/
static void hand_motor_lock_angle_angle_control(Hand_Motor_t *hand_motor,fp32 SPEED_RATE);//RM电机自锁角度串级控制
static void hand_motor_GM_lock_angle_control(MOTOR_send *motor_send , Hand_Motor_t *hand_motor);

/*机械臂电机反馈*/
void Hand_Feedback_Update(void);//机械臂数据反馈
static fp32 encoder_ecd_to_relative_expansion(uint16_t ecd, int32_t circle_measure ,float last_relative_angle , int32_t offset_ecd , float offect_height);
static fp32 hand_motor_ecd_to_angle_change(int16_t ecd, int32_t offset_ecd , fp32 offset_angle);
static fp32 hand_motor_ecd_to_angle_2PI(int16_t ecd, int32_t offset_ecd , fp32 offset_angle);
static fp32 hand_motor_cic_relative_angle(float encoder_one_ecd, int64_t cic_mount , float offset_relative_ecd );
static  fp32  hand_motor_hollow_encoder_to_relative_angle_change(const EncoderHollowMotor_t* hand_hollow_encoder_change,int32_t offset_ecd, fp32 offset_angle , fp32 last_relative_angle);
static fp32 hand_motor_hollow_encoder_to_circle_angle(Hand_Motor_t hand_motor_measure , fp32 relative_angle_mid , uint8_t reduction_ratio);//由中空编码器值 计算电机多圈角度值
static void hand_motor_circle_measure(Hand_Motor_t *hand_motor);//计量圈数
static void View_steering_engine_control(Hand_Control_t *gimbal_control_loop );

static void Hand_Temperature_control(Hand_Control_t *hand_control);//用于调试控温

/*电机反转赋值*/
void Hand_Set_Reverse(void);

/*机械臂电机输出*/
void Hand_Current_Output(void);

/*上位机传递数据处理*/
void Get_Hand_Status(const Hand_Control_t hand_control);//只进行值传递
const hand_status_t *get_hand_status_point(void);


#endif
