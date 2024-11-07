#ifndef __HAND_TASK_INTERFACE__
#define __HAND_TASK_INTERFACE__

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
#include "struct_typedef.h"

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


extern Hand_Control_t	hand_control;
extern hand_status_t hand_status;


/*机械臂结构体初始化*/
void Hand_Init(Hand_Control_t *gimbal_init);//机械臂初始化 主要是pid初始化

/*机械臂位置初始化*/
void hand_position_Init(Hand_Control_t *gimbal_init);//机械臂初始化位置
void hand_yaw_5_position_Init(Hand_Control_t *gimbal_init);
void hand_roll_3_position_Init(Hand_Control_t *gimbal_init);

void Hand_Set_Mode(Hand_Control_t *hand_motor_get);//云台遥控器设置模式与控制

/*更新电机设定值*/
void Hand_Set_Contorl( Hand_Control_t *hand_set_control);//机械臂遥控器控制

/*设置机械臂位置*/
void	Hand_Set_Position(Hand_Control_t *hand_set_position);//机械臂遥控器，键鼠设置位置
void Hand_Custom_Set(Hand_Control_t *hand_set_position);

void HAND_Mode_Change_Control_Transit(Hand_Control_t *hand_mode_change);//控制模式切换 控制数据过渡
void HAND_combination_control(Hand_Control_t *hand_combination_control,fp32 relative_angle_set_5,fp32 relative_angle_set_4,fp32 relative_angle_set_3,fp32 relative_angle_set_2 , fp32 relative_angle_set_1 );

void HAND_relative_angle_limit(Hand_Motor_t *hand_Motor_t,fp32 add,fp32 max_limit,fp32 min_limit);//电机角度限制
fp32 fangle_limit(fp32 angle_mount,fp32 add,fp32 max_limit,fp32 min_limit);

/*机械臂PID计算(获取电流输出)*/
void Hand_Control_loop(void);//机械臂控制PID计算
void hand_motor_speed_control(Hand_Motor_t *hand_motor,fp32 SPEED_RATE);//RM电机速度环(note:abandon)
/*电机停止(电流设为0)*/
void hand_motor_raw_angle_control(Hand_Motor_t *hand_motor);//RM电机停止
void hand_motor_GM_raw_angle_control(MOTOR_send *motor_send);//GM电机停止
/*电机'角度'串级pid计算*/
void hand_motor_relative_angle_control(Hand_Motor_t *hand_motor,fp32 SPEED_RATE);//RM电机相对角度串级控制
void hand_motor_GM_relative_angle_control(MOTOR_send *motor_send , Hand_Motor_t *hand_motor);
/*电机'自锁'串级pid计算*/
void hand_motor_lock_angle_angle_control(Hand_Motor_t *hand_motor,fp32 SPEED_RATE);//RM电机自锁角度串级控制
void hand_motor_GM_lock_angle_control(MOTOR_send *motor_send , Hand_Motor_t *hand_motor);

/*机械臂电机反馈*/
void Hand_Feedback_Update(void);//机械臂数据反馈
fp32 encoder_ecd_to_relative_expansion(uint16_t ecd, int32_t circle_measure ,float last_relative_angle , int32_t offset_ecd , float offect_height);
fp32 hand_motor_ecd_to_angle_change(int16_t ecd, int32_t offset_ecd , fp32 offset_angle);
fp32 hand_motor_ecd_to_angle_2PI(int16_t ecd, int32_t offset_ecd , fp32 offset_angle);
fp32 hand_motor_cic_relative_angle(float encoder_one_ecd, int64_t cic_mount , float offset_relative_ecd );
 fp32  hand_motor_hollow_encoder_to_relative_angle_change(const EncoderHollowMotor_t* hand_hollow_encoder_change,int32_t offset_ecd, fp32 offset_angle , fp32 last_relative_angle);
fp32 hand_motor_hollow_encoder_to_circle_angle(Hand_Motor_t hand_motor_measure , fp32 relative_angle_mid , uint8_t reduction_ratio);//由中空编码器值 计算电机多圈角度值
void hand_motor_circle_measure(Hand_Motor_t *hand_motor);//计量圈数
void View_steering_engine_control(Hand_Control_t *gimbal_control_loop );

void Hand_Temperature_control(Hand_Control_t *hand_control);//用于调试控温

/*电机反转赋值*/
void Hand_Set_Reverse(void);

/*机械臂电机输出*/
void Hand_Current_Output(void);

/*dbus掉线处理*/
void Hand_Dbus_Offline_Control(void);

/*上位机传递数据处理*/
void Get_Hand_Status(const Hand_Control_t hand_control);//只进行值传递
const hand_status_t *get_hand_status_point(void);


#endif
