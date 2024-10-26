#ifndef _gimbal_TASK_H
#define _gimbal_TASK_H

#include "main.h"
#include "pid.h"
#include "user_lib.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "MCU_communicaton_task.h"
#include "freertos.h"
#include "timers.h"
#include "hand_task.h"

#define Mouse_control_sensitivity_X 1
#define Mouse_control_sensitivity_Y 1
#define Mouse_control_sensitivity_hand 1


/******************************升降******************************/
/**************************速度比例******************************/
#define _height_SPEED_Rate	    0.005f
/**************************速度单环pid***************************/
#define _height_SPEED_MOTOR_3508_KP       	0.0f //1800
#define _height_SPEED_MOTOR_3508_KI       	0.0f
#define _height_SPEED_MOTOR_3508_KD       	0.0f
#define _height_SPEED_MOTOR_3508_MAX_OUT  	16000.0f
#define _height_SPEED_MOTOR_3508_MAX_IOUT  	16000.0f
/************************角度差值单环pid***************************/
#define _height_differeence_balance_angle_kP  2000.0f
#define _height_differeence_balance_angle_kI  0.0f
#define _height_differeence_balance_angle_kD  0.0f
#define _height_differeence_balance_angle_MAX_OUT   2000.0f
#define _height_differeence_balance_angle_MAX_IOUT  16000.0f
/**************************双环pid********************************/
#define _height_relative_speed_MOTOR_3508_KP       	4000.0f
#define _height_relative_speed_MOTOR_3508_KI       	0.0f
#define _height_relative_speed_MOTOR_3508_KD       	0.0f
#define _height_relative_speed_MOTOR_3508_MAX_OUT  	5000.0f
#define _height_relative_speed_MOTOR_3508_MAX_IOUT  4000.0f

#define _height_relative_angle_MOTOR_3508_KP       	500.0f //16
#define _height_relative_angle_MOTOR_3508_KI       	0.0f
#define _height_relative_angle_MOTOR_3508_KD       	0.0f
#define _height_relative_angle_MOTOR_3508_MAX_OUT  	500.0f
#define _height_relative_angle_MOTOR_3508_MAX_IOUT  500.0f

#define _3508_RISE1_height_L_SPEED_Rate	    _height_SPEED_Rate
#define _3508_RISE2_height_R_SPEED_Rate	    -1.0f*_height_SPEED_Rate

#define _3508_RISE1_height_L_SPEED_PID_KP  		_height_SPEED_MOTOR_3508_KP
#define _3508_RISE1_height_L_SPEED_PID_KI  		_height_SPEED_MOTOR_3508_KI
#define _3508_RISE1_height_L_SPEED_PID_KD  		_height_SPEED_MOTOR_3508_KD
#define _3508_RISE1_height_L_SPEED_PID_MAX_OUT   _height_SPEED_MOTOR_3508_MAX_OUT
#define _3508_RISE1_height_L_SPEED_PID_MAX_IOUT  _height_SPEED_MOTOR_3508_MAX_IOUT

#define _3508_RISE2_height_R_SPEED_PID_KP  		_height_SPEED_MOTOR_3508_KP
#define _3508_RISE2_height_R_SPEED_PID_KI  		_height_SPEED_MOTOR_3508_KI
#define _3508_RISE2_height_R_SPEED_PID_KD  		_height_SPEED_MOTOR_3508_KD
#define _3508_RISE2_height_R_SPEED_PID_MAX_OUT   _height_SPEED_MOTOR_3508_MAX_OUT
#define _3508_RISE2_height_R_SPEED_PID_MAX_IOUT 	_height_SPEED_MOTOR_3508_MAX_IOUT

#define _3508_RISE1_height_L_relative_speed_PID_KP 		_height_relative_speed_MOTOR_3508_KP
#define _3508_RISE1_height_L_relative_speed_PID_KI 		_height_relative_speed_MOTOR_3508_KI
#define _3508_RISE1_height_L_relative_speed_PID_KD 		_height_relative_speed_MOTOR_3508_KD
#define _3508_RISE1_height_L_relative_speed_PID_MAX_OUT 	_height_relative_speed_MOTOR_3508_MAX_OUT
#define _3508_RISE1_height_L_relative_speed_PID_MAX_IOUT   _height_relative_speed_MOTOR_3508_MAX_IOUT

#define _3508_RISE1_height_L_relative_angle_PID_KP 			_height_relative_angle_MOTOR_3508_KP
#define _3508_RISE1_height_L_relative_angle_PID_KI 			_height_relative_angle_MOTOR_3508_KI
#define _3508_RISE1_height_L_relative_angle_PID_KD 			_height_relative_angle_MOTOR_3508_KD
#define _3508_RISE1_height_L_relative_angle_PID_MAX_OUT 	_height_relative_angle_MOTOR_3508_MAX_OUT
#define _3508_RISE1_height_L_relative_angle_PID_MAX_IOUT 	_height_relative_angle_MOTOR_3508_MAX_IOUT

#define _3508_RISE2_height_R_relative_speed_PID_KP 		_height_relative_speed_MOTOR_3508_KP
#define _3508_RISE2_height_R_relative_speed_PID_KI 		_height_relative_speed_MOTOR_3508_KI
#define _3508_RISE2_height_R_relative_speed_PID_KD 		_height_relative_speed_MOTOR_3508_KD
#define _3508_RISE2_height_R_relative_speed_PID_MAX_OUT 	_height_relative_speed_MOTOR_3508_MAX_OUT
#define _3508_RISE2_height_R_relative_speed_PID_MAX_IOUT 	_height_relative_speed_MOTOR_3508_MAX_IOUT

#define _3508_RISE2_height_R_relative_angle_PID_KP 			_height_relative_angle_MOTOR_3508_KP
#define _3508_RISE2_height_R_relative_angle_PID_KI 			_height_relative_angle_MOTOR_3508_KI
#define _3508_RISE2_height_R_relative_angle_PID_KD 			_height_relative_angle_MOTOR_3508_KD
#define _3508_RISE2_height_R_relative_angle_PID_MAX_OUT 	_height_relative_angle_MOTOR_3508_MAX_OUT
#define _3508_RISE2_height_R_relative_angle_PID_MAX_IOUT 	_height_relative_angle_MOTOR_3508_MAX_IOUT

/***************************滑台***************************/
/**************************速度比例***************************/
#define _3508_LeftRight1_height_L_SPEED_Rate	    -0.003f
/*************************速度单环pid***************************/
#define _3508_LeftRight_SPEED_PID_KP  		0.0f //10000
#define _3508_LeftRight_SPEED_PID_KI 		0.0f
#define _3508_LeftRight_SPEED_PID_KD 		0.0f
#define _3508_LeftRight_SPEED_PID_MAX_OUT   16000.0f
#define _3508_LeftRight_SPEED_PID_MAX_IOUT 	16000.0f
/**************************双环pid***************************/
#define _3508_LeftRight_relative_speed_PID_KP 2400.0f //60
#define _3508_LeftRight_relative_speed_PID_KI 0.0f
#define _3508_LeftRight_relative_speed_PID_KD 0.0f
#define _3508_LeftRight_relative_speed_PID_MAX_OUT 16000.0f
#define _3508_LeftRight_relative_speed_PID_MAX_IOUT 16000.0f

#define _3508_LeftRight_relative_angle_PID_KP 550.0f //16
#define _3508_LeftRight_relative_angle_PID_KI 0.0f
#define _3508_LeftRight_relative_angle_PID_KD 0.0f
#define _3508_LeftRight_relative_angle_PID_MAX_OUT  500.0f
#define _3508_LeftRight_relative_angle_PID_MAX_IOUT 500.0f

#define height_L_encoder_offset_ecd             -288
#define height_L_encoder_offset_height          0
#define height_R_encoder_offset_ecd             -256
#define height_R_encoder_offset_height          0
#define heightright_encoder_offset_ecd          -8571
#define heightright_encoder_offset_height       0



#define  _3508_RISE1_offset       				0 
#define  _3508_RISE1_relative_angle_set_max     0.46f
#define  _3508_RISE1_relative_angle_set_min     0.05f

#define _3508_RISE2_offset 					    0  
#define _3508_RISE2_relative_angle_set_max      0.46f
#define _3508_RISE2_relative_angle_set_min      0.05f

#define _3508_LeftRight_offset 					 0  
#define _3508_LeftRight_relative_angle_set_max   0
#define _3508_LeftRight_relative_angle_set_min   0


#define GIMBAL_ANGLE_ERROR 0.03f

#define gimbal_TASK_INIT_TIME 800

#define RC_deadband 10

#define Yaw_RC_SEN  -0.000002f
#define Pitch_RC_SEN -0.000006f //0.005

#define Yaw_Mouse_Sen 0.00006f
#define Pitch_Mouse_Sen 0.00002f

#define Yaw_Encoder_Sen 0.01f
#define Pitch_Encoder_Sen 0.01f

#define gimbal_CONTROL_TIME 10

#define gimbal_TEST_MODE 1

//表示电机正反转
#define _3508_RISE1_height_L_TURN 0
#define _3508_RISE2_height_R_TURN 1
#define _3508_LeftRight_TURN 1
#define _2006_Storage_mechanism_TURN 0
#define _3508_vacuum_pump_hand_TURN 0

#define ecd_range 8191

#define Motor_Ecd_to_ANGLE    0.222980217528f

#define	gimbal_1_channel_RC_SEN   0.00000005f
#define	gimbal_2_channel_RC_SEN   0.00000001f
#define	gimbal_3_channel_RC_SEN   -0.00000005f
#define	gimbal_4_channel_RC_SEN   0.00000005f
#define	gimbal_5_channel_RC_SEN   0.00000005f

#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192
#define Gimbal_Down_LeftRight_Speed_Set 0


#define Motor_Ecd_to_HEIGHT 0.000098f //      
#define Motor_Ecd_to_DISTANCE 0.0002347947265625 // 算错了应该除以32768   0.0000011306762695313 现在除以1024  37.05 

#endif

#define kkkk1 0 
#define bbbb1 0 
#define kkkk2 0 
#define bbbb2 0 

#define JOINTS 7 //最外
#define POSITIONS_PER_JOINT 6 //行数
#define ANGLES_PER_POSITION 6 //列数


typedef enum
{
	Vacuum_Pump_Open	=	0,             
	Vacuum_Pump_Close,       
	Vacuum_Pump_Runing, 
	Vacuum_Pump_NONE,       
}	Vacuum_Pump_hand_position_e;

typedef enum
{
	Storage_mechanism_Close	=	0,
	Storage_mechanism_Open,    
	Storage_mechanism_Running,
	Storage_mechanism_NONE, 

	
}	Gimbal_Storage_mechanism_ANGLE_Position_e;

typedef enum
{
	
  gimbal_position_1 = 0, //逃跑	
  gimbal_position_2,     //取矿     
  gimbal_position_3,     //兑矿
  gimbal_position_4,     //地面矿
  gimbal_position_5,     //储矿
  gimbal_position_N      //初始化位置 逃跑
	 
} gimbal_position_e;

typedef enum
{
	
  gimbal_point_1 = 0, 
  gimbal_point_2,      
  gimbal_point_3,    
  gimbal_point_4,     
  gimbal_point_5,     
  gimbal_point_6      
	 
} gimbal_point_e;

typedef enum
{
	
  gimbal_ZERO_FORCE = 0, 	
  gimbal_INIT,          
  gimbal_CUSTIM,	
  gimbal_all_Operation,	
  gimbal_motionless,     //遥控器无输入一段时间后自锁角度
	
} gimbal_behaviour_e;


//8???
typedef struct
{
	uint8_t Storage_mechanism_Open_Value;
	uint8_t Storage_mechanism_Close_Value;
	

	uint8_t	all_value;
}	IO_t;

typedef struct
{
    const motor_measure_t      *gimbal_motor_measure;
	const external_encoder_t   *gimbal_encoder_measure;

	PidTypeDef        gimbal_motor_relative_speed_pid;			  
    PidTypeDef        gimbal_motor_relative_angle_pid;         
    PidTypeDef        gimbal_motor_speed_pid;				
	PidTypeDef        gimbal_motor_defference_angle_pid;	

    uint16_t offset_ecd;				     
	fp32     lock_angle;					 	
    fp32     relative_angle;     		
    fp32     relative_angle_set; 	
	fp32     encoder_angle;	
	fp32     speed_set;				  
    fp32     motor_position_set;		
    fp32     raw_cmd_current;				
    fp32     current_set;						
    int16_t  given_current;			
	int16_t  current_set_difference;
} Gimbal_Motor_t;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
	elf_measure_t*   hand_custom_control;

	
	IO_t			IO_value;                      
	

	Gimbal_Storage_mechanism_ANGLE_Position_e Gimbal_Storage_mechanism_ANGLE;      
	Gimbal_Storage_mechanism_ANGLE_Position_e Gimbal_Storage_mechanism_ANGLE_last; 
	Gimbal_Storage_mechanism_ANGLE_Position_e Gimbal_Storage_mechanism_ANGLE_set;  

	Vacuum_Pump_hand_position_e Hand_Vacuum_set;


	gimbal_position_e        gimbal_position;
	gimbal_point_e			 gimbal_point;
	gimbal_behaviour_e       gimbal_behaviour_last;
	gimbal_behaviour_e       gimbal_behaviour;

  Gimbal_Motor_t _3508_RISE1_height_L_motor;
  Gimbal_Motor_t _3508_RISE2_height_R_motor;
	Gimbal_Motor_t _3508_LeftRight_motor;


	
	uint8_t gimbal_in_permission1;
} Gimbal_Control_t;


void gimbal_task(void const *pvParameters);

static void gimbal_Init(Gimbal_Control_t *gimbal_init);//云台初始化

static void gimbal_Set_Mode(Gimbal_Control_t *gimbal_set_mode);//云台遥控器设置模式

static void gimbal_Set_Control(Gimbal_Control_t *gimbal_set_control);//云台遥控器以及键鼠设置控制量
static void gimbal_relative_angle_limit(Gimbal_Motor_t *gimbal_Motor,fp32 add,fp32 max_limit,fp32 min_limit);//云台相对角度限制

void gimbal_Feedback_Update(void);//云台数据反馈
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);//RM电机角度值转换为相对角度值
static fp32 encoder_ecd_to_relative_height(uint16_t ecd, int32_t offset_ecd, float offect_height);
static fp32 encoder_ecd_to_relative_distance(uint16_t ecd, int32_t offset_ecd, float offect_height);
static void gimbal_position_Init(Gimbal_Control_t *gimbal_init);


static void gimbal_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change);//控制模式切换 控制数据过渡

static void gimbal_Set_Position(Gimbal_Control_t *gimbal_set_position);//设置云台位置
static void GIMBAL_combination_control(Gimbal_Control_t *gimbal_combination_control,fp32 relative_angle_set_5,fp32 relative_angle_set_4,fp32 relative_angle_set_3);
static void Gimbal_Custom_Set(Gimbal_Control_t *gimbal_set_position);

void gimbal_Control_loop(void);//云台控制PID计算
static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor,fp32 SPEED_RATE);//RM电机相对角度串级pid
static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor);//RM电机停止
static void gimbal_motor_lock_angle_angle_control(Gimbal_Motor_t *gimbal_motor,fp32 SPEED_RATE);//RM电机自锁角度串级pid
static void gimbal_motor_difference_relative_angle_control(Gimbal_Control_t *gimbal_motor);//升降差速控制


static void gimbal_temperature_control(Gimbal_Control_t *gimbal_temperature_control); //用于调试控温

/*电机输出*/
void gimbal_current_out(void);

void Get_Gimbal_Status(const Gimbal_Control_t gimbal_control);//只进行值传递
const gimbal_status_t *get_gimbal_status_point(void);
void vTimerStorageCallback(TimerHandle_t xTimerStorage);


#endif

