#ifndef _HAND_TASK_H
#define _HAND_TASK_H
#include "main.h"
#include "hand_task_interface.h"

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

/*******************************************************************/
//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define Channel_0 0
#define Channel_1 1
#define Channel_2 2
#define Channel_3 3
#define Channel_4 4
#define ModeChannel 0

/*控制器控制电机角度单位时间的变化幅度比例*/
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



/*机械臂主任务*/
void Hand_task(void const *pvParameters);//机械臂主任务

#endif
