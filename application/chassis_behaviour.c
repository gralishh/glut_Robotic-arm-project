
/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      完成底盘行为任务。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#include "chassis_behaviour.h"

#include "chassis_task.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "chassis_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

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

/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);


/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set底盘设置的旋转速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set底盘设置的旋转速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

//底盘行为状态机
static chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
/***********************************************************************
************************************************************************/	
    if (chassis_move_mode == NULL)
    {
        return;
    }
	static uint8_t Keyboard_move_flag = 0;
    //遥控器设置行为模式
	if(		chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_W\
		|| 	chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_S\
		|| 	chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_A\
		|| 	chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_D\
		||	chassis_move_mode->chassis_RC->mouse.x || chassis_move_mode->chassis_RC->mouse.y)
		/*|| 	chassis_move_mode->chassis_RC->k
	ey.v & KEY_PRESSED_OFFSET_Q\
		|| 	chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_E)*/
	{    	Keyboard_move_flag = 1;}
	else 	Keyboard_move_flag = 0;
	
	 if ((switch_is_mid(chassis_move_mode->chassis_RC->rc.s[1]) && switch_is_down(chassis_move_mode->chassis_RC->rc.s[0])) || Keyboard_move_flag)
    {
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    }
	else if(switch_is_down(chassis_move_mode->chassis_RC->rc.s[1]) && switch_is_mid(chassis_move_mode->chassis_RC->rc.s[0]))
	{
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
	}
	else chassis_behaviour_mode = CHASSIS_NO_MOVE;
	
//    if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
//    {
//        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
//    }
//	  else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
//    {
//        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
//    }
//    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
//    {
//        chassis_behaviour_mode =CHASSIS_NO_MOVE;
//    }

//    //云台进入某些状态的时候，底盘保持不动
//    if ( switch_is_up(chassis_move_mode->chassis_RC->rc.s[1])/*gimbal_cmd_to_chassis_stop()*/)
//    {
//        chassis_behaviour_mode = CHASSIS_NO_MOVE;
//    }
/*****************************************************************************************
		底盘行为状态机是在此源文件用于选择底盘任务状态机的一个中间状态
		底盘任务状态机在底盘任务中有使用
*****************************************************************************************/
    //根据行为状态机选择底盘状态机
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; //当行为是底盘无力，则设置底盘状态机为 raw，原生状态机。
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //当行为是底盘不移动，则设置底盘状态机为 底盘不跟随角度 状态机。
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //当行为是底盘不跟随角度，则设置底盘状态机为 底盘不跟随角度 状态机。
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; //当行为是底盘开环，则设置底盘状态机为 底盘原生raw 状态机。
    }
}

void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_open_set_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}
/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}
/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set底盘设置的旋转速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
	int16_t wz_channel;
	fp32 	wz_set_channel;
	if(switch_is_down(chassis_move_rc_to_vector->chassis_RC->rc.s[1]) && switch_is_mid(chassis_move_rc_to_vector->chassis_RC->rc.s[0]))
	{
		rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[0], wz_channel, CHASSIS_RC_DEADLINE);
		wz_set_channel = wz_channel * CHASSIS_WZ_RC_SEN;
	}
	else if(switch_is_mid(chassis_move_rc_to_vector->chassis_RC->rc.s[1]) && switch_is_down(chassis_move_rc_to_vector->chassis_RC->rc.s[0]))
	{
		/*rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], wz_channel, CHASSIS_RC_DEADLINE);*/
		rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[0], wz_channel, CHASSIS_RC_DEADLINE);
		wz_set_channel = wz_channel * CHASSIS_WZ_RC_SEN;
	}
	else wz_set_channel = 0.0f;
	
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
//	if(chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_Q)
//	{
//		wz_set_temp = -3.0f;/*需要测试*/
//	}
//	else if(chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_E)
//	{
//		wz_set_temp = 3.0f;
//	}
//	else wz_set_temp = 0.0f;
    *wz_set = -chassis_move_rc_to_vector->chassis_RC->mouse.x * CHASSIS_WZ_MOUSE_SEN - wz_set_channel;
		//在此写底盘选择代码
}

/**
  * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set底盘设置的旋转速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *wz_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}
