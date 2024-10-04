#include "chassis_task.h"
#include "arm_math.h"
#include "main.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "chassis_behaviour.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "gimbal_task.h"
//#include "PC_communication_task.h"

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

//  static uint32_t number[] = {4,3,5,4,2,4,2,5,3,1,2,4,5,1,3,5,2,3,1,4,  //随机数字数组
//							              	3,2,4,1,5,1,4,3,2,5,5,2,4,3,1,1,5,3,2,4,
//							              	3,4,1,2,5,5,3,1,4,2,3,1,2,5,4,1,4,5,2,3,
//								              2,4,5,1,3,5,2,4,1,3,3,1,4,2,5,1,2,3,4,5,
//                              5,3,2,1,4,2,5,3,4,1,2,5,4,1,3,5,1,4,2,3};	
			
//底盘运动数据
chassis_move_t chassis_move;
//底盘沿x轴运动的距离          表示底盘沿y轴运动的距离
fp32  chassis_movex;
fp32   chassis_movey;
float  chassis_power_limit_buffer_pid_proportion; //缓存pid算出来的比例值
//底盘初始化，主要是pid初始化
static void chassis_init(chassis_move_t *chassis_move_init);
//底盘状态机选择，通过遥控器的开关
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
//底盘数据更新
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
//底盘设置根据遥控器控制量
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
//底盘PID计算以及运动分解
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
//获取底盘功率
static void chassis_power_set(chassis_move_t *chassis_move_init_t);

//visuals_rx_data_t* visuals_rx_classis;
															
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

//主任务
void chassis_task(void const *pvParameters)
{
#if chassisControlBorad
     vTaskDelay(CHASSIS_TASK_INIT_TIME);  //空闲一段时间
	taskENTER_CRITICAL();//用于初始化避免中断
    //底盘初始化
    chassis_init(&chassis_move);   
    taskEXIT_CRITICAL();
    //判断底盘电机是否都在线
//   while (toe_is_error(TOE_3508_M1_ID) || toe_is_error(TOE_3508_M2_ID) || toe_is_error(TOE_3508_M3_ID) || toe_is_error(TOE_3508_M4_ID) || toe_is_error(DBUSTOE))
//   {
//           vTaskDelay(CHASSIS_CONTROL_TIME_MS);
//   }
	
    while (1)
    {
		    //更新底盘功率
		    chassis_power_set(&chassis_move);
        //遥控器设置状态
        chassis_set_mode(&chassis_move);
        //底盘数据更新
        chassis_feedback_update(&chassis_move);
        //底盘控制量设置
        chassis_set_contorl(&chassis_move);
        //底盘控制PID计算
        chassis_control_loop(&chassis_move);

 
		 int16_t chassis_set_current[4];

           //当遥控器掉线的时候，为relax状态，底盘电机指令为零，为了保证一定发送为零，故而不采用设置give_current的方法
            if (toe_is_error(DBUSTOE))
            {
                memset(chassis_set_current,0,sizeof(chassis_set_current));
            }
            else
            {
				for(char i = 0;i<4;i++)
				{
					chassis_set_current[i] = chassis_move.motor_chassis[i].give_current;
				}
            }
			
				CanSendMess(&hcan1,SEND_ID201_204,chassis_set_current); 
						
        
				
        //系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
        
        

    }
#endif  
}    
 fp32 MAX_WHEEL_SPEED;  //底盘运动速度上限		
 fp32 NORMAL_MAX_CHASSIS_SPEED_X; //底盘前进运动速度上限
 fp32 NORMAL_MAX_CHASSIS_SPEED_Y; //底盘平移运动速度上限

//设置底盘功率（速度）
static void chassis_power_set(chassis_move_t *chassis_move_init_t)
{
// //	if(Is_Speed_Limitde())
 			MAX_WHEEL_SPEED = 6.0f;/*1.8f;*/
// 	else
//			MAX_WHEEL_SPEED = 3.6f;/*3.6f;*/
	
	
			NORMAL_MAX_CHASSIS_SPEED_X=MAX_WHEEL_SPEED;
			NORMAL_MAX_CHASSIS_SPEED_Y=MAX_WHEEL_SPEED;//TOSET

		    //最大 最小速度
    chassis_move_init_t->vx_front_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init_t->vx_back_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init_t->vy_left_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init_t->vy_right_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
}

static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

		const static fp32 motor_speed_pid[4][3]={ 
			                                    MOTOR_1_SPEED_PID_KP,MOTOR_1_SPEED_PID_KI,MOTOR_1_SPEED_PID_KD ,\
		                                         MOTOR_2_SPEED_PID_KP,MOTOR_2_SPEED_PID_KI,MOTOR_2_SPEED_PID_KD ,\
		                                         MOTOR_3_SPEED_PID_KP,MOTOR_3_SPEED_PID_KI,MOTOR_3_SPEED_PID_KD ,\
		                                         MOTOR_4_SPEED_PID_KP,MOTOR_4_SPEED_PID_KI,MOTOR_4_SPEED_PID_KD ,\
		                                        };																				
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
		
		


    //底盘开机状态为停止
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    //初始化PID 运动
		uint8_t i;
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i); 
    }
		PID_Init(&chassis_move_init->motor_speed_pid[0], PID_POSITION, &motor_speed_pid[0][0], MOTOR_1_SPEED_PID_MAX_OUT, MOTOR_1_SPEED_PID_MAX_IOUT,0,0);  //四个底盘电机PID初始化
		PID_Init(&chassis_move_init->motor_speed_pid[1], PID_POSITION, &motor_speed_pid[1][0], MOTOR_2_SPEED_PID_MAX_OUT, MOTOR_2_SPEED_PID_MAX_IOUT,0,0);  //四个底盘电机PID初始化
		PID_Init(&chassis_move_init->motor_speed_pid[2], PID_POSITION, &motor_speed_pid[2][0], MOTOR_3_SPEED_PID_MAX_OUT, MOTOR_3_SPEED_PID_MAX_IOUT,0,0);  //四个底盘电机PID初始化
		PID_Init(&chassis_move_init->motor_speed_pid[3], PID_POSITION, &motor_speed_pid[3][0], MOTOR_4_SPEED_PID_MAX_OUT, MOTOR_4_SPEED_PID_MAX_IOUT,0,0);  //四个底盘电机PID初始化 
		

    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);	

    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}

static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    chassis_behaviour_mode_set(chassis_move_mode);
}


static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
	
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //更新电机速度，加速度是速度的PID微分
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE ;//SSIS_CONTROL_FREQUENCE能是一个常量，表示底盘控制频率，它的单位可能是 Hz（赫兹）。在将 D 分量输出乘以底盘控制频率后，可以得到“每秒钟”应该调整的位置偏差量。通过这种方式，可以把 D 分量输出转化为位置偏差，并按照固定的频率进行位置修正，以达到控制底盘运动的目的。
    }

    //更新底盘前进速度 x， 平移速度y，旋转速度wz，坐标系为右手系
	/*	
		0	1
		3	2
	*/
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;


}

//遥控器的数据处理成底盘的前进vx速度，vy速度
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    //遥控器原始通道值
    int16_t vx_channel, vy_channel;
		//速度运算的中间变量
    fp32 vx_set_channel, vy_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
	if(switch_is_down(chassis_move_rc_to_vector->chassis_RC->rc.s[1]) && switch_is_mid(chassis_move_rc_to_vector->chassis_RC->rc.s[0]))
	{
		vx_channel = vy_channel = 0;
	}
	else 
	{
		rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
		rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
	}
		/***********************************************************************************************/

			vx_set_channel =   - vx_channel * CHASSIS_VX_RC_SEN;
			vy_set_channel =   - vy_channel * CHASSIS_VY_RC_SEN;
	 	
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
				vx_set_channel = -0.3f*chassis_move_rc_to_vector->vx_front_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
				vx_set_channel = -0.3f*chassis_move_rc_to_vector->vx_back_speed;
    }
	
	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY && chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)
    {
				vx_set_channel = -chassis_move_rc_to_vector->vx_front_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY && chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)
    {
				vx_set_channel = -chassis_move_rc_to_vector->vx_back_speed;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
				vy_set_channel = 0.6f*chassis_move_rc_to_vector->vy_left_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
				vy_set_channel = 0.6f*chassis_move_rc_to_vector->vy_right_speed;
    }
	
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY && chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)
    {
				vy_set_channel = 1.5f*chassis_move_rc_to_vector->vy_left_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY && chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)
    {
				vy_set_channel = 1.5f*chassis_move_rc_to_vector->vy_right_speed;
    }

    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

//    //停止信号，不需要缓慢加速，直接减速到零
//    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
//    {
//        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
//    }

//    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
//    {
//        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
//    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}



extern uint8_t IsGimbalMotionless(void);

//设置遥控器输入控制量
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }

    //设置速度
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);
//    chassis_move_control->vx += visuals_rx_classis->vx; //视觉部分
//    chassis_move_control->vy += visuals_rx_classis->vy;
     if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {

        //放弃跟随云台
        //这个模式下，角度设置的为 角速度
        chassis_move_control->wz_set = angle_set;
		//fp32_constrain为速度限幅函数
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_back_speed, chassis_move_control->vx_front_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_right_speed, chassis_move_control->vy_left_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
}
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])//计算wheel_speed
{
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    wheel_speed[0] = - vx_set - vy_set + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] =   vx_set - vy_set + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] =   vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = - vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);//eel_speed来自于static void chassis_vector_to_mecanum_wheel_speed

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //赋值电流值
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
				
        //raw控制直接返回
        return;
    }
    //计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)//TODO
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    } 
//    for (i = 0; i < 4; i++)
//    {
//		chassis_move_control_loop->motor_chassis[i].speed_set=0.8f;
//		    }
    //计算pid
  // chassis_move_control_loop->motor_chassis[i].speed反馈速度
    for (i = 0; i < 4; i++)
    {
        PID_Calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }
		
    //赋值电流值
    for (i = 0; i < 4; i++)
    {
			//最终
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }			
}



float  GetChassisMaxOutput()
{
	return    chassis_move.motor_speed_pid[0].max_out; 
}

Chassis_Motor_t *getChassisGive_current()
{
  return chassis_move.motor_chassis;
}



