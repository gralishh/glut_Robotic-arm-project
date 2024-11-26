#include "DJI_motor.h"

#include "CAN_receive.h"
#include "angle_process.h"
#include <string.h>

// 电流输出数组宏函数
#define IS_OUTPUT_ID_200H(id) ((id>=0x201)&&(id<=0x204))
#define IS_OUTPUT_ID_1FFH(id) ((id>=0x205)&&(id<=0x208))
#define GET_OUTPUT_CURRENT_INDEX(id) ((id-0x200)%4-1)

// Motor_Ctrl宏函数
#define __DJI_Motor_Ctrl_get_reverse_current(motor_ptr,current) (motor_ptr->reverse_flag?-current:current)
#define __DJI_Motor_Ctrl_get_torque(motor_ptr) ((motor_ptr->recv_pack).torque)
#define __DJI_Motor_Ctrl_get_speed(motor_ptr) ((motor_ptr->recv_pack).speed_rpm)
#define __DJI_Motor_Ctrl_get_ecd(motor_ptr) ((motor_ptr->recv_pack).ecd)
#define __DJI_Motor_Ctrl_get_ecd_angle(motor_ptr) (motor_ptr->ecd_angle)
#define __DJI_Motor_Ctrl_get_angle(motor_ptr) ((motor_ptr->circle_count)*PI*2+*(motor_ptr->ref_ptr))
#define __DJI_Motor_Ctrl_get_init_state(motor_ptr,state_flag) ((motor_ptr->init_state)&(state_flag))
#define __DJI_Motor_Ctrl_set_init_state(motor_ptr,state_flag) ((motor_ptr->init_state)|=(state_flag))

#define __DJI_Motor_Ctrl_write_current(motor_ptr,current) \
  if(IS_OUTPUT_ID_200H(motor_ptr->id)) \
    (motor_ptr->mounted_bus->output_current200H)[GET_OUTPUT_CURRENT_INDEX(motor_ptr->id)]=__DJI_Motor_Ctrl_get_reverse_current(motor_ptr,current); \
  if(IS_OUTPUT_ID_1FFH(motor_ptr->id)) \
    (motor_ptr->mounted_bus->output_current1FFH)[GET_OUTPUT_CURRENT_INDEX(motor_ptr->id)]=__DJI_Motor_Ctrl_get_reverse_current(motor_ptr,current); \


/*靠近角度边界且被监测值(速度，电流)仍向边界行去时触发*/
#define __DJI_Motor_Ctrl_boundary_ctrl(motor_ptr,detected_value) \
{ \
  if(__DJI_Motor_Ctrl_get_init_state(motor,MOTOR_ANGLE_LIMIT_INIT)) \
  { \
    if((motor->max_angle-motor->braking_angle)>__DJI_Motor_Ctrl_get_angle(motor) && \
    detected_value>0) \
      DJI_Motor_set_angle(motor,motor->max_angle); \
    else if((motor->min_angle+motor->braking_angle)<__DJI_Motor_Ctrl_get_angle(motor) && \
    detected_value<0) \
      DJI_Motor_set_angle(motor,motor->min_angle); \
  }  \
} \

void DJI_Motor_init(DJI_Motor_Ctrl_t* motor,DJI_Motor_Bus_t* bus,DJI_Motor_Type_e motor_type,uint16_t id)
{
  memset((void*)motor,0x0,sizeof(DJI_Motor_Ctrl_t));
  motor->type=motor_type;
  motor->id=id;
  motor->mounted_bus=bus;
  motor->ref_ptr=&(motor->ecd_angle);

  DJI_CANBus_add_motor(bus,motor);
  DJI_Motor_set_nonforce(motor);
}

/**
 * @brief 角度限制设置
 * @param[in,out] motor 电机控制句柄
 * @param[in] max_angle 最大角度
 * @param[in] min_angle 最小角度
 * @param[in] braking_angle 边界刹车角度
 * @note 力矩(电流)与速度控制时，当角度在
 * max_angle-braking_angle ~ max_angle
 * 或min_angle ~ min_angle+braking_angle
 * 的范围时，设置为最值的位置环控制
 */
void DJI_Motor_set_angle_limit(DJI_Motor_Ctrl_t* motor,fp32 max_angle,fp32 min_angle,fp32 braking_angle)
{
  if(motor==NULL)
    return;
  motor->max_angle=max_angle;
  motor->min_angle=min_angle;
  motor->braking_angle=braking_angle;
  __DJI_Motor_Ctrl_set_init_state(motor,MOTOR_ANGLE_LIMIT_INIT);
}

void inline DJI_Motor_set_reverse(DJI_Motor_Ctrl_t* motor)
{
  motor->reverse_flag=1;
}

/**
 * @brief 设置外部角度反馈
 */
void DJI_Motor_set_angle_feedback(DJI_Motor_Ctrl_t* motor,fp32* feedback_angle)
{
  motor->ref_ptr=feedback_angle;
  __DJI_Motor_Ctrl_set_init_state(motor,EXTERN_ENCODER_INIT);
}

/**
 * @brief 电机位置环PID初始化
 */
void DJI_Motor_Pos_PID_init(DJI_Motor_Ctrl_t* motor,enum PID_MODE pid_mode,
  fp32 Kp,fp32 Ki,fp32 Kd,
  fp32 max_out,fp32 max_iout)
{
  fp32 pid[3]={Kp,Ki,Kd};
  PID_Init(&(motor->pid_pos_loop),pid_mode,pid,max_out,max_iout,0.01,10);
  __DJI_Motor_Ctrl_set_init_state(motor,MOTOR_POS_PID_INIT);
}

/**
 * @brief 电机速度环PID初始化
 */
void DJI_Motor_Speed_PID_init(DJI_Motor_Ctrl_t* motor,enum PID_MODE pid_mode,
  fp32 Kp,fp32 Ki,fp32 Kd,
  fp32 max_out,fp32 max_iout)
{
  fp32 pid[3]={Kp,Ki,Kd};
  PID_Init(&(motor->pid_speed_loop),pid_mode,pid,max_out,max_iout,0.01,10);
  __DJI_Motor_Ctrl_set_init_state(motor,MOTOR_SPEED_PID_INIT);
}

void DJI_Motor_PID_set_deadband(DJI_Motor_Ctrl_t* motor,fp32 deadband)
{

}

void DJI_Motor_set_angle(DJI_Motor_Ctrl_t* motor, fp32 angle)
{
  if(!__DJI_Motor_Ctrl_get_init_state(motor,MOTOR_SPEED_PID_INIT|MOTOR_POS_PID_INIT))
  {
    __DJI_Motor_Ctrl_set_init_state(motor,NON_FORCE);
    return ;
  }

  if(__DJI_Motor_Ctrl_get_init_state(motor,MOTOR_ANGLE_LIMIT_INIT))
  {
    if(motor->max_angle<angle)
      angle=motor->max_angle;
    else if(motor->min_angle>angle)
      angle=motor->min_angle;
  }

  motor->mode=POS_LOOP;
  motor->set_angle=angle;
}

void DJI_Motor_set_speed(DJI_Motor_Ctrl_t* motor, uint16_t speed_rpm)
{
  if(!__DJI_Motor_Ctrl_get_init_state(motor,MOTOR_SPEED_PID_INIT))
  {
    __DJI_Motor_Ctrl_set_init_state(motor,NON_FORCE);
    return ;
  }

  motor->mode=SPEED_LOOP;
  motor->set_speed=speed_rpm;
}

void DJI_Motor_set_current(DJI_Motor_Ctrl_t* motor, int16_t current)
{
  motor->mode=GIVING_CURRENT;
  motor->set_current=current;
}

void DJI_Motor_set_nonforce(DJI_Motor_Ctrl_t* motor)
{
  motor->mode=NON_FORCE;
  motor->set_current=0x00;
}

void DJI_Motor_lockup(DJI_Motor_Ctrl_t* motor)
{
  if(!__DJI_Motor_Ctrl_get_init_state(motor,MOTOR_POS_PID_INIT|MOTOR_SPEED_PID_INIT))
  {
    __DJI_Motor_Ctrl_set_init_state(motor,NON_FORCE);
    return ;
  }

  motor->mode=LOCK;
  motor->set_angle=__DJI_Motor_Ctrl_get_ecd_angle(motor);
}

/**
 * @brief 获取反馈数值
 * @param[in]  motor  电机控制句柄
 * @param[out] torque 力矩/电流
 * @param[out] speed  速度(rpm)
 * @param[out] angle  角度(rad)
 */
void DJI_Motor_get_feedback(DJI_Motor_Ctrl_t* motor,fp32* torque,fp32* speed,fp32* angle)
{
  if(torque!=NULL)
    *torque=__DJI_Motor_Ctrl_get_torque(motor);
  if(speed!=NULL)
    *speed=__DJI_Motor_Ctrl_get_speed(motor);
  if(angle!=NULL)
    *angle=__DJI_Motor_Ctrl_get_angle(motor);
  return;
}

void __DJI_Motor_speed_ctrl_loop(DJI_Motor_Ctrl_t* motor)
{
  if(!__DJI_Motor_Ctrl_get_init_state(motor,MOTOR_SPEED_PID_INIT))
  {
    __DJI_Motor_Ctrl_set_init_state(motor,NON_FORCE);
    return ;
  }


  __DJI_Motor_Ctrl_boundary_ctrl(motor,motor->set_speed);
  __DJI_Motor_Ctrl_write_current(motor,__DJI_Motor_speed_loop_calc(motor));
}

void __DJI_Motor_pos_ctrl_loop(DJI_Motor_Ctrl_t* motor)
{
  if((!__DJI_Motor_Ctrl_get_init_state(motor,MOTOR_SPEED_PID_INIT))||
  (!__DJI_Motor_Ctrl_get_init_state(motor,MOTOR_POS_PID_INIT)))
  {
    __DJI_Motor_Ctrl_set_init_state(motor,NON_FORCE);
    return ;
  }

  __DJI_Motor_Ctrl_write_current(motor,__DJI_Motor_angle_loop_calc(motor));
}

/**
 * @brief 电机电流控制循环
 * 用于在定时器中刷新电流输出
 */
void __DJI_Motor_current_ctrl_loop(DJI_Motor_Ctrl_t* motor)
{
  __DJI_Motor_Ctrl_boundary_ctrl(motor,motor->set_current);
  __DJI_Motor_Ctrl_write_current(motor,motor->set_current);
}

void __DJI_Motor_get_feedback(DJI_Motor_Ctrl_t* motor,uint8_t* rx_msg)
{
  //memcpy((void*)&(motor->recv_pack),rx_msg,sizeof(uint8_t)*8);//报文高低8位方向相反，不能直接memcpy
  // 报文赋值
  motor->recv_pack.ecd=rx_msg[0]<<8;
  motor->recv_pack.ecd|=rx_msg[1];
  motor->recv_pack.speed_rpm=rx_msg[2]<<8;
  motor->recv_pack.speed_rpm|=rx_msg[3];
  motor->recv_pack.torque=rx_msg[4]<<8;
  motor->recv_pack.torque|=rx_msg[5];
  motor->recv_pack.temp=rx_msg[6];

  // 刷新角度值
  motor->last_ecd_angle=motor->ecd_angle;
  motor->ecd_angle=NORMALIZE_TO_2PI(ecd_to_angle(motor->recv_pack.ecd,8191,0,0.0));

  // 反馈反转(如果启用反转)
  /*note: 上电角度可能不是0.0，我也不知道捏...*/
  motor->recv_pack.speed_rpm=__DJI_Motor_Ctrl_get_reverse_current(motor,motor->recv_pack.speed_rpm);
  motor->recv_pack.torque=__DJI_Motor_Ctrl_get_reverse_current(motor,motor->recv_pack.torque);
  motor->ecd_angle=NORMALIZE_TO_2PI(angle_normalize(2*PI-motor->ecd_angle,0.0));

  // 刷新圈数
  if(motor->circle_count_flag)
  {
    if(motor->ecd_angle>(2*PI*2/4)&&motor->last_ecd_angle<(2*PI*1/4))
    {
      motor->circle_count--;
    }
    else if(motor->last_ecd_angle>(2*PI*2/4)&&motor->ecd_angle<(2*PI*1/4))
    {
      motor->circle_count++;
    }
  }
}

static fp32 __DJI_Motor_speed_loop_calc(DJI_Motor_Ctrl_t* motor)
{
  return PID_Calc(&(motor->pid_speed_loop),__DJI_Motor_Ctrl_get_speed(motor),motor->set_speed);
}

static fp32 __DJI_Motor_angle_loop_calc(DJI_Motor_Ctrl_t* motor)
{
  if(motor->circle_count_flag)
  {
    return PID_Calc(&(motor->pid_speed_loop),
      __DJI_Motor_Ctrl_get_speed(motor),
      PID_Calc(
          &(motor->pid_pos_loop),
          __DJI_Motor_Ctrl_get_angle(motor),
          motor->set_angle
      )
    );
  }
  else
  {
    return PID_Calc(&(motor->pid_speed_loop),
      __DJI_Motor_Ctrl_get_speed(motor),
      PID_Calc(
        &(motor->pid_pos_loop),
        0,
        angle_normalize(motor->set_angle,-__DJI_Motor_Ctrl_get_angle(motor))
      )
    );
  }
}

