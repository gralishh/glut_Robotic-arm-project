#include "DJI_motor.h"

#include "CAN_receive.h"
#include "angle_process.h"
#include <string.h>

// 电流输出数组宏函数
#define IS_OUTPUT_ID_200H(id) ((id>=0x201)&&(id<=0x204))
#define IS_OUTPUT_ID_1FFH(id) ((id>=0x205)&&(id<=0x208))
#define GET_OUTPUT_CURRENT_INDEX(id) ((id-0x200)%4)

// CANBus宏函数
#define __DJI_CANBus_get_motor_instance(bus_ptr,index) ((bus_ptr->mounted_motor)[index])
#define __DJI_CANBus_get_motor_count(bus_ptr) (bus_ptr->mounted_motor_count)

// Motor_Ctrl宏函数
#define __DJI_Motor_Ctrl_get_reverse_current(motor_ptr,current) (motor_ptr->reverse_flag?-current:current)
#define __DJI_Motor_Ctrl_get_speed(motor_ptr) ((motor_ptr->recv_pack).speed_rpm)
#define __DJI_Motor_Ctrl_get_ecd(motor_ptr) ((motor_ptr->recv_pack).ecd)
#define __DJI_Motor_Ctrl_get_ecd_angle(motor_ptr) (motor_ptr->ecd_angle)
//ecd_to_angle(__DJI_Motor_Ctrl_get_ecd(motor_ptr),8191,0,0) 
#define __DJI_Motor_Ctrl_get_angle(motor_ptr) ((motor_ptr->circle_count)*PI+&ref_ptr)
#define __DJI_Motor_Ctrl_get_init_state(motor_ptr,state_flag) ((motor_ptr->init_state)&(0x01<<state_flag))
#define __DJI_Motor_Ctrl_set_init_state(motor_ptr,state_flag) ((motor_ptr->init_state)|=(0x01<<state_flag))

#define __DJI_Motor_Ctrl_write_current(motor_ptr,current) \
  if(IS_OUTPUT_ID_200H(motor_ptr->id)) \
    (motor_ptr->mounted_bus->output_current200H)[GET_OUTPUT_CURRENT_INDEX(motor_ptr->id)]=__DJI_Motor_Ctrl_get_reverse_current(motor_ptr,current); \
  if(IS_OUTPUT_ID_1FFH(motor_ptr->id)) \
    (motor_ptr->mounted_bus->output_current1FFH)[GET_OUTPUT_CURRENT_INDEX(motor_ptr->id)]=__DJI_Motor_Ctrl_get_reverse_current(motor_ptr,current); \


/**
 * @brief CANBus初始化
 */
void DJI_CANBus_init(DJI_Motor_Bus_t* bus,FDCAN_HandleTypeDef* can)
{
  bus->can=can;
}

void DJI_CANBus_config_init(DJI_Motor_Bus_t* bus, DJI_Motor_Config_t* config)
{

}

void DJI_CANBus_add_motor(DJI_Motor_Bus_t* bus,DJI_Motor_Ctrl_t* motor)
{
  if(__DJI_CANBus_get_motor_count(bus)>6)
    return;
  __DJI_CANBus_get_motor_instance(bus,__DJI_CANBus_get_motor_count(bus))=motor;

  motor->mounted_bus = bus;
  bus->mounted_motor_count++;
}

/**
 * @brief 总线电机控制循环
 * 执行pid计算和电流发送的操作
 */
void __DJI_CANBus_ctrl_loop(DJI_Motor_Bus_t* bus)
{
  static uint16_t index;

  for(index=0;index<__DJI_CANBus_get_motor_count(bus);index++)
  {
    DJI_Motor_Ctrl_t* motor = __DJI_CANBus_get_motor_instance(bus,index);
    switch(motor->mode)
    {
      case SPEED_LOOP:
        DJI_Motor_speed_ctrl_loop(motor);
        break;

      case LOCK:
      case POS_LOOP:
        DJI_Motor_pos_ctrl_loop(motor);

      default:
      case NON_FORCE:
        DJI_Motor_set_current(motor, 0);
      case GIVING_CURRENT:
        DJI_Motor_current_ctrl_loop(__DJI_CANBus_get_motor_instance(bus,index));
      break;
    }
  }
}

int8_t __DJI_CANBus_feedback_update(DJI_Motor_Bus_t* bus, uint8_t* rx_data, uint16_t rx_id)
{
  static uint16_t index;

  for(index=0;index<__DJI_CANBus_get_motor_count(bus);index++)
  {
    if(rx_id==__DJI_CANBus_get_motor_instance(bus,index)->id)
    {
      //memcpy(&(__DJI_CANBus_get_motor_instance(bus,index)->recv_pack),rx_data,sizeof(uint8_t)*8);
      DJI_Motor_get_feedback(__DJI_CANBus_get_motor_instance(bus,index),rx_data);
      return 1;
    }
  }
  return 0;
}

void DJI_Motor_init(DJI_Motor_Ctrl_t* motor,DJI_Motor_Bus_t* bus,Motor_Type_e motor_type,uint16_t id)
{
  motor->type=motor_type;
  motor->id=id;
  motor->mounted_bus=bus;
  motor->ref_ptr=&(motor->ecd_angle);
}

void DJI_Motor_set_angle_feedback(DJI_Motor_Ctrl_t* motor,fp32* feedback_angle)
{
  motor->ref_ptr=feedback_angle;
  __DJI_Motor_Ctrl_set_init_state(motor,ENXTERN_ENCODER_INIT);
}

void DJI_Motor_Pos_PID_init(DJI_Motor_Ctrl_t* motor,enum PID_MODE pid_mode,
  fp32 Kp,fp32 Ki,fp32 Kd,
  fp32 max_out,fp32 max_iout)
{
  fp32 pid[3]={Kp,Ki,Kd};
  PID_Init(&(motor->pid_pos_loop),pid_mode,pid,max_out,max_iout,0.01,10);
  __DJI_Motor_Ctrl_set_init_state(motor,MOTOR_POS_PID_INIT);
}

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
  if((!__DJI_Motor_Ctrl_get_init_state(motor,MOTOR_SPEED_PID_INIT))||
  (!__DJI_Motor_Ctrl_get_init_state(motor,MOTOR_POS_PID_INIT)))
  {
    __DJI_Motor_Ctrl_set_init_state(motor,NON_FORCE);
    return ;
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

void DJI_Motor_lockup(DJI_Motor_Ctrl_t* motor)
{
  if((!__DJI_Motor_Ctrl_get_init_state(motor,MOTOR_SPEED_PID_INIT))||
  (!__DJI_Motor_Ctrl_get_init_state(motor,MOTOR_POS_PID_INIT)))
  {
    __DJI_Motor_Ctrl_set_init_state(motor,NON_FORCE);
    return ;
  }

  motor->mode=LOCK;
  motor->set_angle=__DJI_Motor_Ctrl_get_ecd_angle(motor);
}

void __DJI_Motor_speed_ctrl_loop(DJI_Motor_Ctrl_t* motor)
{
  if(!__DJI_Motor_Ctrl_get_init_state(motor,MOTOR_SPEED_PID_INIT))
  {
    __DJI_Motor_Ctrl_set_init_state(motor,NON_FORCE);
    return ;
  }

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

  __DJI_Motor_Ctrl_write_current(motor,__DJI_Motor_pos_loop_calc(motor));
}

/**
 * @brief 电机电流控制循环
 * 用于在定时器中刷新电流输出
 */
void __DJI_Motor_current_ctrl_loop(DJI_Motor_Ctrl_t* motor)
{
  __DJI_Motor_Ctrl_write_current(motor,motor->set_current);
}

void __DJI_Motor_get_feedback(DJI_Motor_Ctrl_t* motor,uint8_t* rx_msg)
{
  memcpy((motor->recv_pack),rx_msg,sizeof(uint8_t)*8);
}

static fp32 __DJI_Motor_speed_loop_calc(DJI_Motor_Ctrl_t* motor)
{
  return PID_Calc(&(motor->pid_speed_loop),__DJI_Motor_Ctrl_get_speed(motor),motor->set_speed);
}

static fp32 __DJI_Motor_angle_loop_calc(DJI_Motor_Ctrl_t* motor)
{
  return PID_Calc(&(motor->pid_speed_loop),
    __DJI_Motor_Ctrl_get_speed(motor),
    PID_Calc(&(motor->pid_pos_loop),*(motor->ref_ptr),motor->set_pos))
}

