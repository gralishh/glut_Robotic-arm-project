#include "DJI_motor_canbus.h"

// CANBus宏函数
#define __DJI_CANBus_get_motor_instance(bus_ptr,index) ((bus_ptr->mounted_motor)[index])
#define __DJI_CANBus_get_motor_count(bus_ptr) (bus_ptr->mounted_motor_count)

/**
 * @brief CANBus初始化
 */
void DJI_CANBus_init(DJI_Motor_Bus_t* bus,FDCAN_HandleTypeDef* can)
{
  bus->can=can;
  bus->mounted_motor_count=0;
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
        __DJI_Motor_speed_ctrl_loop(motor);
        break;

      case LOCK:
      case POS_LOOP:
        __DJI_Motor_pos_ctrl_loop(motor);

      default:
      case NON_FORCE:
      case GIVING_CURRENT:
        __DJI_Motor_current_ctrl_loop(__DJI_CANBus_get_motor_instance(bus,index));
      break;
    }
  }
}

/**
 * @brief CAN总线数据更新钩子
 */
int8_t __DJI_CANBus_feedback_update(DJI_Motor_Bus_t* bus, uint8_t* rx_data, uint16_t rx_id)
{
  static uint16_t index;

  for(index=0;index<__DJI_CANBus_get_motor_count(bus);index++)
  {
    if(rx_id==__DJI_CANBus_get_motor_instance(bus,index)->id)
    {
      //memcpy(&(__DJI_CANBus_get_motor_instance(bus,index)->recv_pack),rx_data,sizeof(uint8_t)*8);
      __DJI_Motor_get_feedback(__DJI_CANBus_get_motor_instance(bus,index),rx_data);
      return 1;
    }
  }
  return 0;
}
