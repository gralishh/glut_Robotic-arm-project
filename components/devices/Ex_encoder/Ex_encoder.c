#include "Ex_encoder.h"
#include "Ex_ecd_protocol.h"
#include "CAN_receive.h"
#include "angle_process.h"
#include "struct_typedef.h"
#include <stdlib.h>
#include <string.h>

//delta=901,L=901, H=14A0

External_ecd_bus_handler_t Ex_ecd_bus={0};
External_ecd_handler_t External_ecd_handler = {0};

OID_ECD_FEEDBACK_T oid_ecd_feedback;
//extern OID_ECD_FEEDBACK_T oid_ecd_feedback;
//OID_ECD_CMD_T message;
void External_ecd_bus_init(void)
{
  /*预留*/
}

void External_ecd_init_on_can(External_ecd_handler_t* ecd,uint32_t max_ecd,ecd_type_e type,uint16_t device_id,FDCAN_HandleTypeDef* can,uint16_t bus_id)
{
  ecd->type=type;
  Ex_ecd_bus.can_communication_ecd[Ex_ecd_bus.can_ecd_count]=ecd;
  Ex_ecd_bus.can_ecd_count++;
  ecd->protocol_type=ECD_CAN_COMMUNICATION;
  ecd->device_id=device_id;

  ecd->can=can;
  ecd->bus_id=bus_id;
  ecd->max_ecd=max_ecd;
}

uint32_t External_ecd_get_value(External_ecd_handler_t* ecd)
{
  return ecd->ecd;
}

fp32 External_ecd_get_angle(External_ecd_handler_t* ecd)
{
  return ecd->angle;
} 

fp32* External_ecd_get_angle_pointer(External_ecd_handler_t* ecd)
{
  return &(ecd->angle);
}

///**
// * @brief oid编码器设置id
// * @note 不能正常工作，也不需要
// */
//void External_ecd_oid_set_id(FDCAN_HandleTypeDef *CANx, uint16_t oid_id, uint16_t new_id)
//{
//  OID_ECD_CMD_T message;
//  message.data_length = 0x04;        // 数据长度固定为 4
//  message.encoder_addr = oid_id; // 当前编码器地址
//  message.command_code = 0x02;       // 指令码为 0x02，表示修改ID
//  message.set_data = new_id;         // 新的编码器地址
//  memset(message.reserved, 0x00, sizeof(message.reserved)); // 填充保留字节为 0

//  CanSendMsg(CANx,0x01,(uint8_t*)(void*)&message);
//}

void External_ecd_oid_set_mode(FDCAN_HandleTypeDef *CANx, uint16_t oid_id, uint16_t commmand, uint16_t set_data, uint16_t set_other_ata)
{
  OID_ECD_CMD_T message;
  message.data_length = 0x04;                               // 数据长度固定为 4
  message.encoder_addr = oid_id;                            // 当前编码器地址
  message.command_code = commmand;                          // 指令码
  message.set_data = set_data;                              // 设置的数据
  memset(message.reserved, set_other_ata, sizeof(message.reserved)); // 查阅手册，一般为0x00，如果要加请更改数据长度
  CanSendMsg(CANx, 0x01, (uint8_t *)(void *)&message);
}

void __External_ecd_can_feedback_hook(FDCAN_HandleTypeDef *CANX, uint16_t id, uint8_t *msg)
{
  uint16_t index;
  External_ecd_handler_t* ecd;
  //oid_ecd_feedback = (OID_ECD_FEEDBACK_T *)(void *)&msg;
  memcpy(&oid_ecd_feedback, msg, sizeof(OID_ECD_FEEDBACK_T));//测试用的
  for (index = 0; index < Ex_ecd_bus.can_ecd_count; index++)
  {
    ecd=(Ex_ecd_bus.can_communication_ecd)[index];
    if(ecd->can!=CANX||ecd->bus_id!=id||
      ((OID_ECD_FEEDBACK_T*)(void*)msg)->id!=ecd->device_id
    )
      continue;

	  //可添加其他编码器
    switch(ecd->type)
    {
      case OID_ECD:
        ecd->ecd=((OID_ECD_FEEDBACK_T*)(void*)msg)->ecd;
        ecd->angle=NORMALIZE_TO_2PI(ecd_to_angle(ecd->ecd,ecd->max_ecd,0,0.0));
        break;
      default:
        ;
    }
  }
}

