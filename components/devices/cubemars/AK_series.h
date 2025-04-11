#ifndef AK_SERIES_H
#define AK_SERIES_H

#include "main.h"

typedef FDCAN_HandleTypeDef AK_hcan_t;

typedef enum {
  CAN_PACKET_SET_DUTY = 0, //占空比模式
  CAN_PACKET_SET_CURRENT, //电流环模式
  CAN_PACKET_SET_CURRENT_BRAKE, // 电流刹车模式
  CAN_PACKET_SET_RPM, // 转速模式
  CAN_PACKET_SET_POS, // 位置模式
  CAN_PACKET_SET_ORIGIN_HERE, //设置原点模式
  CAN_PACKET_SET_POS_SPD, //位置速度环模式
} CAN_PACKET_ID;

typedef struct{
  uint16_t id;
  uint16_t mode;

  uint8_t tx_buffer[4];

  float angle;/*automatically flush*/
} AK_Joint_Motor_t;

void AK_joint_motor_init(AK_Joint_Motor_t *motor, uint16_t id);
void AK_joint_motor_pos_speed_ctrl(AK_Joint_Motor_t *motor);
void __AK_joint_motor_ctrl_hook(AK_Joint_Motor_t *motor,AK_hcan_t *can);

#endif

