#include "AK_series.h"
#include "can_bsp.h"

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void AK_joint_motor_init(AK_Joint_Motor_t *motor, uint16_t id)
{
  motor->id=id;
  AK_joint_motor_nonforce_ctrl(motor);
}

void AK_joint_motor_current_ctrl(AK_Joint_Motor_t *motor,float current)
{
  motor->mode=CAN_PACKET_SET_CURRENT;
}

void AK_joint_motor_pos_speed_ctrl(AK_Joint_Motor_t *motor)
{
  motor->mode=CAN_PACKET_SET_POS_SPD;
}

void AK_joint_motor_nonforce_ctrl(AK_Joint_Motor_t *motor)
{

}

void __AK_joint_motor_ctrl_hook(AK_Joint_Motor_t *motor,AK_hcan_t *can)
{

}
