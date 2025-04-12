#include "AK_series.h"
#include "can_bsp.h"
/*当前AK70-10的id:93*/

AK_Joint_Motor_t AK70_10_motor={0};

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
  motor->enable=0;
  AK_joint_motor_nonforce_ctrl(motor);
}

void AK_joint_motor_enable(AK_Joint_Motor_t *motor)
{
  motor->enable=1;
}

void AK_joint_motor_disable(AK_Joint_Motor_t *motor)
{
  /*todo:disable的时候需要电机进入失力状态*/
  motor->enable=0;
}

void AK_joint_motor_current_ctrl(AK_Joint_Motor_t *motor,float current)
{
  motor->mode=CAN_PACKET_SET_CURRENT;
  int32_t send_index = 0;
  buffer_append_int32((motor->tx_buffer), (int32_t)(current * 1000.0), &send_index);
}

void AK_joint_motor_speed_ctrl(AK_Joint_Motor_t *motor,float rpm)
{
  motor->mode=CAN_PACKET_SET_RPM;
  int32_t send_index = 0;
  buffer_append_int32(motor->tx_buffer, (int32_t)rpm, &send_index);
}

void AK_joint_motor_pos_speed_ctrl(AK_Joint_Motor_t *motor,float pos,float spd,float RPA)
{
  motor->mode=CAN_PACKET_SET_POS_SPD;
  int32_t send_index = 0;
  int16_t send_index1 = 4;
  buffer_append_int32(motor->tx_buffer, (int32_t)(pos * 10000.0), &send_index);
  buffer_append_int16(motor->tx_buffer,spd/10.0, & send_index1);
  buffer_append_int16(motor->tx_buffer,RPA/10.0, & send_index1);
}

void AK_joint_motor_nonforce_ctrl(AK_Joint_Motor_t *motor)
{
  AK_joint_motor_current_ctrl(motor,0.0f);
}

void __AK_joint_motor_feedback_hook(AK_Joint_Motor_t *motor)
{

}

void __AK_joint_motor_ctrl_hook(AK_Joint_Motor_t *motor,AK_hcan_t *can)
{
  static uint32_t data_len;
  if(motor->enable)
  {
    switch(motor->mode)
    {
      case CAN_PACKET_SET_CURRENT:
      case CAN_PACKET_SET_RPM:
        data_len=CAN_DATA_SIZE_4_BYTES;
        break;
      case CAN_PACKET_SET_POS_SPD:
        data_len=CAN_DATA_SIZE_8_BYTES;
        break;
      default:
        AK_joint_motor_nonforce_ctrl(motor);
        data_len=CAN_DATA_SIZE_4_BYTES;
    }
    fdcanx_send_data_ex_mode(can
      ,(motor->id)|((uint32_t)(motor->mode) << 8)
      ,motor->tx_buffer,data_len);
  }
}
