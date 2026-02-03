#include "AK_series.h"
#include "can_bsp.h"

// 查询手册，本质AK_joint_motor_speed_ctrl 和AK_joint_motor_pos_speed_ctrl是调用电机内置的速度（kp,di)位置（kp,kd）去操控电流环发送电流
// 但由于更换新电机着实不好连接上位机调参，随使用和大疆电机写法过两个pid环输出电机位置(即AK_motor_pid_speed_ctrl和AK_motor_pid_position_ctrl)
//若方便可以连接上位机根据已设置参数转换成上位机数据

/*当前AK70-10的id:93*/
AK_Joint_Motor_t AK70_10_motor = {0};

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

  AK_Motor_Speed_PID_init(motor, PID_POSITION, 0.008, 0, 0, 3000, 1000);
  AK_Motor_Pos_PID_init(motor, PID_POSITION, 0.005, 0, 0, 2000, 1000);
}
void AK_Motor_Pos_PID_init(AK_Joint_Motor_t *motor, enum PID_MODE pid_mode,
                           fp32 Kp, fp32 Ki, fp32 Kd,
                           fp32 max_out, fp32 max_iout)
{
  fp32 pid[3] = {Kp, Ki, Kd};
  PID_Init(&(motor->pid_pos_loop), pid_mode, pid, max_out, max_iout, 0.01, 10);
}
void AK_Motor_Speed_PID_init(AK_Joint_Motor_t *motor, enum PID_MODE pid_mode,
                           fp32 Kp, fp32 Ki, fp32 Kd,
                           fp32 max_out, fp32 max_iout)
{
  fp32 pid[3] = {Kp, Ki, Kd};
  PID_Init(&(motor->pid_speed_loop), pid_mode, pid, max_out, max_iout, 0.01, 10);
}

void AK_joint_motor_enable(AK_Joint_Motor_t * motor)
{
  motor->enable = 1;
}

void AK_joint_motor_disable(AK_Joint_Motor_t * motor)
{
  /*todo:disable的时候需要电机进入失力状态*/
  motor->enable = 0;
}

void AK_motor_pid_speed_ctrl(AK_Joint_Motor_t *motor, float speed)
{
  float current = 0;
  current = PID_Calc(&(motor->pid_speed_loop), motor->spd, speed);
  AK_joint_motor_current_ctrl(motor, current);
}

void AK_motor_pid_position_ctrl(AK_Joint_Motor_t *motor, float angle)
{
  float current = 0;
  float output_speed = 0;
  output_speed = PID_Calc(&(motor->pid_pos_loop), motor->pos, angle);
  current = PID_Calc(&(motor->pid_speed_loop), motor->spd, output_speed);
  AK_joint_motor_current_ctrl(motor, current);
}

void AK_joint_motor_current_ctrl(AK_Joint_Motor_t * motor, float current)
{
  motor->mode = CAN_PACKET_SET_CURRENT;
  int32_t send_index = 0;
  buffer_append_int32((motor->tx_buffer), (int32_t)(current * 1000.0), &send_index);
}

void AK_joint_motor_speed_ctrl(AK_Joint_Motor_t * motor, float rpm)
{
  motor->mode = CAN_PACKET_SET_RPM;
  int32_t send_index = 0;
  buffer_append_int32(motor->tx_buffer, (int32_t)rpm, &send_index);
}

void AK_set_pos(AK_Joint_Motor_t *motor_ptr, float angle)
{
  if (motor_ptr->id == 0x5d)
    AK_joint_motor_pos_speed_ctrl(motor_ptr, angle, 3500,5000);
}

void AK_joint_motor_pos_speed_ctrl(AK_Joint_Motor_t * motor, float pos, float spd, float RPA)
{
  motor->mode = CAN_PACKET_SET_POS_SPD;
  int32_t send_index = 0;
  int16_t send_index1 = 4;
  buffer_append_int32(motor->tx_buffer, (int32_t)(pos * 10000.0), &send_index);
  buffer_append_int16(motor->tx_buffer, spd / 10.0, &send_index1);
  buffer_append_int16(motor->tx_buffer, RPA / 10.0, &send_index1);
}

void AK_joint_motor_nonforce_ctrl(AK_Joint_Motor_t * motor)
{
  AK_joint_motor_current_ctrl(motor, 0.0f);
}

void __AK_joint_motor_feedback_hook(AK_Joint_Motor_t * motor, uint8_t *rx_message)
{
  // void motor_receive(float* motor_pos,float* motor_spd,float* cur,int_8* temp,int_8* error,rx_message)
  // float motor_pos,motor_spd,cur;
  // int8_t temp,error;
  int16_t pos_int = (rx_message[0] << 8) | rx_message[1];
  int16_t spd_int = (rx_message[2] << 8) | rx_message[3];
  int16_t cur_int = (rx_message[4] << 8) | rx_message[5];
  motor->pos = (float)(pos_int * 0.1f);      // 电机位置
  motor->spd = (float)(spd_int * 10.0f);     // 电机速度
  motor->current = (float)(cur_int * 0.01f); // 电机电流
  motor->temp = rx_message[6];               // 电机温度
  motor->error = rx_message[7];              // 电机故障码
}

void AK_joint_motor_set_zero_pos(AK_Joint_Motor_t * motor)
{
  // int32_t send_index = 0;
  // uint8_t buffer;
  motor->mode = CAN_PACKET_SET_ORIGIN_HERE;
  motor->tx_buffer[0] = 0x00;
  // comm_can_transmit_eid(controller_id |((uint32_t) CAN_PACKET_SET_ORIGIN_HERE << 8), &buffer, send_index);
}

// 实测并没有掉电不丢失位置作用，效果和AK_joint_motor_set_zero_pos一样
void AK_joint_motor_set_forever_zero_pos(AK_Joint_Motor_t *motor)
{

  motor->mode = CAN_PACKET_SET_ORIGIN_HERE;
  motor->tx_buffer[0] = 0x01;
}

void __AK_joint_motor_ctrl_hook(AK_Joint_Motor_t * motor, AK_hcan_t * can)
{
  static uint32_t data_len;
  if (motor->enable)
  {
    switch (motor->mode)
    {
    case CAN_PACKET_SET_ORIGIN_HERE:
      data_len=CAN_DATA_SIZE_1_BYTES;
      break;
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


//这个电机油大饼啊，各种非人设计，设置永久零点为什么零点不在当前位置,上电后后位置也是混乱的，这根本就不是双编码电机
//调试该电机请认真阅读书册，最好拆下来尝试
//上位机写参数前一定要读参数