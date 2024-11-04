#ifndef __GO_M8010_CONTROL_H
#define __GO_M8010_CONTROL_H

#include "ris_protocol.h"
#include "motor_control.h"
#include "crc_ccitt.h"

/**
 * yaw_5_motor  SERVO1 USART2
 * roll_3_motor SERVO2 USART3
 */

const MOTOR_recv *get_hand_roll_3_motor_rx_point(void);
const MOTOR_recv *get_hand_yaw_5_motor_rx_point(void);


int modify_data(MOTOR_send *motor_s);
HAL_StatusTypeDef SERVO1_RS485_Send(MOTOR_send *pData , MOTOR_recv* rData);
HAL_StatusTypeDef SERVO2_RS485_Send(MOTOR_send *pData , MOTOR_recv* rData);

int extract_data(MOTOR_recv *motor_r);
MOTOR_recv* SERVO_Recv(MOTOR_recv *rData);





#endif 

