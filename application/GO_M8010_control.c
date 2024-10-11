#include "GO_M8010_control.h"
#include "string.h"
#include "stdbool.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
//#include "task.h"
//#include "gimbal_task.h"
#include "bsp_usart.h"
//#include "referee.h" //大抵是图形反馈
#include "core_cm7.h"
#include "motor_control.h"
#include "crc_ccitt.h"
#include "main.h"

extern DMA_HandleTypeDef hdma_usart6_tx;

#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 

/**
 * @brief 将发送结构体数据封装成数据包，得到直接用于发送的数据
 */
int modify_data(MOTOR_send *motor_s)
{
    motor_s->hex_len = 17;
    motor_s->motor_send_data.head[0] = 0xFE;
    motor_s->motor_send_data.head[1] = 0xEE;
	
//		SATURATE(motor_s->id,   0,    15);
//		SATURATE(motor_s->mode, 0,    7);
		SATURATE(motor_s->GM_Send_Kp_Pos,  0.0f,   25.599f);
		SATURATE(motor_s->GM_Send_Kd_Speed,  0.0f,   25.599f);
		SATURATE(motor_s->GM_Send_Effort,   -127.99f,  127.99f);
		SATURATE(motor_s->GM_Send_speed,   -804.00f,  804.00f);
		SATURATE(motor_s->GM_Send_Pos, -411774.0f,  411774.0f);

    motor_s->motor_send_data.mode.id   = motor_s->id;
    motor_s->motor_send_data.mode.status  = motor_s->mode;
    motor_s->motor_send_data.comd.k_pos  = motor_s->GM_Send_Kp_Pos/25.6f*32768;
    motor_s->motor_send_data.comd.k_spd  = motor_s->GM_Send_Kd_Speed/25.6f*32768;
    motor_s->motor_send_data.comd.pos_des  = motor_s->GM_Send_Pos/6.2832f*32768;
    motor_s->motor_send_data.comd.spd_des  = motor_s->GM_Send_speed/6.2832f*256;
    motor_s->motor_send_data.comd.tor_des  = motor_s->GM_Send_Effort*256;
    motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);
    return 0;
}

/**
 * @brief 1号电机数据发送
 * @param pData 发送数据句柄(或对象)指针
 * @param rData 接收数据句柄(或对象)指针，用于判断pData中的id是否正确
 * @note 请在描述性文件或注释中表明对应的电机,当前使用USART2
 */
HAL_StatusTypeDef SERVO1_RS485_Send(MOTOR_send *pData , MOTOR_recv* rData)
{
    //发送错误电机id或接收发送错误
    if(rData->motor_id != pData->id)
    {
        pData->mode = 0;
    }
    modify_data(pData);
	  USART2_Send((uint8_t *)pData,sizeof(pData->motor_send_data));
    return HAL_OK;
}

/**
 * @brief 2号电机数据发送
 * @param pData 发送数据句柄(或对象)指针
 * @param rData 接收数据句柄(或对象)指针，用于判断pData中的id是否正确
 * @note 请在描述性文件或注释中表明对应的电机,当前使用USART3
 */
HAL_StatusTypeDef SERVO2_RS485_Send(MOTOR_send *pData , MOTOR_recv* rData)
{
    if(rData->motor_id != pData->id)
    {
        pData->mode = 0;
    }
    modify_data(pData);
	  USART3_Send((uint8_t *)pData,sizeof(pData->motor_send_data));
    return HAL_OK;
}

/**
 * @brief 解包数据函数，负责解包数据并存在结构体中
 */
int extract_data(MOTOR_recv *motor_r)
{
    if(motor_r->motor_recv_data.CRC16 !=
        crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, 14)){
        // printf("[WARNING] Receive data CRC error");
        motor_r->correct = 0;
        return motor_r->correct;
    }
    else
		{
        motor_r->motor_id = motor_r->motor_recv_data.mode.id;
        motor_r->mode = motor_r->motor_recv_data.mode.status;
        motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
        motor_r->MError = motor_r->motor_recv_data.fbk.MError;
        motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed/256)*6.2832f ;
        motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256;
        motor_r->Pos = 6.2832f*((float)motor_r->motor_recv_data.fbk.pos) / 32768;
		    motor_r->footForce = motor_r->motor_recv_data.fbk.force;
		    motor_r->correct = 1;
        return motor_r->correct;
    }
}

/**
 * @brief 电机数据接收函数
 * @param rData 接收数据句柄(或对象)指针
 * @note 该函数并不实现接收数据的功能，只负责解包数据和更新状态
 */
MOTOR_recv* SERVO_Recv(MOTOR_recv *rData)
{

	uint8_t *rp = (uint8_t *)&rData->motor_recv_data;
  /*判断帧头*/
  if(rp[0] == 0xFD && rp[1] == 0xEE)
  {
  	rData->correct = 1;
  	extract_data(rData);
    return rData;
  }
  
  return rData;
}

