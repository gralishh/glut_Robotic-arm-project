#include "M8010_motor.h"

#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 

#define M8010_RECV() \
    next_usart2:\
    usart2_length = USART2_GetDataCount();  \
    if(usart2_length >= 10)\
    {\
        if(USART2_At(0) == 0xFD && USART2_At(1) == 0xEE)  \
        {\
            USART2_Recv(Transmission_usart2, FEEDBACK_DATA_SIZE);  \
            yaw_5_motor_rx = *SERVO_Recv((MOTOR_recv *)Transmission_usart2);\
            USART2_Drop(4096);\
        }\
        else\
        {\
            USART2_Drop(1);\
            vTaskDelay(1);\
            usart2_length = USART2_GetDataCount();\
            if(usart2_length > 3000)\
            {\
                USART2_Drop(4096);\
            }\
            goto next_usart2;\
        }\
    }\
    else\
    {\
        USART2_Drop(1);\
        for(;;)\
        {\
            usart2_length =  USART2_GetDataCount();\
            if( usart2_length > 0)\
            {\
                if(USART2_At(0) == 0xFD && USART2_At(1) == 0xEE)  \
                {\
                    break;\
                }\
                else\
                {\
                    USART2_Drop(1);\
                    vTaskDelay(1);\
                }\
                 usart2_length =  USART2_GetDataCount();\
                if(usart2_length > 3000)\
                {\
                    USART2_Drop(4096);\
                    break;\
                }\
            }\
            else\
            {\
                break;\
            }\
        }\
    }\


void M8010_motor_init(M8010_motor_t motor,uint8_t id,float Kp,float Kd)
{
  motor->send_data.id=id;
  motor->send_data.GM_Send_Kp_Pos=Kp;
  motor->send_data.GM_Send_Kd_Speed=Kd;
}

void M8010_motor_change_param(M8010_motor_t motor,float Kp,float Kd)
{
  motor->send_data.GM_Send_Kp_Pos=Kp;
  motor->send_data.GM_Send_Kd_Speed=Kd;
}

void M8010_motor_lock(M8010_motor_t motor)
{

}

void M8010_motor_set_output(M8010_motor_t motor,float speed,float pos,float torque)
{

}

MOTOR_recv* SERVO_Recv(M8010_motor_t motor)
{

}

HAL_StatusTypeDef SERVO_Send(M8010_motor_t motor)
{

}

/**
 * @brief 将发送结构体数据封装成数据包，得到直接用于发送的数据
 */
int __modify_data(MOTOR_send *motor_s)
{
    motor_s->hex_len = 17;
    motor_s->motor_send_data.head[0] = 0xFE;
    motor_s->motor_send_data.head[1] = 0xEE;
	
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
 * @brief 解包数据函数，负责解包数据并存在结构体中
 */
int __extract_data(MOTOR_recv *motor_r)
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
