#include "Vofa.h"
#include <stdarg.h>
#include <stdio.h>
#include "bsp_usart.h"
#include "vofa_uart_task.h"
#include "general_motor_module.h"
#include "cmsis_os2.h"
#include "gimbal_task.h"
#include "hand_task.h"
#include "Ex_encoder.h"
#include "chassis_task.h"

extern External_ecd_handler_t uplift_ecd;
extern DJI_Motor_Ctrl_t DJI_Motor_uplift;
extern fp32 vofa_output_speed;
fp32 vofa_send_data_pack[VOFA_FEEDBACK_COUNT];
RX_pack Rece_pack;

VOFA_TASK_HANDKER_TYPE vofa_para = {0};
VOFA_TASK_HANDKER_TYPE *vofa_para_ptr = &vofa_para;

void vofa_rx_unpack(void)
{
    // UART7_Recv((void *)&Rece_pack, 2 * sizeof(uint8_t));
    // if (Rece_pack.char_check[0] == 0xAA && Rece_pack.char_check[1] == 0xEE)
    //  UART7_Recv((void *)&((&Rece_pack)->char_type), sizeof(uint8_t) + sizeof(float));

    static uint32_t uart7_length = 0;

    uart7_length = UART7_GetDataCount();

    // UART7_Recv((void *)&Rece_pack, 2 * sizeof(uint8_t));// 先接收前2字节检查帧头

    if (UART7_At(0) == 0xAA && UART7_At(1) == 0xEE)
    {
        if (uart7_length >= (3 * sizeof(uint8_t) + sizeof(float)))
        {
            UART7_Recv((void *)&Rece_pack, 3 * sizeof(uint8_t) + sizeof(float)); // 接收剩余的数据
        }
    }
    else if (uart7_length > 100)
    {
        UART7_Drop(uart7_length);
    }
    else
    {
        UART7_Drop(1);
    }
}

//  关于接收
void vofa_rece_set_type(VOFA_TASK_HANDKER_TYPE *handler, RX_pack *Rece_pack_ptr)
{
    switch (Rece_pack_ptr->char_type)
    {
    case 0x01:
        handler->vofa_motor_en |= MOTOR_START; // 启动电机
        break;
    case 0x10:
        handler->vofa_motor_en |= MOTOR_OFF; // 关闭电机
        break;
    case 0x20:
        handler->vofa_motor_en |= MOTOR_SPEED_LOOP_EN;  // 速度环使能
        handler->vofa_motor_en |= MOTOR_POS_LOOP_DISEN; // 位置环失能
        break;
    case 0x02:
        handler->vofa_motor_en |= MOTOR_SPEED_LOOP_DISEN; // 速度环失能
        break;
    case 0x30:
        handler->vofa_motor_en |= MOTOR_POS_LOOP_EN;      // 位置环使能
        handler->vofa_motor_en |= MOTOR_SPEED_LOOP_DISEN; // 速度环失能
        break;
    case 0x03:
        handler->vofa_motor_en |= MOTOR_POS_LOOP_DISEN; // 位置环失能
        break;
    case 0x40:
        handler->vofa_motor_en |= MOTOR_GET_SPEED; // 调速度
        handler->speed_ref = Rece_pack_ptr->float_data;
        break;
    case 0x50:
        handler->vofa_motor_en |= MOTOR_GET_POS; // 调位置
        handler->pos_ref = Rece_pack_ptr->float_data;
        break;
    case 0x60:
        handler->vofa_motor_en |= MOTOR_GET_SPEED_Kp; // 速度Kp
        handler->speed_Kp = Rece_pack_ptr->float_data;
        break;
    case 0x70:
        handler->vofa_motor_en |= MOTOR_GET_SPEED_Ki; // 速度Ki
        handler->speed_Ki = Rece_pack_ptr->float_data;
        break;
    case 0x80:
        handler->vofa_motor_en |= MOTOR_GET_POS_Kp; // 位置Kp
        handler->pos_Kp = Rece_pack_ptr->float_data;
        break;
    case 0x90:
        handler->vofa_motor_en |= MOTOR_GET_POS_Kd; // 位置Pd
        handler->pos_Kd = Rece_pack_ptr->float_data;
        break;
    default:
        break;
    }
}

// 加点保护
void vofa_type_set_motor_HOOK(VOFA_TASK_HANDKER_TYPE *handler, Joint_Motor_t *motor_handler, uint8_t motor)
{
    if (handler->vofa_motor_en & MOTOR_START)
    {
        if (handler->vofa_motor_en & MOTOR_SPEED_LOOP_EN)
        {
            handler->vofa_motor_en |= MOTOR_POS_LOOP_DISEN; // 位置环失能

            if (handler->vofa_motor_en & MOTOR_GET_SPEED)
                hand_task_handler_ptr->motor_speed[motor] = handler->speed_ref;
            if (handler->vofa_motor_en & MOTOR_GET_SPEED_Kp)
                motor_handler->Kp = handler->speed_Kp;
            // if(handler->vofa_motor_en &= MOTOR_GET_SPEED_Ki)
            // motor_handler->ki = handler->speed_Ki;
        }
        else if (handler->vofa_motor_en & MOTOR_SPEED_LOOP_DISEN)
        {
            hand_task_handler_ptr->motor_ctrl_mode[motor] = NON_FORCE;
            // motor_handler->Kp = 0.0001f;
            // motor_handler->Ki = 0.00001f;
        }

        if (handler->vofa_motor_en & MOTOR_POS_LOOP_EN)
        {
            handler->vofa_motor_en |= MOTOR_SPEED_LOOP_DISEN; // 速度环失能
            if (handler->vofa_motor_en & MOTOR_GET_POS)
                hand_task_handler_ptr->motor_angle[motor] = handler->pos_ref;
            if (handler->vofa_motor_en & MOTOR_GET_POS_Kp)
                motor_handler->Kp = handler->pos_Kp;
            if (handler->vofa_motor_en & MOTOR_GET_POS_Kd)
                motor_handler->Kd = handler->pos_Kd;
        }
        else if (handler->vofa_motor_en & MOTOR_POS_LOOP_DISEN)
        {
            hand_task_handler_ptr->motor_ctrl_mode[motor] = NON_FORCE;
            motor_handler->Kp = 0.0001f;
            motor_handler->Kd = 0.00001f;
        }
    }
    else if (handler->vofa_motor_en & MOTOR_OFF)
    {
        // 电机失能
        hand_task_handler_ptr->motor_ctrl_mode[motor] = NON_FORCE;
    }
}

// 关于发送
void vofa_data_into_pack(fp32 *vofa_send_data_pack)
{
    //    uint8_t i;
    //
    //    for (i = 0; i < HAND_MOTOR_COUNT - 1; i++)
    //    {
    //        vofa_send_data_pack[i] = hand_task_handler.motor_angle[i];
    //        vofa_send_data_pack[i + HAND_MOTOR_COUNT-1] = hand_task_handler.feedback_motor_angle[i];
    //    }

    // vofa_send_data_pack[0] = chassis_task_handler.motor_speed[0];
    // vofa_send_data_pack[1] = chassis_task_handler.feedback_motor_speed[0];
    //显示抬升数据
   vofa_send_data_pack[0] = vofa_output_speed;
   vofa_send_data_pack[1] = (DJI_Motor_uplift.recv_pack).speed_rpm;
//    vofa_send_data_pack[2] = DJI_Motor_uplift.set_angle;
//    vofa_send_data_pack[3] = (DJI_Motor_uplift.circle_count) * PI * 2 + (DJI_Motor_uplift.ecd_angle);
   vofa_send_data_pack[2] = uplift_ecd.process_ecd;
   vofa_send_data_pack[3] = gimbal_task_handler.oid_length;
}

void vofa_uart_task(void *argument)
{
    Vofa_Init(&vofa_handler, VOFA_MODE_SKIP);
    while (1)
    {
        // vofa接收调用
        // vofa_rx_unpack();
        // vofa_rece_set_type(vofa_para_ptr, &Rece_pack);
        // vofa_type_set_motor_HOOK(vofa_para_ptr, &DM_Motor_J2, DM_J2);
        // vofa发送调用
        vofa_data_into_pack(vofa_send_data_pack);
        Vofa_JustFloat(&vofa_handler, vofa_send_data_pack, VOFA_FEEDBACK_COUNT);
        osDelay(1);
    }
}
