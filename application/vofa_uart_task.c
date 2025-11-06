#include "Vofa.h"
#include <stdarg.h>
#include <stdio.h>
#include "bsp_usart.h"
#include "vofa_uart_task.h"
#include "general_motor_module.h"
#include "cmsis_os2.h"

#include "hand_task.h"
#include "hand_task_interface.h"
#include "chassis_task.h"

fp32 vofa_send_data_pack[VOFA_FEEDBACK_COUNT];
// unsigned char vofa_buffer[8];
Resolve_Typedef Rece_aray[2];

VOFA_TASK_HANDKER_TYPE vofa_para = {0};
VOFA_TASK_HANDKER_TYPE *vofa_para_ptr = &vofa_para;

// void vofa_buffer_turn_rx_aray(void)
//{
//	int para,index;
//	for(index = 0; index < 2; index++)
//	{
//		for(para = 0; para < 4; para++)
//		{
//			Rece_aray[index].char_table[para] = vofa_buffer[para];
//		}
//	}
// }
//  关于接收
void vofa_rece_unpack(VOFA_TASK_HANDKER_TYPE *handler, uint8_t motor)
{

    if ((Rece_aray[0].char_table[0] == 0xAA) && (Rece_aray[0].char_table[1] == 0xEE))
    {

        if (Rece_aray[1].char_table[0] == 1) // 启动电机
        {

            if (Rece_aray[0].char_table[2] == 0x01) // 速度环使能
            {

                if (Rece_aray[0].char_table[3] == 0x01) // 调速指令
                {
                    handler->speed_ref = Rece_aray[1].float_data;
                    // hand_task_handler_ptr->motor_speed[motor] = handler->speed_ref;
                }
                else if (Rece_aray[0].char_table[3] == 0x02) // 速度环Kp
                {
                    handler->speed_Kp = Rece_aray[1].float_data;
                }
                else if (Rece_aray[0].char_table[3] == 0x03) // 速度环Ki
                {
                    handler->speed_Ki = Rece_aray[1].float_data;
                }
            }
            if (Rece_aray[0].char_table[2] == 0x02) // 位置环使能
            {
                if (Rece_aray[0].char_table[3] == 0x04) // 调位指令
                {
                    handler->pos_ref = Rece_aray[1].float_data;
                    // hand_task_handler_ptr->motor_angle[motor] = handler->pos_ref;
                }
                else if (Rece_aray[0].char_table[3] == 0x05) // 位置环Kp
                {
                    handler->pos_Kp = Rece_aray[1].float_data;
                }
                else if (Rece_aray[0].char_table[3] == 0x06) // 位置环Kd
                {
                    handler->pos_Kd = Rece_aray[1].float_data;
                }
            }
        }
        else if (Rece_aray[1].char_table[0] == 1)
        {
            // 电机失能
            hand_task_handler_ptr->motor_ctrl_mode[DM_J2] = NON_FORCE;
        }
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

    vofa_send_data_pack[0] = chassis_task_handler.motor_speed[0];
    vofa_send_data_pack[1] = chassis_task_handler.feedback_motor_speed[0];
}

void vofa_uart_task(void *argument)
{
    Vofa_Init(&vofa_handler, VOFA_MODE_SKIP);
    while (1)
    {
        // vofa接收调用
        UART7_Recv((void *)Rece_aray, 8 * sizeof(uint8_t));
        vofa_rece_unpack(vofa_para_ptr, DM_J2);
        // vofa发送调用
        vofa_data_into_pack(vofa_send_data_pack);
        Vofa_JustFloat(&vofa_handler, vofa_send_data_pack, VOFA_FEEDBACK_COUNT);
        osDelay(1);
    }
}
