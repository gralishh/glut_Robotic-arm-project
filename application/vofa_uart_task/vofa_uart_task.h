#ifndef VOFA_UART_TASK_H
#define VOFA_UART_TASK_H

#include "main.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#include "dm4310_drv.h"
#include "AK_series.h"

#define PI 3.14159f
typedef enum
{
    MOTOR_START = 0x001 << 0,
    MOTOR_OFF = 0x001 << 1,
    MOTOR_SPEED_LOOP_EN = 0x001 << 2,
    MOTOR_SPEED_LOOP_DISEN = 0x001 << 3,
    MOTOR_POS_LOOP_EN = 0x001 << 4,
    MOTOR_POS_LOOP_DISEN = 0x001 << 5,
    MOTOR_GET_SPEED = 0x001 << 6,
    MOTOR_GET_POS = 0x001 << 7,
    MOTOR_GET_SPEED_Kp = 0x001 << 8,
    MOTOR_GET_SPEED_Ki = 0x001 << 9,
    MOTOR_GET_POS_Kp = 0x001 << 10,
    MOTOR_GET_POS_Kd = 0x001 << 11,
} Vofa_motor_Ctrl_state_e;

// 显示的数据
typedef enum
{
    // VOFA_AK_J1,
    //  VOFA_DM_J2,
    //  VOFA_DJI_J3,
    //  VOFA_DJI_HE_L,
    // VOFA_DJI_HE_R,
    // VOFA_AK_J1_FB,
    //  VOFA_DM_J2_FB,
    //  VOFA_DJI_J3_FB,
    //  VOFA_DJI_HE_L_FB,
    //   VOFA_DJI_HE_R_FB,

    // VOFA_DJI_LF,
    //  VOFA_DJI_RF,
    //  VOFA_DJI_LB,
    //  DJI_RB,
    // VOFA_DJI_LF_FB,
    //  VOFA_DJI_RF_FB,
    //  VOFA_DJI_LB_FB,
    //  DJI_RB_FB,

    vofa_up_s,
    vofa_up_s_f,
    vofa_up_p,
    vofa_up_p_f,
    VOFA_FEEDBACK_COUNT,
}
VOFA_FEEDBACK_INDEX;

typedef struct
{
    float speed_ref;
    float pos_ref;
    float speed_Kp;
    float speed_Ki;
    float pos_Kp;
    float pos_Kd;

    Vofa_motor_Ctrl_state_e vofa_motor_en;

} VOFA_TASK_HANDKER_TYPE;

typedef __packed struct
{
    uint8_t char_check[2];
    uint8_t char_type;
    float float_data;
} Resolve_Typedef;

typedef Resolve_Typedef RX_pack;

extern fp32 vofa_send_data_pack[VOFA_FEEDBACK_COUNT];
extern RX_pack Rece_aray;
extern RX_pack *Rece_aray_ptr;
extern VOFA_TASK_HANDKER_TYPE vofa_para;
extern VOFA_TASK_HANDKER_TYPE *vofa_para_ptr;
// 发送数据包
void vofa_rx_unpack(void);
void vofa_rece_set_type(VOFA_TASK_HANDKER_TYPE *handler, RX_pack *Rece_pack_ptr);
void vofa_type_set_motor_HOOK(VOFA_TASK_HANDKER_TYPE *handler, AK_Joint_Motor_t *motor_handler, uint8_t motor);

// 将想要显示的数据填入发送包中
void vofa_data_into_pack(fp32 *vofa_send_data_pack);
void vofa_uart_task(void *argument);

#endif
