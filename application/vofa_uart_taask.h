#ifndef VOFA_UART_TASK_H
#define VOFA_UART_TASK_H

#include "main.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

// typedef enum
// {
//     hand,
//     gimbal,
//     cahssis,
// } APPLICATION_TYPE_T;

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

    VOFA_DJI_LF,
    // VOFA_DJI_RF,
    // VOFA_DJI_LB,
    // DJI_RB,
    VOFA_DJI_LF_FB,
    // VOFA_DJI_RF_FB,
    // VOFA_DJI_LB_FB,
    // DJI_RB_FB,
    VOFA_FEEDBACK_COUNT,
} VOFA_FEEDBACK_INDEX;

typedef struct
{
    float speed_ref;
    float pos_ref;
    float speed_Kp;
    float speed_Ki;
    float pos_Kp;
    float pos_Kd;
} VOFA_TASK_HANDKER_TYPE;

typedef union
{
    float float_data;
    uint8_t char_table[4];
} Resolve_Typedef;

// extern float speed_ref;
// extern float pos_ref;
// extern float speed_Kp;
// extern float speed_Ki;
// extern float pos_Kp;
// extern float pos_Kd;

extern fp32 vofa_send_data_pack[VOFA_FEEDBACK_COUNT];
extern Resolve_Typedef Rece_aray[2];
extern VOFA_TASK_HANDKER_TYPE vofa_para;
extern VOFA_TASK_HANDKER_TYPE *vofa_para_ptr;
// 发送数据包

void vofa_rece_unpack(VOFA_TASK_HANDKER_TYPE *handler, uint8_t motor);

// 将想要显示的数据填入发送包中
void vofa_data_into_pack(fp32 *vofa_send_data_pack);
void vofa_uart_task(void *argument);

#endif
