#include "CAN_receive.h"
//#include "MCU_communicaton_task.h"
#include "circular_buffer.h"
#include "can_bsp.h"
#include "cmsis_os.h"
//#include "PC_communication_task.h"
#include "string.h"
#include "detect_task.h"
#include "Ex_encoder.h"
#include "DJI_motor_canbus.h"
#include "dm4310_drv.h"
#include "AK_series.h"

extern Joint_Motor_t DM_Motor_J2;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/*"云台"和底盘主控通信缓冲*/
//extern CircBuf_t chassis_can2_rxcbuf;
//extern CircBuf_t gimbal_can2_rxcbuf;

#define get_motor_measure(ptr, rx_message)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((rx_message)[0] << 8 | (rx_message)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((rx_message)[2] << 8 | (rx_message)[3]);      \
        (ptr)->given_current = (uint16_t)((rx_message)[4] << 8 | (rx_message)[5]);  \
        (ptr)->temperate = (rx_message)[6];                                   \
    }                 \

#define get_cmotor_measure(ptr, rx_message , min_limit , max_limit , ecdforall)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((rx_message)[0] << 8 | (rx_message)[1] );            \
		if((ptr)->last_ecd > max_limit  && (ptr)->ecd < min_limit ) 		\
			{					\
					(ptr)->crc++;                        \
			}																																							\
		if((ptr)->ecd > max_limit && (ptr)->last_ecd < min_limit ) 		\
			{								             														\
					(ptr)->crc--;                    \
			}																				\
        (ptr)->speed_rpm = (uint16_t)((rx_message)[2] << 8 | (rx_message)[3]);      \
        (ptr)->given_current = (uint16_t)((rx_message)[4] << 8 | (rx_message)[5]);  \
        (ptr)->temperate = (rx_message)[6];                                   \
    }     	\
	
#define external_encoder(ptr, rx_message)                                                       \
    {                                                                                           \
        (ptr)->data_lenth = (uint8_t)(rx_message[0]);                        \
        (ptr)->encoder_ID = (uint8_t)(rx_message[1]);                        \
        (ptr)->func_cmd   = (uint8_t)(rx_message[2]);                        \
		    (ptr)->ecd        = (uint32_t)(rx_message[3] | (uint32_t)rx_message[4] << 8 | (uint32_t)rx_message[5] << 16  );  \
	}    \
		
#define external_cencoder(ptr, rx_message , min_limit , max_limit , ecdforall )           \
    {             \
				(ptr)->last_ecd = (ptr)->ecd;                               \
				(ptr)->ecd = (uint32_t)(rx_message[3] | (uint32_t)rx_message[4] << 8 | (uint32_t)rx_message[5] << 16  ) + (ptr)->crc*ecdforall; \
				if((ptr)->last_ecd > max_limit + (ptr)->crc*ecdforall  && (ptr)->ecd < min_limit + (ptr)->crc*ecdforall) 		\
				{					\
					(ptr)->crc=(uint8_t)1;                        \
				}																																							\
				if((ptr)->ecd > max_limit + (ptr)->crc*ecdforall && (ptr)->last_ecd < min_limit + (ptr)->crc*ecdforall) 		\
				{																						\
					(ptr)->crc=(uint8_t)0;                    \
				}		     \
		(ptr)->ecd = (uint32_t)(rx_message[3] | (uint32_t)rx_message[4] << 8 | (uint32_t)rx_message[5] << 16  ) + (ptr)->crc*ecdforall; \
        (ptr)->data_lenth = (uint8_t)(rx_message[0]);                        \
        (ptr)->encoder_ID = (uint8_t)(rx_message[1]);                        \
        (ptr)->func_cmd   = (uint8_t)(rx_message[2]);                        \
	}    \

static motor_measure_t   motor_chassis[4];

//返回电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

/**
 * @brief CAN接收钩子
 * @note 执行方式:判断对应CAN总线的对应ID，直接将报文赋值给对应电机的反馈报文结构体
 */
void CAN_RX_hook(FDCAN_HandleTypeDef* CANx, FDCAN_RxHeaderTypeDef* rx_header,uint8_t* rx_message) 
{
  


	if(CANx == &hfdcan2)
	{
    __DJI_CANBus_feedback_update(&DJI_CAN2_Bus_ctrl,rx_message,rx_header->Identifier);
		switch (rx_header->Identifier)
		{
      case 0x233:
        dm4310_fbdata(&DM_Motor_J2,rx_message,FDCAN_DLC_BYTES_8);
        DetectHook(TOE_J2);
        break;
      case 0x295d:
        __AK_joint_motor_feedback_hook(&AK70_10_motor,rx_message);
        DetectHook(TOE_J1);
        break;
      case 0x201:
        DetectHook(TOE_HE_L);
        break;
      case 0x204:
        DetectHook(TOE_J3);
        break;
      case 0x208:
        DetectHook(TOE_HE_R);
        break;
      default:
        break;/*do nothing*/
    }
  }
  else if(CANx == &hfdcan1)
  {
    __DJI_CANBus_feedback_update(&DJI_CAN1_Bus_ctrl,rx_message,rx_header->Identifier);
    switch(rx_header->Identifier)
    {
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			{
					static uint8_t i = 0;
					//处理电机ID号
					i = rx_header->Identifier - CAN_3508_M1_ID;
					
					get_motor_measure(&motor_chassis[i], rx_message);

					DetectHook(TOE_3508_M1_ID + i);
					break;
			}
      case 0x205:
        DetectHook(TOE_UPLIFT);
        break;
			default:
				break;
    }
	}
	else//CAN3
	{
	__External_ecd_can_feedback_hook(rx_header->Identifier,rx_message);
    __DJI_CANBus_feedback_update(&DJI_CAN3_Bus_ctrl,rx_message,rx_header->Identifier);
		switch (rx_header->Identifier)
		{	
      default:
      ;
    } 
  }
}



/**
 * @brief CAN发送混乱 XD
 * @details 以DJI电机特有的颠倒方式发送数据
 */
void CanSendMess(FDCAN_HandleTypeDef* CANx,uint32_t SendID,int16_t *message)
{
	
  uint8_t can_send_data[8];
  can_send_data[0] = (uint8_t)(message[0] >> 8); 
  can_send_data[1] = (uint8_t) message[0];          
  can_send_data[2] = (uint8_t)(message[1] >> 8);
  can_send_data[3] = (uint8_t) message[1];
  can_send_data[4] = (uint8_t)(message[2] >> 8);
  can_send_data[5] = (uint8_t) message[2];
  can_send_data[6] = (uint8_t)(message[3] >> 8);
  can_send_data[7] = (uint8_t) message[3];

  fdcanx_send_data(CANx,SendID,can_send_data,0x08);
  	
}

/**
 * @brief CAN发送消息
 */
void CanSendMsg(FDCAN_HandleTypeDef* CANx,uint32_t SendID,uint8_t *message)
{
  uint8_t can_send_data[8];
  memcpy(can_send_data,(void*)message,0x08);
  fdcanx_send_data(CANx,SendID,can_send_data,0x08);
}

//允许发送任意长度的数据，并在不足8个字节的部分用0填充
void CanSendMoreMess(FDCAN_HandleTypeDef* CANx, uint32_t SendID, uint8_t *message, uint8_t messageLength)
{
    //// 计算需要填充的0的数量
    uint8_t fill_zeros = (4 - (messageLength % 4)) % 4;

    // 发送数据
    for (uint8_t i = 0; i < messageLength + fill_zeros; i += 4) {
        // 将数据拷贝到发送缓冲区
        uint8_t can_send_data[8] = {0}; // 初始化为0
        for (uint8_t j = 0; j < 4; ++j) {
            if (i + j < messageLength) {
                can_send_data[j] = message[i + j];
            }
        }  

        // 发送数据
        fdcanx_send_data(CANx,SendID,can_send_data,0x08);
		
		    vTaskDelay(20);
    }
	
}

