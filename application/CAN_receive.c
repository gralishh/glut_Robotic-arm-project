#include "CAN_receive.h"
//#include "MCU_communicaton_task.h"
#include "circular_buffer.h"
#include "can_bsp.h"
#include "cmsis_os.h"
//#include "PC_communication_task.h"
#include "string.h"
#include "detect_task.h"

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

	
static motor_measure_t   motor_chassis[4],                                                              \
						_3508_RISE1_height_L, _3508_RISE2_height_R,_3508_LeftRight,\
						_2006_HAND_roll_1,_2006_HAND_pitch_2,_2006_HAND_x_4;\
						//	2006_VIEW_pitch_,
static	external_encoder_t  gimbal_encoder_height_l,gimbal_encoder_height_R,gimbal_encoder_leftRight,_encoder_HAND_x_4;  


//返回电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
/****************************************************************/
const motor_measure_t *get_3508_RISE1_height_L_Measure_Point(void)
{
    return &_3508_RISE1_height_L;
}
const motor_measure_t *get_3508_RISE2_height_R_Measure_Point(void)
{
    return &_3508_RISE2_height_R;
}
const motor_measure_t *get_3508_LeftRight_Measure_Point(void)
{
    return &_3508_LeftRight;
}

/****************************************************************/
const motor_measure_t *get__roll_1_Measure_Point(void)
{
    return &_2006_HAND_roll_1;
}
const motor_measure_t *get_2006_HAND_pitch_2_Measure_Point(void)
{
    return &_2006_HAND_pitch_2;
}
const external_encoder_t *get_encoder_HAND_x_4_Measure_Point(void)
{
    return &_encoder_HAND_x_4;
}
const motor_measure_t *get_2006_HAND_x_4_Measure_Point(void)
{
    return &_2006_HAND_x_4;
}

/****************************************************************/
// const motor_measure_t *get_2006_VIEW_pitch_Measure_Point(void)
// {
//     return &_2006_VIEW_pitch;
// }
/****************************************************************/
const external_encoder_t *get_gimbal_encoder_height_L_Point(void)
{
    return &gimbal_encoder_height_l;
}
const external_encoder_t *get_gimbal_encoder_height_R_Point(void)
{
    return &gimbal_encoder_height_R;
}
const external_encoder_t *get_gimbal_encoder_leftRight_Point(void)
{
    return &gimbal_encoder_leftRight;
}




/**
 * @brief CAN接收钩子
 * @note 执行方式:判断对应CAN总线的对应ID，直接将报文赋值给对应电机的反馈报文结构体
 */
void CAN_RX_hook(FDCAN_HandleTypeDef* CANx, FDCAN_RxHeaderTypeDef* rx_header,uint8_t* rx_message) 
{
	if(CANx == &hfdcan2)
	{
		switch (rx_header->Identifier)
		{
			case CAN_2006_roll_1_ID:
			{
					//get_motor_measure(&_2006_HAND_roll_1, rx_message);
					get_cmotor_measure(&_2006_HAND_roll_1, rx_message, 2000 ,6000 ,8192);

					DetectHook(TOE_2006_roll_1_ID);
					break;
			}
			case CAN_2006_HAND_pitch_2_ID:
			{
					
					get_cmotor_measure(&_2006_HAND_pitch_2, rx_message, 2000 ,6000 ,8192);

					DetectHook(TOE_2006_HAND_pitch_2_ID);
					break;
			}
		    case CAN_2006_HAND_x_4_ID:
			{
					
					get_motor_measure(&_2006_HAND_x_4, rx_message );

					DetectHook(TOE_2006_HAND_x_4_ID);
					break;
			}
		    case CAN_encoder_HAND_x_4_ID:
			{
					
					external_cencoder(&_encoder_HAND_x_4, rx_message , 2000 ,30000 ,32768);

					DetectHook(TOE_encoder_x_4);
					break;
			}
    }
  }
  else if(CANx == &hfdcan1)
  {
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
			default:
			{

				break;
			}	
    }
	}
	else//CAN3
	{
		switch (rx_header->Identifier)
		{	
			
			case CAN_EXTER_ENCODER_HEIGHT_L:
			{					
					external_encoder(&gimbal_encoder_height_l, rx_message);
					DetectHook(TOE_EXTER_ENCODER_1);
					break;					    
			}
			case CAN_EXTER_ENCODER_HEIGHT_R:
			{	
					external_encoder(&gimbal_encoder_height_R, rx_message);
					DetectHook(TOE_EXTER_ENCODER_2);
					break;					    
			}
			case CAN_EXTER_ENCODER_LEFTRIGHT:
			{	
					external_encoder(&gimbal_encoder_leftRight, rx_message);
					DetectHook(TOE_EXTER_ENCODER_3);
					break;					    
			}
			case CAN_3508_RISE1_height_L_ID:
			{
					get_motor_measure(&_3508_RISE1_height_L, rx_message);
					DetectHook(TOE_3508_RISE1_height_L_5_ID);
					break;
			}
			case CAN_3508_RISE1_height_R_ID:
			{
					
					get_motor_measure(&_3508_RISE2_height_R, rx_message);
					DetectHook(TOE_3508_RISE2_height_R_6_ID);
					break;
			}	
			case CAN_3508_LeftRight_ID:
			{
					
					get_motor_measure(&_3508_LeftRight, rx_message);
					DetectHook(TOE_3508_LeftRight_7_ID);
					break;
			}
			//#if chassisControlBorad
			case CAN_CHASSIS_CONTROLLER:
			{
  				//CircBuf_Push(&gimbal_can2_rxcbuf , rx_message , sizeof(rx_message)); 
				break;
			}
			//#endif
			//  #if gimbalControlBoard
			case CAN_GIMBAL_CONTROLLER:
			{
				//CircBuf_Push(&chassis_can2_rxcbuf , rx_message , sizeof(rx_message)); //视觉用缓冲区处理 
				break;
//				memcpy(&gimbal_can2_rxbuf , rx_message , sizeof(rx_message));
//				self_control_measure(gimbal_can2_rxbuf , &gimbal_can_self_measure);
			}
		//	#endif
	    }
    } 
}




void CanSendMess(FDCAN_HandleTypeDef* CANx,uint32_t SendID,int16_t *message)
{
	

	//int32_t send_mail_box0;
  //uint32_t send_mail_box1;
  //CAN_TxHeaderTypeDef tx_message;
  //tx_message.StdId = SendID;  //ID 设备标识符（编码器地址）
  //tx_message.IDE = CAN_ID_STD;  //标准帧
  //tx_message.RTR = CAN_RTR_DATA;     //数据帧
  //tx_message.DLC = 0x08; //帧长度  数据长度
	
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

