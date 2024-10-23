/**
 * @note 注意板间通信发送函数是否被注释
 */
#include "MCU_communicaton_task.h"
#include "referee_DispatchTask.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "cmsis_os.h"
#include "circular_buffer.h"
#include "referee.h"
#include "gimbal_task.h"
#include "hand_task.h"
#include "PC_communication_task.h"
#include "CAN_receive.h"
#include "math.h"
ext_self_control_t *chassis_self_control;
visuals_rx_t *chassis_visuals_rx_data;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

void chassis_send_task(void const *parmas)
{
    #if chassisControlBorad
    chassis_self_control = get_self_control_original_point();
    chassis_visuals_rx_data = (visuals_rx_t*)get_visuals_rx_point();
    vTaskDelay(1200);//等待接收缓冲区完成初始化
    while(1)
    {
        if(chassis_self_control->self_control_point.header  == 0xAA && chassis_self_control->self_control_point.tailer == 0x08 )
        {
            /*取消主控板间通信*/
            //CanSendMoreMess(&hfdcan2 , CAN_CHASSIS_CONTROLLER , (uint8_t *)&chassis_self_control->self_control_point, sizeof(chassis_self_control->self_control_point));
            chassis_self_control->self_control_point.tailer = 0X00; //防止重复发送
        }
        if (chassis_visuals_rx_data->header == 0xaa && chassis_visuals_rx_data->tailer == 0x70 )
        {
            /*取消主控板间通信*/
            //CanSendMoreMess(&hfdcan2 , CAN_CHASSIS_CONTROLLER , (uint8_t *)&chassis_visuals_rx_data , sizeof(chassis_visuals_rx_data));
            chassis_visuals_rx_data->tailer = 0x00;
        }

        vTaskDelay(1);
    }  
   #endif
} 





uint16_t chassis_can2_length;
uint8_t chassis_can2_rxbuf[4096];
CircBuf_t chassis_can2_rxcbuf;
gimbal_to_chassis_t chassis_rx;
UI_need_data_t chassis_UI_need;
visuals_need_data_t chassis_Visual_need;

const UI_need_data_t *get_UI_need_data_point(void)
{
    return &chassis_UI_need;
}
const visuals_need_data_t *get_Visual_need_data_point(void)
{
    return &chassis_Visual_need;
}


void chassis_receive_task(void const *parmas)
{
    CircBuf_Init(&chassis_can2_rxcbuf , chassis_can2_rxbuf , 4096);
    vTaskDelay(100);
    while(1)
    {
       chassis_next_can2:
       chassis_can2_length = CircBuf_GetUsedSize(&chassis_can2_rxcbuf);  // 得出数据的长度，包括帧头、帧尾、ID和有用的数据

       if(chassis_can2_length >= 10)
       {
           if(CircBuf_At(&chassis_can2_rxcbuf , 0) == 0xaa && CircBuf_At(&chassis_can2_rxcbuf , 39) == 0x70)  // 判断数据的起始值是否为0xFD 0xEE
           {
               CircBuf_Push(&chassis_can2_rxcbuf , (unsigned char *)&chassis_rx , 40);  // 把数据出栈并存储在Transmission_BufferOfUsart6，数据处理在中断里
               gimbal_data_allocate_to_UI_or_Visual(chassis_rx , &chassis_UI_need , &chassis_Visual_need);
           }
           else
           {
               CircBuf_Drop(&chassis_can2_rxcbuf , 1);
               vTaskDelay(1);
               
				chassis_can2_length = CircBuf_GetUsedSize(&chassis_can2_rxcbuf);
               if(chassis_can2_length > 3000)
               {
                   CircBuf_Drop(&chassis_can2_rxcbuf , 4096);
               }
               
               goto chassis_next_can2;
           }
       }
       else
       {
           CircBuf_Drop(&chassis_can2_rxcbuf , 1);
           for(;;)
           {
				chassis_can2_length = CircBuf_GetUsedSize(&chassis_can2_rxcbuf);
               if( chassis_can2_length > 0)
               {
                   if(CircBuf_At(&chassis_can2_rxcbuf , 0) == 0xaa && CircBuf_At(&chassis_can2_rxcbuf , 39) == 0x80)  // 判断数据的起始值是否为0xFD 0xEE
                   {
                       break;
                   }
                   else
                   {
                       CircBuf_Drop(&chassis_can2_rxcbuf , 1);
                       vTaskDelay(1);
                   }
                    chassis_can2_length =  CircBuf_GetUsedSize(&chassis_can2_rxcbuf);
                   if(chassis_can2_length > 3000)
                   {
                       CircBuf_Drop(&chassis_can2_rxcbuf , 4096);
                       break;
                   }
               }
               else
               {
                   break;
               }
           }
       }
        vTaskDelay(15);
    }
}


static void gimbal_data_allocate_to_UI_or_Visual(const gimbal_to_chassis_t chassis_rx , UI_need_data_t* UI_need , visuals_need_data_t* Visual_need )
{
    for(uint8_t i= 0 ; i < 4 ; i++)
    {
        Visual_need->motor_position1[i] = chassis_rx.gimbal_status[i];
        Visual_need->motor_position2[i] = chassis_rx.gimbal_status[i+4];
        Visual_need->motor_position3[i] = chassis_rx.gimbal_status[i+8];
        Visual_need->motor_position4[i] = chassis_rx.gimbal_status[i+12];
        Visual_need->motor_position5[i] = chassis_rx.gimbal_status[i+16];
        Visual_need->motor_position6[i] = chassis_rx.gimbal_status[i+20];
        Visual_need->motor_position7[i] = chassis_rx.gimbal_status[i+24];
    }

}

gimbal_status_t *gimbal_status_float_to_byte;
hand_status_t *hand_status_float_to_byte;
gimbal_to_chassis_t gimbal_tx;
typedef union
{
    float fdata;
    unsigned long ldata;
} FloatLongType;
void Float_to_Byte(float f,unsigned char byte[])
{
    FloatLongType fl;
    fl.fdata=f;
    byte[0]=(unsigned char)fl.ldata;
    byte[1]=(unsigned char)(fl.ldata>>8);
    byte[2]=(unsigned char)(fl.ldata>>16);
    byte[3]=(unsigned char)(fl.ldata>>24);
}
void gimbal_send_task(void const *parmas)
{
    #if gimbalControlBoard
    gimbal_status_float_to_byte = (gimbal_status_t *)get_gimbal_status_point();
    hand_status_float_to_byte = (hand_status_t *)get_hand_status_point();
    gimbal_tx.header = 0xaa;
    gimbal_tx.tailer = 0x70;
    vTaskDelay(200);//等待接收缓冲区完成初始化
    while(1)
    {
        Float_to_Byte(gimbal_status_float_to_byte->motor_position1 , &gimbal_tx.gimbal_status[0]);
        Float_to_Byte(gimbal_status_float_to_byte->motor_position2 , &gimbal_tx.gimbal_status[4]);

        Float_to_Byte(hand_status_float_to_byte->motor_position3 , &gimbal_tx.hand_status[0]);
        Float_to_Byte(hand_status_float_to_byte->motor_position4 , &gimbal_tx.hand_status[4]);
        Float_to_Byte(hand_status_float_to_byte->motor_position5 , &gimbal_tx.hand_status[8]);
        Float_to_Byte(hand_status_float_to_byte->motor_position6 , &gimbal_tx.hand_status[12]);
        Float_to_Byte(hand_status_float_to_byte->motor_position7 , &gimbal_tx.hand_status[16]);
        /*取消主控板间通信*/
        //CanSendMoreMess(&hfdcan2 , CAN_GIMBAL_CONTROLLER , (uint8_t *)&gimbal_tx , sizeof(gimbal_tx));

        vTaskDelay(15);
    }  
   #endif
} 

static elf_measure_t gimbal_self_measure;
const elf_measure_t *get_self_measure_point(void)
{
    return &gimbal_self_measure;
}

visuals_rx_t gimbal_visuals_rx;
static visuals_rx_data_t gimbal_visuals_rx_data;
const  visuals_rx_data_t *get_visuals_rx_data_point(void)
{
    return &gimbal_visuals_rx_data;
}



uint16_t gimbal_can2_length;
uint8_t gimbal_can2_rxbuf[4096];
CircBuf_t gimbal_can2_rxcbuf;
ext_self_control_t gimbal_self_control;

void gimbal_receive_task(void const *parmas)
{
	#if gimbalControlBoard
    CircBuf_Init(&gimbal_can2_rxcbuf , gimbal_can2_rxbuf , 4096);
    vTaskDelay(50);
    while(1)
    {
        gimbal_next_can2:
        gimbal_can2_length = CircBuf_GetUsedSize(&gimbal_can2_rxcbuf);  // 得出数据的长度，包括帧头、帧尾、ID和有用的数据

        if(gimbal_can2_length >= 10)
        {
            if(CircBuf_At(&gimbal_can2_rxcbuf , 0) == 0xAA && CircBuf_At(&gimbal_can2_rxcbuf , 29) == 0x08)  // 判断数据的起始值是否为0xFD 0xEE
            {
                CircBuf_Pop(&gimbal_can2_rxcbuf , (unsigned char *)&gimbal_self_control.self_control_point, 30);  // 把数据出栈并存储在Transmission_BufferOfUsart6，数据处理在中断里
                self_control_measure(gimbal_self_control , &gimbal_self_measure);
            }
//			 if(CircBuf_At(&gimbal_can2_rxcbuf , 0) == 0xaa)
//			 {
//				 CircBuf_Pop(&gimbal_can2_rxcbuf , (unsigned char *)&gimbal_self_control.self_control_point, 30);  // 把数据出栈并存储在Transmission_BufferOfUsart6，数据处理在中断里
//                 self_control_measure(gimbal_self_control , &gimbal_self_measure);
//			 }
            if(CircBuf_At(&gimbal_can2_rxcbuf , 0) == 0xAA && CircBuf_At(&gimbal_can2_rxcbuf , 43) == 0x80)  // 判断数据的起始值是否为0xFD 0xEE
            {
                CircBuf_Pop(&gimbal_can2_rxcbuf , (unsigned char *)&gimbal_visuals_rx, 15);  // 把数据出栈并存储在Transmission_BufferOfUsart6，数据处理在中断里
                Transmission_Visual_measure_visuals_rx(gimbal_visuals_rx , &gimbal_visuals_rx_data);
            }
            else
            {
                CircBuf_Drop(&gimbal_can2_rxcbuf , 1);
                vTaskDelay(1);
                
				gimbal_can2_length = CircBuf_GetUsedSize(&gimbal_can2_rxcbuf);
                if(gimbal_can2_length > 3000)
                {
                    CircBuf_Drop(&gimbal_can2_rxcbuf , 1);
                }
                
                goto gimbal_next_can2;
            }
        }
        else
        {
            CircBuf_Drop(&gimbal_can2_rxcbuf , 1);
            for(;;)
            {
				gimbal_can2_length = CircBuf_GetUsedSize(&gimbal_can2_rxcbuf);
                if( gimbal_can2_length > 0)
                {
                    if(CircBuf_At(&gimbal_can2_rxcbuf , 0) == 0xaa)  // 判断数据的起始值是否为0xaa 
                    {
                        break;
                    }
                    else
                    {
                        CircBuf_Drop(&gimbal_can2_rxcbuf , 1);
                        vTaskDelay(1);
                    }
                     gimbal_can2_length =  CircBuf_GetUsedSize(&gimbal_can2_rxcbuf);
                    if(gimbal_can2_length > 3000)
                    {
                        CircBuf_Drop(&gimbal_can2_rxcbuf , 4096);
                        break;
                    }
                }
                else
                {
                    break;
                }
            }
        }
        vTaskDelay(1);
    }
	#endif
}

static float bytesToFloat(const uint8_t* byte) {
    // 将四个字节按照字节序列合成一个32位整数
    uint32_t combinedBytes = (*byte << 24) | (*(byte + 1) << 16) | (*(byte + 2) << 8) | *(byte + 3);
    // 使用联合体将整数转换为浮点数
    union {
        uint32_t intValue;
        float floatValue;
    } converter;
    converter.intValue = combinedBytes;

    return converter.floatValue;
}

void Transmission_Visual_measure_visuals_rx(const visuals_rx_t Transmission_Visual , visuals_rx_data_t* visuals_rx)
{
    visuals_rx->vx = bytesToFloat(Transmission_Visual.vx);
    visuals_rx->vy = bytesToFloat(Transmission_Visual.vy);
    visuals_rx->position1 = bytesToFloat(Transmission_Visual.position1);
    visuals_rx->position2 = bytesToFloat(Transmission_Visual.position2);
    visuals_rx->position3 = bytesToFloat(Transmission_Visual.position3);
    visuals_rx->position4 = bytesToFloat(Transmission_Visual.position4);
    visuals_rx->position5 = bytesToFloat(Transmission_Visual.position5);
    visuals_rx->position6 = bytesToFloat(Transmission_Visual.position6);
    visuals_rx->position7 = bytesToFloat(Transmission_Visual.position7);
}
        

static float linear_map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (((x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min;
}
 void self_control_measure(const ext_self_control_t self_control , elf_measure_t*self_measure  )
{
    
    self_measure->last_position1 = self_measure->motor_position1;
    self_measure->last_position2 = self_measure->motor_position2;
    self_measure->last_position3 = self_measure->motor_position3;
    self_measure->last_position4 = self_measure->motor_position4;
    self_measure->last_position5 = self_measure->motor_position5;
    self_measure->last_position6 = self_measure->motor_position6;

    self_measure->header = self_control.self_control_point.header;

    self_measure->motor_position1 = bytesToFloat(self_control.self_control_point.motor_position1);
    self_measure->motor_position2 = bytesToFloat(self_control.self_control_point.motor_position2);
    self_measure->motor_position3 = bytesToFloat(self_control.self_control_point.motor_position3);
    self_measure->motor_position4 = bytesToFloat(self_control.self_control_point.motor_position4);
    self_measure->motor_position5 = bytesToFloat(self_control.self_control_point.motor_position5);
    self_measure->motor_position6 = bytesToFloat(self_control.self_control_point.motor_position6);


    self_measure->tailer = self_control.self_control_point.tailer;
}
static uint16_t merge_uint8_to_uint16(uint8_t a, uint8_t b)
{
    return ((uint16_t)a << 8) | b;
}
static void split_uint8_to_uint8(uint8_t input, uint8_t *output1, uint8_t *output2, uint8_t *output3) 
{
    *output1 = (input & 0x01) ? 0x01 : 0x00;
    *output2 = (input & 0x02) ? 0x01 : 0x00;
    *output3 = (input & 0x04) ? 0x01 : 0x00;
}


