#include "Custom_ctrl.h"

#define HEADER 0xa5
/**/
#define IS_HEADER(byte) (byte==HEADER)

inline CUSTOM_CTRL_T* Custom_Ctrl_get_ptr(void)
{
  return &Custom_Ctrl_handler;
}

inline CUSTOM_CTRL_T* Custom_Ctrl_get_rx_pack_ptr(void)
{
  return &(Custom_Ctrl_handler.rx_pack);
}

/**
 * @brief 解包数据
 */
void Custom_Ctrl_unpack(Custom_Ctrl_handler* handler,uint8_t* pack)
{

}

/**
 * @brief 初始化数值
 */
void Data_init(data_t *data_init)
{
	data_init->header.sof = HEADER;
	data_init->header.data_length = 30;
	data_init->header.seq = 0;
	data_init->header.crc8 = 0;
	data_init->cmd_id = 0x0302;
}

void Custom_Ctrl_Task(void* para)
{
  while(1)
  {
    next_usart2:
    usart2_length = USART2_GetDataCount();  // 得出数据的长度，包括帧头、帧尾、ID和有用的数据

    if(usart2_length >= 10)
    {
      if(USART2_At(0) == 0xFD && USART2_At(1) == 0xEE)  // 判断数据的起始值是否为0xFD 0xEE
      {
        USART2_Recv(Transmission_usart2, FEEDBACK_DATA_SIZE);  // 把数据出栈并存储在Transmission_BufferOfusart2，数据处理在中断里
        //usart2_motor_rx = *SERVO_Recv((MOTOR_recv *)Transmission_usart2);
        SERVO_Recv(&joint1_motor,Transmission_usart2);
        USART2_Drop(4096);
      }
      else
      {
        USART2_Drop(1);
        vTaskDelay(1);
        usart2_length = USART2_GetDataCount();
        if(usart2_length > 3000)
        {
            USART2_Drop(4096);
        }
        goto next_usart2;
      }
    }
    else
    {
      USART2_Drop(1);
      for(;;)
      {
		    usart2_length =  USART2_GetDataCount();
        if( usart2_length > 0)
        {
            if(USART2_At(0) == 0xFD && USART2_At(1) == 0xEE)  // Frame head
            {
                break;
            }
            else
            {
                USART2_Drop(1);
                vTaskDelay(1);
            }
             usart2_length =  USART2_GetDataCount();
            if(usart2_length > 3000)
            {
                USART2_Drop(4096);
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

}
