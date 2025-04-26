#include "Custom_ctrl.h"
#include "crc.h"
#include "bsp_usart.h"
#include "cmsis_os.h"

#define HEADER 0xa5
#define PACK_LENGTH (sizeof(data_t))
#define RX_BUF ((void*)Custom_Ctrl_get_rx_pack_ptr())
/**/
#define IS_HEADER(byte) (byte==HEADER)

CUSTOM_CTRL_T CC_handler;

inline CUSTOM_CTRL_T* Custom_Ctrl_get_ptr(void)
{
  return &CC_handler;
}

inline data_t* Custom_Ctrl_get_rx_pack_ptr(void)
{
  return &(CC_handler.rx_pack);
}

/**
 * @brief 解包数据
 */
void Custom_Ctrl_unpack(void)
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
  static uint32_t usart1_length=0x00;
  while(1)
  {
    next_usart1:
    usart1_length = USART1_GetDataCount();  // 得出数据的长度，包括帧头、帧尾、ID和有用的数据

    if(usart1_length >= PACK_LENGTH)
    {
      if(IS_HEADER(USART1_At(0)))  
      {
        //if(USART1_At(5)==0x02 && USART1_At(6)==0x03)
        USART1_Recv(RX_BUF, sizeof(frame_header_t)+sizeof(uint16_t));  
        if(((data_t*)RX_BUF)->cmd_id==0x0302)
          USART1_Recv((void*)&(((data_t*)RX_BUF)->key),sizeof(float)*6+sizeof(uint8_t)*4+sizeof(uint16_t));


        Custom_Ctrl_unpack();
        //USART1_Drop(4096);
      }
      else
      {
        USART1_Drop(1);
        vTaskDelay(1);
        usart1_length = USART1_GetDataCount();
        if(usart1_length > 3000)
        {
            USART1_Drop(4096);
        }
        goto next_usart1;
      }
    }
    else
    {
      for(;;)
      {
		    usart1_length =  USART1_GetDataCount();
        if( usart1_length > 0)
        {
            if(IS_HEADER(USART1_At(0)))  // Frame head
            {
              break;
            }
            else
            {
              USART1_Drop(1);
              vTaskDelay(1);
            }
            usart1_length =  USART1_GetDataCount();
            if(usart1_length > 3000)
            {
              USART1_Drop(4096);
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
