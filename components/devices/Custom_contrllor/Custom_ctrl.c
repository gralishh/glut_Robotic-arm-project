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
  static uint32_t uart7_length=0x00;
  while(1)
  {
    next_uart7:
    uart7_length = UART7_GetDataCount();  // 得出数据的长度，包括帧头、帧尾、ID和有用的数据

    if(uart7_length >= PACK_LENGTH)
    {
      if(IS_HEADER(UART7_At(0)))  
      {
        UART7_Recv(RX_BUF, PACK_LENGTH);  
        Custom_Ctrl_unpack();
      }
      else
      {
        uart7_length = UART7_GetDataCount();
        if(uart7_length > 3000)
        {
            UART7_Drop(4096);
        }
        goto next_uart7;
      }
    }
    else
    {
      for(;;)
      {
		    uart7_length =  UART7_GetDataCount();
        if( uart7_length > 0)
        {
            if(IS_HEADER(UART7_At(0)))  // Frame head
            {
              break;
            }
            else
            {
              UART7_Drop(1);
              vTaskDelay(1);
            }
            uart7_length =  UART7_GetDataCount();
            if(uart7_length > 3000)
            {
              UART7_Drop(4096);
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
