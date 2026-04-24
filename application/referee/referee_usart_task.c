/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "detect_task.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"



/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
static void referee_unpack_fifo_data(void);

 
extern UART_HandleTypeDef huart6;

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

unpack_data_t referee_unpack_obj;

/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void referee_usart_task(void const * argument)
{
    init_referee_struct_data();
    //fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    //usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
    while(1)
    {
        referee_unpack_fifo_data();
        osDelay(10);
    }
}


/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
void referee_unpack_fifo_data(void)
{
  static uint32_t rx_length=0x00;
  static uint16_t data_seq_length=0x00;
  rx_length = USART10_GetDataCount();  // 得出数据的长度，包括帧头、帧尾、ID和有用的数据

  if(data_seq_length)
  {
    if(rx_length>=REF_PROTOCOL_CMD_SIZE + data_seq_length+REF_PROTOCOL_CRC16_SIZE)
    {
      USART10_Recv(referee_unpack_obj.protocol_packet+REF_PROTOCOL_HEADER_SIZE,REF_PROTOCOL_CMD_SIZE+data_seq_length+REF_PROTOCOL_CRC16_SIZE);
      if ( verify_CRC16_check_sum(referee_unpack_obj.protocol_packet, REF_HEADER_CRC_CMDID_LEN + data_seq_length) )
      {
        referee_data_solve(referee_unpack_obj.protocol_packet);
      }
      data_seq_length=0x00;
    }
  }
  else if(USART10_At(0)==HEADER_SOF)  
  {
    if(rx_length>REF_PROTOCOL_HEADER_SIZE)
    {
      USART10_Recv(referee_unpack_obj.protocol_packet,REF_PROTOCOL_HEADER_SIZE);
      if(verify_CRC8_check_sum(referee_unpack_obj.protocol_packet, REF_PROTOCOL_HEADER_SIZE))
      {
        data_seq_length=((frame_header_struct_t*)(referee_unpack_obj.protocol_packet))->data_length;
      }
    }
  }
  else if(rx_length > 3000)
  {
    USART10_Drop(rx_length);
  }
  else
  {
    USART10_Drop(1);
  }
}

extern bool_t refereeControl;

