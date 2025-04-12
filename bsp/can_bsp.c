#include "can_bsp.h"
#include "CAN_receive.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/**
************************************************************************
* @brief:      	can_bsp_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN ???
************************************************************************
**/
void can_bsp_init(void)
{
	can_filter_init();
	HAL_FDCAN_Start(&hfdcan1);                               //????FDCAN
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN??????????
************************************************************************
**/
void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //???ID
	fdcan_filter.FilterIndex = 0;                                  //?????????                   
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   //????????????ID
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //??????0??????FIFO0  
	fdcan_filter.FilterID1 = 0x000;                               //32λID
	fdcan_filter.FilterID2 = 0x000;                               //????ID1
	HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter); 		 				  //????ID2
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);

	HAL_FDCAN_ConfigFilter(&hfdcan2,&fdcan_filter); 		 				  //????ID2
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);

	HAL_FDCAN_ConfigFilter(&hfdcan3,&fdcan_filter); 		 				  //????ID2
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);
}

/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param       hfdcan??FDCAN???
* @param       id??CAN?豸ID
* @param       data???????????
* @param       len 该参数在此函数中未使用
* @retval      void
* @details     
* @warning     len参数在此函数中不被使用，在扩展帧中却被使用，如要修改
*              本函数，请同时检查该函数的所有调用的数值是否符合HAL文档中
*              FDCAN Data Length Code条目的宏定义
***********************************************************************
**/
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
	FDCAN_TxHeaderTypeDef TxHeader;
	
  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;																// 
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;														// 
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;                            // 固定报文为8字节
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;										
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;															// 
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;															// 
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;										// 
  TxHeader.MessageMarker = 0x00; 			// ????????TX EVENT FIFO?????Maker??????????????Χ0??0xFF                
    
  if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data)!=HAL_OK) 
		return 1;//????
	return 0;	
}

/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param       hfdcan can句柄
* @param       id     CAN设备ID
* @param       data   指向要发送的数据的指针
* @param       len    数据长度，必须为HAL库文档中FDCAN Data Length Code的宏定义之一
*                     或文件顶部的宏定义之一
* @retval      数据发送状态
* @details     
***********************************************************************
**/
uint8_t fdcanx_send_data_ex_mode(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;
	
  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = len;                            
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;										
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;														 
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;															 
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;									 
  TxHeader.MessageMarker = 0x00; 			
    
  if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data)!=HAL_OK) 
		return 1;//????
	return 0;	
}

/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan??FDCAN???
* @param:       buf?????????????
* @retval:     	????????????
* @details:    	????????
************************************************************************
**/
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef* fdcan_RxHeader, uint8_t *buf)
{	
  if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, fdcan_RxHeader, buf)!=HAL_OK)
		return 0;//????????
  return (fdcan_RxHeader->DataLength)>>16;	
}
/**
************************************************************************
* @brief:      	HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
* @param:       hfdcan??FDCAN???
* @param:       RxFifo0ITs???ж???λ
* @retval:     	void
* @details:    	HAL???FDCAN?ж???????
************************************************************************
**/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
		if(hfdcan == &hfdcan1)
		{
			fdcan1_rx_callback();
		}

		if(hfdcan == &hfdcan2)
		{
			fdcan2_rx_callback();
		}

		if(hfdcan == &hfdcan3)
		{
			fdcan3_rx_callback();
		}
	}
}
/**
************************************************************************
* @brief:      	fdcan_rx_callback(void)
* @param:       void
* @retval:     	void
* @details:    	???????????????????
************************************************************************
**/
uint8_t rx_data1[8] = {0};
void fdcan1_rx_callback(void)
{
	FDCAN_RxHeaderTypeDef fdcan_RxHeader;
	fdcanx_receive(&hfdcan1, &fdcan_RxHeader, rx_data1);
  CAN_RX_hook(&hfdcan1,&fdcan_RxHeader,rx_data1);
}

uint8_t rx_data2[8] = {0};
void fdcan2_rx_callback(void)
{
	FDCAN_RxHeaderTypeDef fdcan_RxHeader;
	fdcanx_receive(&hfdcan2, &fdcan_RxHeader, rx_data2);
  CAN_RX_hook(&hfdcan2,&fdcan_RxHeader,rx_data2);
}

uint8_t rx_data3[8] = {0};
void fdcan3_rx_callback(void)
{
	FDCAN_RxHeaderTypeDef fdcan_RxHeader;
	fdcanx_receive(&hfdcan3, &fdcan_RxHeader, rx_data3);
  CAN_RX_hook(&hfdcan3,&fdcan_RxHeader,rx_data3);
}
