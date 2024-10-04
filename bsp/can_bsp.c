#include "can_bsp.h"
#include "CAN_receive.h"

extern FDCAN_HandleTypeDef hfdcan1;

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
	//HAL_FDCAN_Start(&hfdcan2);
	//HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	//HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	//HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
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
	fdcan_filter.FilterType = FDCAN_FILTER_DUAL;                   //????????????ID
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //??????0??????FIFO0  
	fdcan_filter.FilterID1 = 0x204;                               //32λID
	fdcan_filter.FilterID2 = 0x201;                               //????ID1
	HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter); 		 				  //????ID2
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan??FDCAN???
* @param:       id??CAN?豸ID
* @param:       data???????????
* @param:       len??????????????
* @retval:     	void
* @details:    	????????
************************************************************************
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
		//if(hfdcan == &hfdcan2)
		//{
		//	fdcan2_rx_callback();
		//}
		//if(hfdcan == &hfdcan3)
		//{
		//	fdcan3_rx_callback();
		//}
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
  /*
  sprintf((char *)CDC_tx_data,"%-8d %-d\n",(rx_data1[2]<<8)+rx_data1[3],(rx_data1[0]<<8)+rx_data1[1]);
  CDC_Transmit_HS(CDC_tx_data,strlen((char*)CDC_tx_data));
  */
}
//uint8_t rx_data2[8] = {0};
//void fdcan2_rx_callback(void)
//{
	//fdcanx_receive(&hfdcan2, rx_data2);
//}
//uint8_t rx_data3[8] = {0};
//void fdcan3_rx_callback(void)
//{
	//fdcanx_receive(&hfdcan3, rx_data3);
//}
