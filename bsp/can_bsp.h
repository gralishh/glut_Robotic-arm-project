#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__
#include "main.h"
//#include "fdcan.h"

//#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

#define CAN_DATA_SIZE_8_BYTES FDCAN_DLC_BYTES_8
#define CAN_DATA_SIZE_4_BYTES FDCAN_DLC_BYTES_4
#define CAN_DATA_SIZE_1_BYTES FDCAN_DLC_BYTES_1

void can_bsp_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_send_data_ex_mode(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef* fdcan_RxHeader, uint8_t *buf);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);

#endif /* __CAN_BSP_H_ */

