#ifndef BSP_RC_H
#define BSP_RC_H
#include "struct_typedef.h"
#include "main.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

#define DBUS_USART huart5
#define DMA_DBUS_USART_RX (*(huart5.hdmarx))
#define DBUS_UART_OBJ UART5
//#define DBUS_IRQHandler UART5_IRQHandler

extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);
#endif
