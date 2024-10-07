#include "bsp_rc.h"
#include "main.h"

//获取DMA_Stream_TypeDef类型的hdma.Instance
#define __HAL_DMA_GET_INSTANCE(hdma) ((DMA_Stream_TypeDef *)(hdma.Instance))

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&DBUS_USART,rx1_buf,(uint16_t)(dma_buf_num/2));
}
void RC_unable(void)
{
    __HAL_UART_DISABLE(&DBUS_USART);
}
void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&DBUS_USART);
    __HAL_DMA_DISABLE(&DMA_DBUS_USART_RX);

    __HAL_DMA_GET_INSTANCE(DMA_DBUS_USART_RX)->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&DMA_DBUS_USART_RX);
    __HAL_UART_ENABLE(&DBUS_USART);

}







