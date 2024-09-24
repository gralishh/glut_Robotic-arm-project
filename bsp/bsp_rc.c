#include "bsp_rc.h"
#include "main.h"

//获取DMA_Stream_TypeDef类型的hdma.Instance
#define __HAL_DMA_GET_INSTANCE(hdma) ((DMA_Stream_TypeDef *)(hdma.Instance))

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(DBUS_USART.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&DBUS_USART, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&DMA_DBUS_USART_RX);
    while(__HAL_DMA_GET_INSTANCE(DMA_DBUS_USART_RX)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&DMA_DBUS_USART_RX);
    }

    __HAL_DMA_GET_INSTANCE(DMA_DBUS_USART_RX)->PAR = (uint32_t) & (DBUS_UART_OBJ->RDR);
    //memory buffer 1
    //内存缓冲区1
    __HAL_DMA_GET_INSTANCE(DMA_DBUS_USART_RX)->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    __HAL_DMA_GET_INSTANCE(DMA_DBUS_USART_RX)->M1AR = (uint32_t)(rx1_buf);
    //data length
    //数据长度
    __HAL_DMA_GET_INSTANCE(DMA_DBUS_USART_RX)->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(__HAL_DMA_GET_INSTANCE(DMA_DBUS_USART_RX)->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&DMA_DBUS_USART_RX);


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







