#include "bsp_usart.h"
#include "remote_control.h"
#include "circular_buffer.h"
#include "string.h"


extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;


/*****中断callback函数*****/
/**
 * @brief 接收函数回调
 * 请在其它文件中定义hook函数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size) 
{
  if(huart->Instance==DBUS_UART_OBJ)
    DBUS_IDLERX_HOOK();
  else if(huart->Instance==USART1)
    USART1_IDLERX_HOOK(huart,Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==DBUS_UART_OBJ)
    DBUS_ERR_HOOK();
}


/*****全局设置*****/
#define MAX_RING_BUF_SIZE 4096
#define DMA_DOUBLE_BUFFER_MODE 0


/*****USART1变量*****/
CircBuf_t USART1_RxCBuf,USART1_TxCBuf;/*环形缓冲区句柄*/
#define USART1_BUF_SIZE 1024
/*DMA缓冲区*/
unsigned char USART1_RxBuf0[ USART1_BUF_SIZE ] = {0};
unsigned char USART1_RxBuf1[ USART1_BUF_SIZE ] = {0};
unsigned char USART1_TxBuf[ 300 ] = {0};
/*环形缓冲区*/
unsigned char USART1_TxCBuf_Arr[ MAX_RING_BUF_SIZE ] = {0};
unsigned char USART1_RxCBuf_Arr[ MAX_RING_BUF_SIZE ] = {0};


/*****UART7变量*****/
CircBuf_t UART7_RxCBuf,UART7_TxCBuf;/*环形缓冲区句柄*/
#define USART7_BUF_SIZE 1024
/*DMA缓冲区*/
unsigned char UART7_RxBuf0[ USART7_BUF_SIZE ] = {0};
unsigned char UART7_RxBuf1[ USART7_BUF_SIZE ] = {0};
unsigned char UART7_TxBuf[ 300 ] = {0};
/*环形缓冲区*/
unsigned char UART7_TxCBuf_Arr[ MAX_RING_BUF_SIZE ] = {0};
unsigned char UART7_RxCBuf_Arr[ MAX_RING_BUF_SIZE ] = {0};


/*****UART10(4pin口1)变量*****/
CircBuf_t UART10_RxCBuf,UART10_TxCBuf;/*环形缓冲区句柄*/
#define USART10_BUF_SIZE 1024
/*DMA缓冲区*/
unsigned char UART10_RxBuf0[ USART10_BUF_SIZE ] = {0};
unsigned char UART10_RxBuf1[ USART10_BUF_SIZE ] = {0};
unsigned char UART10_TxBuf[ 300 ] = {0};
/*环形缓冲区*/
unsigned char UART10_TxCBuf_Arr[ MAX_RING_BUF_SIZE ] = {0};
unsigned char UART10_RxCBuf_Arr[ MAX_RING_BUF_SIZE ] = {0};



/*****模板*****/
#define SINGLE_BUF_RX_TEMPLATE(huart_ptr,rx_circbuf,buf_ptr,buf_size,Size) \
    CircBuf_Push(&rx_circbuf,buf_ptr,Size);\
    HAL_UARTEx_ReceiveToIdle_DMA(huart_ptr,buf_ptr,buf_size);\

/**
 * @brief 发送代码模板,
 * 因tx代码结构相同,直接使用宏来方便修改
 */
#define TX_TEMPLATE(huart,tx_circbuf,buf_ptr,buf_size)\
    unsigned int result = HAL_OK;\
    unsigned int isBuffNotEmpty = CircBuf_GetUsedSize(&tx_circbuf);\
    result = CircBuf_Push(&tx_circbuf, data, len);\
    if(isBuffNotEmpty == 0)\
    {\
        len = CircBuf_Pop(&tx_circbuf, buf_ptr, buf_size);\
    }\
    HAL_UART_Transmit_DMA(&huart, buf_ptr, len);\
    return result;\


/*****USART1函数*****/
void usart1_init(void)
{

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1,USART1_RxBuf0,(uint16_t)USART1_BUF_SIZE);
  /*初始化环形缓冲*/
  CircBuf_Init(&USART1_TxCBuf, USART1_TxCBuf_Arr, MAX_RING_BUF_SIZE);  
  CircBuf_Init(&USART1_RxCBuf, USART1_RxCBuf_Arr, MAX_RING_BUF_SIZE);
  __HAL_DMA_ENABLE((&huart1)->hdmarx);
  __HAL_DMA_ENABLE((&huart1)->hdmatx);
}

/**
 * @brief 空闲中断hook函数
 */
void USART1_IDLERX_HOOK(UART_HandleTypeDef *huart,uint16_t Size)
{
  SINGLE_BUF_RX_TEMPLATE(huart,USART1_RxCBuf, USART1_RxBuf0, USART1_BUF_SIZE,Size);
}

/**
 * @brief 错误中断hook函数
 */
void USART1_ERR_HOOK(void)
{
  memset(USART1_RxBuf0,0,USART1_BUF_SIZE);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1,USART1_RxBuf0,USART1_BUF_SIZE);
}

unsigned int USART1_Send(uint8_t *data, unsigned short len)
{
  TX_TEMPLATE(huart1,USART1_TxCBuf,USART1_TxBuf,USART1_BUF_SIZE);
}

unsigned int USART1_Recv(unsigned char *data, unsigned short len)
{    
  unsigned int result = 0;

  if(data != NULL)
      result = CircBuf_Pop(&USART1_RxCBuf, data, len);

  return result;
}

unsigned char USART1_At( unsigned short offset)
{
    return CircBuf_At(&USART1_RxCBuf, offset);
}

void USART1_Drop( unsigned short LenToDrop)
{
    CircBuf_Drop(&USART1_RxCBuf, LenToDrop);
}

unsigned int USART1_GetDataCount( void )
{
    return CircBuf_GetUsedSize(&USART1_RxCBuf);
}

void USART1_Free(void)
{
    CircBuf_Free(&USART1_RxCBuf);
}

/*****UART7函数*****/


/*****UART10函数*****/
