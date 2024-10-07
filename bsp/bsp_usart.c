#include "bsp_usart.h"
#include "remote_control.h"

/**
 * @brief 接收函数回调
 * 请在其它文件中定义hook函数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size) 
{
  if(huart->Instance==DBUS_UART_OBJ)
    DBUS_IDLERX_HOOK();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==DBUS_UART_OBJ)
    DBUS_ERR_HOOK();
}
