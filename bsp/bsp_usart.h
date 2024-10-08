#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "main.h"
#include "struct_typedef.h"

/*方便以后统一改成uint8_t,虽然可能没必要*/
typedef unsigned char uart_data_t;

//USART1(3pin口)
//UART7(4pin口0)
//UART10(4pin口1)

/*USART1(3pin口)*/
void usart1_init(void);
unsigned int USART1_Send(uint8_t *data, unsigned short len);
unsigned int USART1_Recv(unsigned char *data, unsigned short len);
unsigned char USART1_At( unsigned short offset);
void USART1_Drop( unsigned short LenToDrop);
unsigned int USART1_GetDataCount( void );
void USART1_Free(void);

void USART1_IDLERX_HOOK(UART_HandleTypeDef *huart,uint16_t Size);
void USART1_ERR_HOOK(void);

/*UART7(4pin口0)*/

/*UART10(4pin口1)*/

#endif 
