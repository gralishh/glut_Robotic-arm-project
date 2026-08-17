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
void USART1_TX_CPLT_HOOK(DMA_HandleTypeDef* hdma);


/*UART7(4pin口0)*/
void uart7_init(void);
unsigned int UART7_Send(uint8_t *data, unsigned short len);
unsigned int UART7_Recv(unsigned char *data, unsigned short len);
unsigned char UART7_At( unsigned short offset);
void UART7_Drop( unsigned short LenToDrop);
unsigned int UART7_GetDataCount( void );
void UART7_Free(void);

void UART7_IDLERX_HOOK(UART_HandleTypeDef *huart,uint16_t Size);
void UART7_ERR_HOOK(void);
void UART7_TX_CPLT_HOOK(DMA_HandleTypeDef* hdma);


/*USART10(4pin口1)*/
void usart10_init(void);
unsigned int USART10_Send(uint8_t *data, unsigned short len);
unsigned int USART10_Recv(unsigned char *data, unsigned short len);
unsigned char USART10_At( unsigned short offset);
void USART10_Drop( unsigned short LenToDrop);
unsigned int USART10_GetDataCount( void );
void USART10_Free(void);

void USART10_IDLERX_HOOK(UART_HandleTypeDef *huart,uint16_t Size);
void USART10_ERR_HOOK(void);
void USART10_TX_CPLT_HOOK(DMA_HandleTypeDef* hdma);


#endif 
