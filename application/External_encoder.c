#include "External_encoder.h"
#include "usart.h"
#include "circular_buffer.h"

// //	UART7_RX(PE7) DMA1:数据流3通道5

// volatile unsigned char ENCODER_rx_buffer[2][ENCODER_RX_BUF_NUM];
// volatile unsigned char ENCODER_rx_buffer_8[2][ENCODER_RX_BUF_NUM];

// void Encoder_uart7_Init(void)
// {
//  	GPIO_InitTypeDef GPIO_InitStructure;
// 	USART_InitTypeDef USART_InitStructure;	
// 	NVIC_InitTypeDef NVIC_InitStructure;
// 	DMA_InitTypeDef DMA_InitStructure;
     
// 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
// 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7,ENABLE);
// 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 , ENABLE);
	
// 	RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART7, ENABLE);
//     RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART7, DISABLE);
	
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	                                                                                                                                                                                                                                                                                                         
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
// 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
// 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
// 	GPIO_Init(GPIOE,&GPIO_InitStructure);	
	
// 	GPIO_PinAFConfig(GPIOE,GPIO_PinSource7 ,GPIO_AF_UART7);
	
// 	USART_DeInit(UART7);
	
// 	USART_InitStructure.USART_BaudRate = 115200;	//SBUS 100K baudrate
// 	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
// 	USART_InitStructure.USART_StopBits = USART_StopBits_1;
// 	USART_InitStructure.USART_Parity = USART_Parity_No; 
// 	USART_InitStructure.USART_Mode = USART_Mode_Rx;
// 	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
// 	USART_Init(UART7,&USART_InitStructure);	
	
// 	USART_ClearFlag(UART7, USART_FLAG_IDLE);
// 	USART_ITConfig(UART7, USART_IT_IDLE, ENABLE);
											
// 	USART_DMACmd(UART7,USART_DMAReq_Rx,ENABLE);
// 	USART_Cmd(UART7, ENABLE);	
	
// 	NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = UART7_NVIC; 
// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
// 	NVIC_Init(&NVIC_InitStructure);

// 	DMA_Cmd(DMA1_Stream3, DISABLE);	
// 	 while (DMA1_Stream3->CR & DMA_SxCR_EN); //禁止数据流
	 
// 	DMA_DeInit(DMA1_Stream3);	
// 	DMA_InitStructure.DMA_Channel				= DMA_Channel_5;
// 	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)&(UART7->DR);	
// 	DMA_InitStructure.DMA_Memory0BaseAddr 		= (uint32_t)ENCODER_rx_buffer[0];			
// 	DMA_InitStructure.DMA_DIR 					= DMA_DIR_PeripheralToMemory;									
// 	DMA_InitStructure.DMA_BufferSize 			= 7;	//接收原始数据，为7个字节，给了14个字节长度，防止DMA传输越界															
// 	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;			
// 	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;							
// 	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
// 	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte; 		
// 	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Circular;												
// 	DMA_InitStructure.DMA_Priority 				= DMA_Priority_VeryHigh;							
// 	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
// 	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
// 	DMA_InitStructure.DMA_MemoryBurst 			= DMA_Mode_Normal;
// 	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
// 	DMA_Init(DMA1_Stream3,&DMA_InitStructure);
// 	DMA_DoubleBufferModeConfig(DMA1_Stream3, (uint32_t)ENCODER_rx_buffer[1], DMA_Memory_0);
// 	DMA_DoubleBufferModeCmd(DMA1_Stream3, ENABLE);
// 	DMA_Cmd(DMA1_Stream3, DISABLE); //Add a disable
// 	DMA_Cmd(DMA1_Stream3, ENABLE); 
// }
// //	UART8_TX(PE1) DMA1:数据流0通道5	
// /*	UART8_RX(PE0) DMA1:数据流6通道5*/
// void Encoder_uart8_Init(void)
// {
//  	GPIO_InitTypeDef GPIO_InitStructure;
// 	USART_InitTypeDef USART_InitStructure;	
// 	NVIC_InitTypeDef NVIC_InitStructure;
// 	DMA_InitTypeDef DMA_InitStructure;
     
// 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
// 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8,ENABLE);
// 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 , ENABLE);
	
// 	RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART8, ENABLE);
//     RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART8, DISABLE);
	
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	                                                                                                                                                                                                                                                                                                         
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
// 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
// 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
// 	GPIO_Init(GPIOE,&GPIO_InitStructure);	
	
// 	GPIO_PinAFConfig(GPIOE,GPIO_PinSource0 ,GPIO_AF_UART8);
	
// 	USART_DeInit(UART8);
	
// 	USART_InitStructure.USART_BaudRate = 115200;	//SBUS 100K baudrate
// 	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
// 	USART_InitStructure.USART_StopBits = USART_StopBits_1;
// 	USART_InitStructure.USART_Parity = USART_Parity_No; 
// 	USART_InitStructure.USART_Mode = USART_Mode_Rx;
// 	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
// 	USART_Init(UART8,&USART_InitStructure);	
	
// 	USART_ClearFlag(UART8, USART_FLAG_IDLE);
// 	USART_ITConfig(UART8, USART_IT_IDLE, ENABLE);
											
// 	USART_DMACmd(UART8,USART_DMAReq_Rx,ENABLE);
// 	USART_Cmd(UART8, ENABLE);	
	
// 	NVIC_InitStructure.NVIC_IRQChannel = UART8_IRQn;
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = UART8_NVIC; 
// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
// 	NVIC_Init(&NVIC_InitStructure);

// 	DMA_Cmd(DMA1_Stream6, DISABLE);	
// 	 while (DMA1_Stream6->CR & DMA_SxCR_EN); //禁止数据流
	 
// 	DMA_DeInit(DMA1_Stream6);	
// 	DMA_InitStructure.DMA_Channel				= DMA_Channel_5;
// 	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)&(UART8->DR);	
// 	DMA_InitStructure.DMA_Memory0BaseAddr 		= (uint32_t)ENCODER_rx_buffer_8[0];			
// 	DMA_InitStructure.DMA_DIR 					= DMA_DIR_PeripheralToMemory;									
// 	DMA_InitStructure.DMA_BufferSize 			= 7;	//接收原始数据，为7个字节，给了14个字节长度，防止DMA传输越界															
// 	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;			
// 	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;							
// 	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
// 	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte; 		
// 	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Circular;												
// 	DMA_InitStructure.DMA_Priority 				= DMA_Priority_VeryHigh;							
// 	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
// 	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
// 	DMA_InitStructure.DMA_MemoryBurst 			= DMA_Mode_Normal;
// 	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
// 	DMA_Init(DMA1_Stream6,&DMA_InitStructure);
// 	DMA_DoubleBufferModeConfig(DMA1_Stream6, (uint32_t)ENCODER_rx_buffer_8[1], DMA_Memory_0);
// 	DMA_DoubleBufferModeCmd(DMA1_Stream6, ENABLE);
// 	DMA_Cmd(DMA1_Stream6, DISABLE); //Add a disable
// 	DMA_Cmd(DMA1_Stream6, ENABLE); 
// 
//}

