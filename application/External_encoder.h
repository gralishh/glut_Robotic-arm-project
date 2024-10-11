 #ifndef _EXTERNAL_ENCODER_H
#define _EXTERNAL_ENCODER_H
#include "main.h"

//encoder_hand_pitch_2_ID和encoder_haroll_3l_3_ID，分别表示手动推杆轴和手动旋转轴外部编码器的CAN ID。
#define encoder_hand_pitch_2_ID 0x09 	/*回传时间2ms*/
#define encoder_hand_roll_3_ID 0x08		/*回传时间2ms*/

///*********************************************************************************/
//ENCODER_RX_BUF_NUM宏定义了ENCODER_rx_buffer数组的长度，表示每个元素可以存储14个unsigned char类型的数据。
//ENCODER_FRAME_LENGTH宏定义了数据传输时的编码器数据包帧长度（以字节为单位），即一个完整的编码器数据对应7个字节。
//ENCODER_rx_buffer是一个长度为2xENCODER_RX_BUF_NUM的二维数组，用于存储接收到的编码器数据。其中第一维表示两个编码器的数据，第二维表示每个编码器数据有多少个unsigned char类型的数据。
//ENCODER_rx_buffer_8也是一个长度为2xENCODER_RX_BUF_NUM的二维数组，但其每个元素仅能存储8位的数据（而ENCODER_rx_buffer数组每个元素可存储14位数据），只是用于暂存缓冲区的数据
#define ENCODER_RX_BUF_NUM 14u
#define ENCODER_FRAME_LENGTH 7u
extern volatile unsigned char ENCODER_rx_buffer[2][ENCODER_RX_BUF_NUM];
extern volatile unsigned char ENCODER_rx_buffer_8[2][ENCODER_RX_BUF_NUM];

//encoder_ID：uint8_t类型，用于指定编码器通信的CAN ID。
//func_cmd：uint8_t类型，用于命令控制并判断数据包。
//data_lenth：uint8_t类型，用于指定数据块长度。
//ecd：uint16_t类型，表示外部编码器的监测值。
//hand_encoder_offset_ecd：int16_t类型，表示手动控制的偏移量。
typedef struct
{
    uint8_t encoder_ID;
    uint8_t func_cmd;
	uint8_t data_lenth;
    uint16_t ecd;
	int16_t hand_encoder_offset_ecd;
} ENCODER_uart_t;

//const ENCODER_uart_t *get_hand_encoder_hand_pitch_2_Point(void)：该函数返回一个指向已初始化的ENCODER_uart_t类型结构体的常量指针，用于获取手动控制云台俯仰轴的编码器数据。
//const ENCODER_uart_t *get_hand_encoder_hand_roll_3_Point(void)：该函数返回一个指向已初始化的ENCODER_uart_t类型结构体的常量指针，用于获取手动控制云台横滚轴的编码器数据。
//void ENCODER_TO_ECD(volatile const uint8_t *encoder_buf, ENCODER_uart_t *encoder_uart_t)：该函数用于将从编码器接收到的数据转换成所需的ECU清零数据，并存储到给定的ENCODER_uart_t类型结构体中。
//void Encoder_uart7_Init(void)：该函数用于初始化UART7串口，用于接收来自编码器的数据包。
const ENCODER_uart_t *get_hand_encoder_hand_pitch_2_Point(void);
const ENCODER_uart_t *get_hand_encoder_hand_roll_3_Point(void);

// extern void ENCODER_TO_ECD(volatile const uint8_t *encoder_buf, ENCODER_uart_t *encoder_uart_t);



// extern void Encoder_uart7_Init(void);
// extern void Encoder_uart8_Init(void);

#endif

