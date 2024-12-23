#ifndef __CUSTOM_CTRL_H__
#define __CUSTOM_CTRL_H__

#include "main.h" 

typedef __packed struct{

} CUSTOM_CTRL_RX_PACK;

typedef struct{
  CUSTOM_CTRL_RX_PACK rx_pack;
} CUSTOM_CTRL_T;
CUSTOM_CTRL_T Custom_Ctrl_handler;

CUSTOM_CTRL_T* Custom_Ctrl_get_ptr(void);
CUSTOM_CTRL_T* Custom_Ctrl_get_rx_pack_ptr(void);
void Custom_Ctrl_unpack(Custom_Ctrl_handler* handler,uint8_t* pack);
void Custom_Ctrl_resolve(Custom_Ctrl_handler* handler);
void Custom_Ctrl_Task(void* para);

#endif
