#ifndef __EX_ECD__
#define __EX_ECD__

#include "main.h"
#include "struct_typedef.h"

typedef struct __External_ecd_handler_t External_ecd_handler_t;
typedef struct __External_ecd_bus_handler_t External_ecd_bus_handler_t;

typedef enum{
  NONE_ECD=0,
  ECD_USART_COMMUNICATION,
  ECD_CAN_COMMUNICATION
} ecd_protocol_type_e;

typedef enum{
  /*can通信*/
  OID_ECD=1,

  ___DIVIDING__LINE___,
  /*usart通信*/
} ecd_type_e;

struct __External_ecd_handler_t{
  ecd_protocol_type_e protocol_type;
  ecd_type_e type;

  uint32_t ecd;
  fp32 angle;
  uint32_t max_ecd;

  /*CAN总线专用*/
  FDCAN_HandleTypeDef* can;
  uint16_t id;

  /*usart专用*/
  unsigned int (*USART_Recv)(uint8_t *,unsigned short);
};

struct __External_ecd_bus_handler_t{
  External_ecd_handler_t* can_communication_ecd[6];
  External_ecd_handler_t* usart_communication_ecd[3];

  uint16_t can_ecd_count;
  uint16_t usart_ecd_count;
};

/*只能有一个*/
extern External_ecd_bus_handler_t Ex_ecd_bus;

void External_ecd_bus_init(void);
void External_ecd_init_on_can(External_ecd_handler_t* ecd,uint32_t max_ecd,ecd_type_e type,FDCAN_HandleTypeDef* can,uint16_t id);

uint32_t External_ecd_get_value(External_ecd_handler_t* ecd);
fp32 External_ecd_get_angle(External_ecd_handler_t* ecd);
fp32* External_ecd_get_angle_pointer(External_ecd_handler_t* ecd);


void __External_ecd_usart_feedback_hook(void);
void __External_ecd_can_feedback_hook(uint16_t id,uint8_t* msg);

#endif