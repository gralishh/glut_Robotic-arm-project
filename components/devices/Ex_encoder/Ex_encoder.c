#include "Ex_encoder.h"
#include "Ex_ecd_protocol.h"
#include "angle_process.h"
#include "struct_typedef.h"
#include <stdlib.h>
#include <string.h>

External_ecd_bus_handler_t Ex_ecd_bus={0};

void External_ecd_bus_init(void)
{
  /*Ô¤Áô*/
}

void External_ecd_init_on_can(External_ecd_handler_t* ecd,uint32_t max_ecd,ecd_type_e type,FDCAN_HandleTypeDef* can,uint16_t id)
{
  ecd->type=type;
  Ex_ecd_bus.can_communication_ecd[Ex_ecd_bus.can_ecd_count]=ecd
  Ex_ecd_bus.can_ecd_count++;
  ecd->protocol_type=ECD_CAN_COMMUNICATION;

  ecd->can=can;
  ecd->id=id;
  ecd->max_ecd=max_ecd;
}

uint32_t External_ecd_get_value(External_ecd_handler_t* ecd)
{
  return ecd->ecd;
}

fp32 External_ecd_get_angle(External_ecd_handler_t* ecd)
{
  return ecd->angle;
} 

fp32* External_ecd_get_angle_pointer(External_ecd_handler_t* ecd)
{
  return &(ecd->angle);
}

void __External_ecd_can_feedback_hook(uint16_t id,uint8_t* msg)
{
  uint16_t index;
  External_ecd_handler_t* ecd;
  for(index=0;index<Ex_ecd_bus.can_ecd_count;index++)
  {
    ecd=(Ex_ecd_bus.can_communication_ecd)[index];
    switch(ecd)
    {
      case OID_ECD:
        ecd->ecd=((OID_ECD_FEEDBACK_T*)(void*)msg).ecd;
        ecd->angle=NORMALIZE_TO_2PI(ecd_to_angle(ecd->ecd,ecd->max_ecd,0,0.0));
        break
      default
    }
  }
}

