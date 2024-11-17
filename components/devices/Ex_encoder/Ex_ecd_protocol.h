#ifndef __EX_ECD_PROTOCOL_H__
#define __EX_ECD_PROTOCOL_H__
#include "struct_typedef.h"

typedef __packed struct{
  uint8_t len;
  uint8_t id;
  uint8_t cmd;
  uint32_t ecd;
  uint8_t REMAIN;
} OID_ECD_FEEDBACK_T;

#endif
