/**
 * @brief CRC for custom controller
 * @see Custom_ctrl.h
 */
#ifndef __CRC_H__
#define __CRC_H__

#include "main.h"
#include "string.h"

extern void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

extern void Append_CRC16_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

#endif

