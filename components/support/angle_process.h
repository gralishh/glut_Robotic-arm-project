/**
 * @brief 角度处理函数库
 */
#ifndef __ANGLE_PROCESS__
#define __ANGLE_PROCESS__

#include "main.h"
#include "struct_typedef.h"

#define PI 3.14159f

fp32 ecd_to_angle(int16_t ecd,int16_t max_ecd, int32_t offset_ecd , fp32 offset_angle);
fp32 angle_normalize(fp32 angle,fp32 offset);

#endif
