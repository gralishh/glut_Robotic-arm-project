#include "angle_process.h"

/**
 * @brief 编码值转角度值
 * @param ecd
 * @param max_ecd
 * @param offset_ecd
 * @param offset_angle
 * @note 同时对角度进行归一化处理
 */
fp32 ecd_to_angle(int16_t ecd,int16_t max_ecd, int32_t offset_ecd , fp32 offset_angle)
{
  int32_t relative_ecd = ecd + offset_ecd;
	fp32 relative_angle;

	relative_angle = (relative_ecd*1.0/max_ecd - 0.5) *2* PI + offset_angle;

  if (relative_angle > PI)
	{
		relative_angle = relative_angle - 2*PI;
	}
	else if (relative_angle < -PI)
	{
		relative_angle = relative_angle + 2*PI;
	}
	
	return relative_angle;

}

fp32 angle_normalize(fp32 angle,fp32 offset)
{
  angle=angle-PI+offset;

  if (angle > PI)
	{
		angle = angle - 2*PI;
	}
	else if (angle < -PI)
	{
		angle = angle + 2*PI;
	}
	
	return angle;
}
