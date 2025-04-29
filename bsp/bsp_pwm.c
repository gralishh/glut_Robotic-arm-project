#include "bsp_pwm.h"
extern TIM_HandleTypeDef htim2;

/*500:middle 350:down 540:up  350~540*/
void set_pwm1_duty(uint16_t duty)
{
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,duty);
}

/*500:middle  right:150 left:800*/
void set_pwm2_duty(uint16_t duty)
{
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,duty);
}
