#include "bsp_buzzer.h"
#include "main.h"
extern TIM_HandleTypeDef htim12;
void buzzer_on(uint16_t psc, uint16_t pwm)
{
	  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	  TIM12->CCR2 = 150;
    //__HAL_TIM_PRESCALER(&htim4, psc);
    //__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
void buzzer_off(void)
{
	  TIM12->CCR2 = 0;
    //__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
