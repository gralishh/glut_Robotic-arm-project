#include "hand_task.h"
#include "hand_task_interface.h"
#include "motor_timer_ctrl.h"

void hand_task(void *argument)
{
  hand_task_init();
  osDelay(1000)
  while(1)
  {
    hand_task_get_feedback();
    hand_task_mode_flush();
    hand_task_set_output();
    hand_task_output();
    osDelay(1);
  }
}

