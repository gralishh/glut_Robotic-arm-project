#include "hand_task.h"
#include "hand_task_interface.h"
#include "motor_timer_ctrl.h"
#include "cmsis_os2.h"

HAND_TASK_HANDLER_TYPE hand_task_handler;/*unique structure*/
HAND_TASK_HANDLER_TYPE* hand_task_handler_ptr=&hand_task_handler;

void hand_task(void *argument)
{
  hand_task_init();
  osDelay(1000);
  while(1)
  {
		
    hand_task_get_feedback();
    hand_task_mode_flush();
    hand_task_set_output();
    hand_task_output();
    if(!(hand_task_handler_ptr->tick_count_halt))
      hand_task_handler_ptr->tick++;
    osDelay(1);
  }
}

