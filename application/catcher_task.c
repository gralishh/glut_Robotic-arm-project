#include "catcher_task.h"
#include "catcher_task_interface.h"
#include "motor_timer_ctrl.h"
#include "cmsis_os2.h"

CATCHER_TASK_HANDLER_TYPE catcher_task_handler;/*unique structure*/
CATCHER_TASK_HANDLER_TYPE* catcher_task_handler_ptr=&catcher_task_handler;

void catcher_task(void *argument)
{
  catcher_task_init();
  osDelay(1000);
  while(1)
  {
    catcher_task_get_feedback();
    catcher_task_mode_flush();
    catcher_task_set_output();
    catcher_task_output();
    if(!(catcher_task_handler_ptr->tick_count_halt))
      catcher_task_handler_ptr->tick++;
    osDelay(1);
  }
}

