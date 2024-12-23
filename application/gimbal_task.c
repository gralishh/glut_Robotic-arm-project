#include "gimbal_task.h"
#include "gimbal_task_interface.h"
#include "motor_timer_ctrl.h"
#include "cmsis_os2.h"

GIMBAL_TASK_HANDLER_TYPE gimbal_task_handler;/*unique structure*/
GIMBAL_TASK_HANDLER_TYPE* gimbal_task_handler_ptr=&gimbal_task_handler;

void gimbal_task(void *argument)
{
  gimbal_task_init();
  osDelay(1000);
  while(1)
  {
    gimbal_task_get_feedback();
    gimbal_task_mode_flush();
    gimbal_task_set_output();
    gimbal_task_output();
    if(!(gimbal_task_handler_ptr->tick_count_halt))
      gimbal_task_handler_ptr->tick++;
    osDelay(1);
  }
}
