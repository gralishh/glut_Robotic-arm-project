#include "chassis_task.h"
#include "chassis_task_interface.h"
#include "motor_timer_ctrl.h"
#include "cmsis_os2.h"

CHASSIS_TASK_HANDLER_TYPE chassis_task_handler;/*unique structure*/
CHASSIS_TASK_HANDLER_TYPE* chassis_task_handler_ptr=&chassis_task_handler;

void chassis_task(void *argument)
{
  chassis_task_init();
  osDelay(1000);
  while(1)
  {
    chassis_task_get_feedback();
    chassis_task_mode_flush();
    chassis_task_set_output();
    chassis_task_output();

    if(!(chassis_task_handler_ptr->tick_count_halt))
      chassis_task_handler_ptr->tick++;
    osDelay(1);
  }
}
