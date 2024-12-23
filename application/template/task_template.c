#include "task_template.h"
#include "task_template_interface.h"
#include "task.h"

void template_task(void *argument)
{
  temp_task_init();
  osDelay(1000)
  while(1)
  {
    temp_task_get_feedback();
    temp_task_mode_flush();
    temp_task_set_output();
    temp_task_output();
    osDelay(1);
  }
}
