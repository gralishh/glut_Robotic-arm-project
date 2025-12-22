#ifndef __CHASSIS_TASK_INTERFACE__
#define __CHASSIS_TASK_INTERFACE__

void chassis_task_init(void);
void chassis_task_get_feedback(void);
void chassis_task_mode_flush(void);
void chassis_task_set_output(void);
void chassis_task_output(void);


/*根据每个模式定义*/
//void __first_mode_ctrl_func(void);

#endif
