#ifndef __CATCHER_TASK_INTERFACE__
#define __CATCHER_TASK_INTERFACE__

void catcher_task_init(void);
void catcher_task_get_feedback(void);
void catcher_task_mode_flush(void);
void catcher_task_set_output(void);
void catcher_task_output(void);

/*根据每个模式定义*/
//void __first_mode_ctrl_func(void);

#endif
