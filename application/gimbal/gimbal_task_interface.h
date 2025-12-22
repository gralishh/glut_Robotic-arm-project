#ifndef __GIMBAL_TASK_INTERFACE__
#define __GIMBAL_TASK_INTERFACE__

void gimbal_task_init(void);
void gimbal_task_get_feedback(void);
void gimbal_task_mode_flush(void);
void gimbal_task_set_output(void);
void gimbal_task_output(void);

/*根据每个模式定义*/
//void __first_mode_ctrl_func(void);

#endif
