#ifndef __HAND_TASK_INTERFACE__
#define __HAND_TASK_INTERFACE__

void hand_task_init(void);
void hand_task_get_feedback(void);
void hand_task_mode_flush(void);
void hand_task_set_output(void);
void hand_task_output(void);

/*根据每个模式定义*/
//void __first_mode_ctrl_func(void);

//reset_para
//J2
#define J2_reset_speed_Hvalue 0.0023f
#define J2_reset_speed_Lvalue 0.0008f

#endif
