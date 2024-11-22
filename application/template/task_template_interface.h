/**
 * @brief 模板任务接口
 * @note 添加"添加模式"的规范
 */
#ifndef __TASK_TEMPLATE_INTERFACE__
#define __TASK_TEMPLATE_INTERFACE__

#include "task_template.h"

void temp_task_init(void);
void temp_task_get_feedback(void);
void temp_task_mode_flash(void);
void temp_task_set_output(void);
void temp_task_output(void);

/*根据每个模式定义*/
void __first_mode_ctrl_func(void);

#endif
