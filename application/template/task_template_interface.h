/**
 * @brief 模板任务接口
 * @author C_XIAOBAI
 * @note 添加"添加模式"的规范
 * @details 
 * 特定机构控制的类型分为两种:
 *             电机直接控制
 * ctrl_type <
 *             关节控制
 * 两者主要为变量的映射过程不同,主要解决一个关节状态对应两个或两个以上的电机的问题，以及机构实际位置与电机编码器旋转角度的映射问题
 * 
 * 
 * 以下讲述使用此模板的方法
 * 
 * ||添加电机类型
 * 1. MOTOR_TYPE_E中添加该电机类型
 * 2. 在temp_task_get_feedback中添加该类型的case
 * 3. 在temp_task_output中添加该类型的case，并分为Motor_Ctrl_mode_e中状态进行输出
 * 
 * ||添加电机:
 * 1. 在MOTOR_INDEX中添加电机标签
 * 2. 初始化电机实例,并将实例指针赋值给motor_instance
 * 3. 设定电机类型
 * 之后可以直接使用__SET_MOTOR_<ITEM>()来控制电机
 * 
 * ||添加关节:
 * 1. 在JOINT_INDEX中添加关节标签
 * 2. 在temp_task_get_feedback电机反馈后添加电机反馈对关节状态的映射(赋值)
 * 3. 在temp_task_output的开头添加关节状态对对应电机状态的映射
 * 之后可以直接使用__SET_JOINT_<ITEM>()来控制关节
 * 
 * ||添加模式:
 * 1. 在CTRL_MODE中添加模式标签
 * 2. 在temp_task_mode_flash定义模式对应拨杆状态
 * 3. 定义该模式执行的钩子函数
 * 4. 在temp_task_set_output中添加对应case* 
 * 
 * @todo 
 *  - 将设置电机状态宏函数放入电机控制量中
 *  - 将INDEX类枚举设为私有
 *  - 添加固定的控制模式:remote_ctrl|auto|lock|non-force
 *  - 添加遥控器未响应时强制进入失能状态功能
 *  - 添加模式切换进入自锁状态过渡功能(防止位置跳变)
 */
#ifndef __TASK_TEMPLATE_INTERFACE__
#define __TASK_TEMPLATE_INTERFACE__

#include "task_template.h"

void temp_task_init(void);
void temp_task_get_feedback(void);
void temp_task_mode_flush(void);
void temp_task_set_output(void);
void temp_task_output(void);

/*根据每个模式定义*/
void __first_mode_ctrl_func(void);

#endif
