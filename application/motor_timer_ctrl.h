/**
 * @brief 电机控制定时器
 * @note 建议所有电机控制与pid计算都放入此处
 */
#ifndef __MOTOR_TIMER_CTRL__
#define __MOTOR_TIMER_CTRL__

#include "main.h"
#include "struct_typedef.h"

#define MOTOR_GROUP_ARR_SIZE 10

/*电机分组下标*/
typedef enum{
  HAND_MOTOR=0,
}motor_group_index_t;

typedef enum{
  MOTOR_CMD_DISABLE=0,
  MOTOR_CMD_ENABLE
}motor_group_statue_t;

/**
 * @brief 电机控制句柄类型
 * @note output仅在feedback_cmd为真时生效
 */
typedef struct{
  uint16_t feedback_cmd_list[MOTOR_GROUP_ARR_SIZE];
  uint16_t output_cmd_list[MOTOR_GROUP_ARR_SIZE];
}motor_ctrl_handler_t;


void motor_ctrl_init(void);
void motor_ctrl_feedback_cmd(motor_group_index_t grp_index,motor_group_statue_t statue);
void motor_ctrl_output_cmd(motor_group_index_t grp_index,motor_group_statue_t statue);
void motor_timer_ctrl_callback(void);

#endif
