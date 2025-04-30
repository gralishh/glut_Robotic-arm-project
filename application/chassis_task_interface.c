#include "chassis_task_interface.h"
#include "chassis_task.h"
#include "general_motor_module.h" 
#include "DJI_motor_canbus.h"
#include "remote_control.h" 
#include "angle_process.h"
#include "cmsis_os2.h"
#include "detect_task.h"
#include "Custom_ctrl.h"

#define HANDLER chassis_task_handler
#define HANDLER_PTR chassis_task_handler_ptr
#define RC_CTRL_PTR (get_remote_control_point())

/*extern*/
extern FDCAN_HandleTypeDef hfdcan1;

/*global macro variable*/
// joint mapping parameter

// controller sensity(degree per loop)
#define VX_CTRL_SEN 4.23f
#define VY_CTRL_SEN 4.23f
#define WZ_CTRL_SEN 9.0f
// chassis para
#define CHASSIS_WZ_SET_SCALE 0.03f
#define MOTOR_DISTANCE_TO_CENTER 0.3f

/*global motor handler*/
DJI_Motor_Ctrl_t DJI_Motor_LeftFront;
DJI_Motor_Ctrl_t DJI_Motor_RightFront;
DJI_Motor_Ctrl_t DJI_Motor_LeftBack;
DJI_Motor_Ctrl_t DJI_Motor_RightBack;

static void __chassis_nonforce(void);
static void __chassis_idle_ctrl(void);
static void __chassis_rc_ctrl(void);
static void __chassis_rc_slow_ctrl(void);
static void __chassis_union_rc_ctrl(void);


/*general handler method*/
/** 
 * macro name format:
 *  __<GET/SET>_<MOTOR/JOINT>_<ITEM>(index[,value])
 */
/*获取电机状态*/
#define __GET_MOTOR_INSTANCE(index) (HANDLER_PTR->motor_instance[index])
#define __SET_MOTOR_INSTANCE(index,instance_ptr) (HANDLER_PTR->motor_instance[index]=((void*)instance_ptr))
#define __GET_STRUCT_MODE() (HANDLER_PTR->ctrl_mode)
#define __SET_STRUCT_MODE(value) (HANDLER_PTR->ctrl_mode=value)

#define __GET_MOTOR_TYPE(index) (HANDLER_PTR->motor_type[index])
#define __SET_MOTOR_TYPE(index,type) (HANDLER_PTR->motor_type[index]=(type))
#define __GET_MOTOR_CTRL_MODE(index) (HANDLER_PTR->motor_ctrl_mode[index])

/*获取电机反馈*/
#define __GET_MOTOR_ANGLE(index) (HANDLER_PTR->feedback_motor_angle[index])
#define __GET_MOTOR_SPEED(index) (HANDLER_PTR->feedback_motor_speed[index])
#define __GET_MOTOR_CURRENT(index) (HANDLER_PTR->feedback_motor_current[index])

/*电机输出控制*/
#define __SET_MOTOR_ANGLE(index,value) {(HANDLER_PTR->motor_angle[index]=value);(HANDLER_PTR->motor_ctrl_mode[index]=POS_LOOP);}
#define __ADD_MOTOR_ANGLE(index,value) {(HANDLER_PTR->motor_angle[index]+=value);(HANDLER_PTR->motor_ctrl_mode[index]=POS_LOOP);}
#define __SET_MOTOR_SPEED(index,value) {(HANDLER_PTR->motor_speed[index]=value);(HANDLER_PTR->motor_ctrl_mode[index]=SPEED_LOOP);}
#define __SET_MOTOR_CURRENT(index,value) {(HANDLER_PTR->motor_current[index]=value);(HANDLER_PTR->motor_ctrl_mode[index]=GIVING_CURRENT);}
#define __SET_MOTOR_LOCKUP(index) (HANDLER_PTR->motor_ctrl_mode[index]=LOCK)
#define __SET_MOTOR_NONFORCE(index) (HANDLER_PTR->motor_ctrl_mode[index]=NON_FORCE)

/*关节控制*/
#define __GET_JOINT_ANGLE(index) (HANDLER_PTR->feedback_joint_angle[index])
#define __SET_JOINT_LIMIT(index,min,max) {(HANDLER_PTR->max_joint_angle[index]=max);(HANDLER_PTR->min_joint_angle[index]=min);}
#define __JOINT_LIMIT(index,value) angle_limit(value,HANDLER_PTR->max_joint_angle[index],HANDLER_PTR->min_joint_angle[index])
#define __SET_JOINT_ANGLE(index,value) {(HANDLER_PTR->joint_angle[index]=__JOINT_LIMIT(index,value));(HANDLER_PTR->motor_ctrl_mode[index]=POS_LOOP);}
#define __ADD_JOINT_ANGLE(index,value) {(HANDLER_PTR->joint_angle[index]=__JOINT_LIMIT(index,HANDLER_PTR->joint_angle[index]+(value)));(HANDLER_PTR->motor_ctrl_mode[index]=POS_LOOP);}

/*时间控制*/
#define __RESET_TICKS() (HANDLER_PTR->tick=0)
#define __HALT_TICKS_COUNTING() (HANDLER_PTR->tick_count_halt=1)
#define __HOLD_TICKS_COUNTING() (HANDLER_PTR->tick_count_halt=0)
#define __GET_TICKS() (HANDLER_PTR->tick)
// unit:seconds
#define __GET_TICKS_TIME() (HANDLER_PTR->tick*1)
#define __GET_PROCESS_PERCENTAGE(PROCESS_TIME) (__GET_TICKS_TIME()/PROCESS_TIME)
#define __IS_MODE_SWITCHED() (1==HANDLER_PTR->mode_switch)


void chassis_task_init()
{
  /*基础初始化*/
  __HALT_TICKS_COUNTING();
  __RESET_TICKS();

  /*电机初始化*/
  __SET_MOTOR_INSTANCE(DJI_LF,&DJI_Motor_LeftFront);
  __SET_MOTOR_TYPE(DJI_LF,DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_LeftFront,&DJI_CAN1_Bus_ctrl,M3508,0x201);
  DJI_Motor_Speed_PID_init(&DJI_Motor_LeftFront,PID_POSITION,22,0,0.00,5000,0);
  DJI_Motor_Pos_PID_init(&DJI_Motor_LeftFront,PID_POSITION,15,0,0,1000,0);

  __SET_MOTOR_INSTANCE(DJI_RF,&DJI_Motor_RightFront);
  __SET_MOTOR_TYPE(DJI_RF,DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_RightFront,&DJI_CAN1_Bus_ctrl,M3508,0x202);
  DJI_Motor_Speed_PID_init(&DJI_Motor_RightFront,PID_POSITION,28,0,0.00,5000,0);
  DJI_Motor_Pos_PID_init(&DJI_Motor_RightFront,PID_POSITION,15,0,0,1000,0);

  __SET_MOTOR_INSTANCE(DJI_LB,&DJI_Motor_LeftBack);
  __SET_MOTOR_TYPE(DJI_LB,DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_LeftBack,&DJI_CAN1_Bus_ctrl,M3508,0x204);
  DJI_Motor_Speed_PID_init(&DJI_Motor_LeftBack,PID_POSITION,22,0,0.00,5000,0);
  DJI_Motor_Pos_PID_init(&DJI_Motor_LeftBack,PID_POSITION,15,0,0,1000,0);

  __SET_MOTOR_INSTANCE(DJI_RB,&DJI_Motor_RightBack);
  __SET_MOTOR_TYPE(DJI_RB,DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_RightBack,&DJI_CAN1_Bus_ctrl,M3508,0x203);
  DJI_Motor_Speed_PID_init(&DJI_Motor_RightBack,PID_POSITION,22,0,0.00,5000,0);
  DJI_Motor_Pos_PID_init(&DJI_Motor_RightBack,PID_POSITION,15,0,0,1000,0);

  __chassis_idle_ctrl();
  //DJI_CANBus_enable_bus(&DJI_CAN1_Bus_ctrl);
}

/**
 * @brief 刷新句柄的反馈值
 * @details 遍历电机控制句柄，针对点击类型刷新反馈
 * @code 
 */
void chassis_task_get_feedback()
{
  uint16_t index;

  /*motor feedback*/
  for(index=0;index<CHASSIS_MOTOR_COUNT;index++)
  {
    GENERAL_MOTOR_GET_FEEDBACK(__GET_MOTOR_INSTANCE(index),
      __GET_MOTOR_TYPE(index),
      &__GET_MOTOR_CURRENT(index),
      &__GET_MOTOR_SPEED(index),
      &__GET_MOTOR_ANGLE(index)
    )
  }

  /*joint angle map*/
  /*
  __GET_JOINT_ANGLE(index,
    ...
  )
  ...
  */
}

/**
 * @brief 模式状态刷新
 * @details 根据控制器拨杆刷新模式(二级模式会与UI耦合)
 */
void chassis_task_mode_flush()
{
  /**/
  static uint8_t last_mode=CHASSIS_MODE_NONFORCE;
  last_mode=HANDLER_PTR->ctrl_mode;
  if(switch_is_down(get_remote_control_point()->rc.s[1]))
  {
    if(switch_is_mid(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(CHASSIS_MODE_RC_CTRL);
    else if(switch_is_up(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(CHASSIS_MODE_RC_CTRL);
    
  }
  else if(switch_is_mid(get_remote_control_point()->rc.s[1]))
  {
    if(switch_is_mid(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(CHASSIS_MODE_UNION_CTRL);
    else 
      __SET_STRUCT_MODE(CHASSIS_MODE_IDLE);
  }
  else if(switch_is_up(get_remote_control_point()->rc.s[1]))
  {
    if(switch_is_mid(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(CHASSIS_MODE_RC_CTRL);
    else 
      __SET_STRUCT_MODE(CHASSIS_MODE_IDLE);
  }
  else
  {
    __SET_STRUCT_MODE(CHASSIS_MODE_IDLE);
  }

  if(switch_is_down(get_remote_control_point()->rc.s[1]) && switch_is_down(get_remote_control_point()->rc.s[0]))
  {
    __SET_STRUCT_MODE(CHASSIS_MODE_NONFORCE);
  }

	if(toe_is_error(DBUSTOE))
  {
    __SET_STRUCT_MODE(CHASSIS_MODE_NONFORCE);
  }

  if(HANDLER_PTR->ctrl_mode==last_mode)
    HANDLER_PTR->mode_switch=0;
  else 
    HANDLER_PTR->mode_switch=1;

}

/**
 * @brief 设置输出量(电流|速度|位置|力矩)
 * @details 模式控制 
 */
void chassis_task_set_output()
{
  switch(__GET_STRUCT_MODE())
  {
    case CHASSIS_MODE_IDLE:
      __chassis_idle_ctrl();
      break;
    case CHASSIS_MODE_RC_CTRL:
      __chassis_rc_ctrl();
      break;
    case CHASSIS_MODE_UNION_CTRL:
      __chassis_union_rc_ctrl();
      break;
    case CHASSIS_MODE_NONFORCE:
    default:
      __chassis_nonforce();
  }
}

/**
 * @brief 控制输出
 * @details 根据电机种类与控制状态设定输出
 */
void chassis_task_output()
{
  uint16_t index;
  /*joint map to motor state*/

  /*motor output*/
  for(index=0;index<CHASSIS_MOTOR_COUNT;index++)
  {
    GENERAL_MOTOR_SET_OUTPUT(__GET_MOTOR_INSTANCE(index),
    __GET_MOTOR_TYPE(index),
    __GET_MOTOR_CTRL_MODE(index),
    HANDLER_PTR->motor_current[index],
    HANDLER_PTR->motor_speed[index],
    HANDLER_PTR->motor_angle[index]
    )
  }
}

void __chassis_nonforce()
{
  int index;
  for(index=0;index<CHASSIS_MOTOR_COUNT;index++)
  {
    __SET_MOTOR_NONFORCE(index);
  }
}

void __chassis_idle_ctrl()
{
  int index;

  if(__IS_MODE_SWITCHED())
  {
    for(index=0;index<CHASSIS_MOTOR_COUNT;index++)
    {
      __SET_MOTOR_ANGLE(index,__GET_MOTOR_ANGLE(index));
    }
  }

  for(index=0;index<CHASSIS_MOTOR_COUNT;index++)
  {
    __ADD_MOTOR_ANGLE(index,0);
  }
}

void __chassis_rc_ctrl()
{
  /*设置输入速度(not real)*/
  HANDLER_PTR->vx=-RC_CTRL_PTR->rc.ch[3]*VX_CTRL_SEN;
  HANDLER_PTR->vy=-RC_CTRL_PTR->rc.ch[2]*VY_CTRL_SEN;
  HANDLER_PTR->wz=-RC_CTRL_PTR->rc.ch[0]*WZ_CTRL_SEN;

  if(GET_KEYBOARD_KEY(KEY_W))
    HANDLER_PTR->vx=-770*VX_CTRL_SEN;
  if(GET_KEYBOARD_KEY(KEY_S))
    HANDLER_PTR->vx=770*VX_CTRL_SEN;
  if(GET_KEYBOARD_KEY(KEY_A))
    HANDLER_PTR->vy=770*VY_CTRL_SEN;
  if(GET_KEYBOARD_KEY(KEY_D))
    HANDLER_PTR->vy=-770*VY_CTRL_SEN;
  if(GET_KEYBOARD_KEY(KEY_Q))
    HANDLER_PTR->wz=660*WZ_CTRL_SEN;
  if(GET_KEYBOARD_KEY(KEY_E))
    HANDLER_PTR->wz=-660*WZ_CTRL_SEN;

  if(remote_data.mouse_x!=0)
    HANDLER_PTR->wz=-remote_data.mouse_x*10*WZ_CTRL_SEN;


  __SET_MOTOR_SPEED(DJI_LF, - HANDLER_PTR->vx - HANDLER_PTR->vy + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  __SET_MOTOR_SPEED(DJI_RF,   HANDLER_PTR->vx - HANDLER_PTR->vy + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  __SET_MOTOR_SPEED(DJI_RB,   HANDLER_PTR->vx + HANDLER_PTR->vy + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  __SET_MOTOR_SPEED(DJI_LB, - HANDLER_PTR->vx + HANDLER_PTR->vy + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
}

void __chassis_rc_slow_ctrl(void)
{
  /*设置输入速度(not real)*/
  HANDLER_PTR->vx=-RC_CTRL_PTR->rc.ch[3]*VX_CTRL_SEN*0.4f;
  HANDLER_PTR->vy=-RC_CTRL_PTR->rc.ch[2]*VY_CTRL_SEN*0.4f;
  HANDLER_PTR->wz=-RC_CTRL_PTR->rc.ch[0]*WZ_CTRL_SEN*0.9f;

  __SET_MOTOR_SPEED(DJI_LF, - HANDLER_PTR->vx - HANDLER_PTR->vy + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  __SET_MOTOR_SPEED(DJI_RF,   HANDLER_PTR->vx - HANDLER_PTR->vy + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  __SET_MOTOR_SPEED(DJI_RB,   HANDLER_PTR->vx + HANDLER_PTR->vy + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  __SET_MOTOR_SPEED(DJI_LB, - HANDLER_PTR->vx + HANDLER_PTR->vy + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
}

void __chassis_union_rc_ctrl()
{
  /*设置输入速度(not real)*/
  HANDLER_PTR->vx=-RC_CTRL_PTR->rc.ch[3]*VX_CTRL_SEN;
  HANDLER_PTR->vy=-RC_CTRL_PTR->rc.ch[2]*VY_CTRL_SEN;
  HANDLER_PTR->wz=0;

  if(GET_KEYBOARD_KEY(KEY_W))
    HANDLER_PTR->vx=-220*VX_CTRL_SEN;
  if(GET_KEYBOARD_KEY(KEY_S))
    HANDLER_PTR->vx=220*VX_CTRL_SEN;
  if(GET_KEYBOARD_KEY(KEY_A))
    HANDLER_PTR->vy=220*VY_CTRL_SEN;
  if(GET_KEYBOARD_KEY(KEY_D))
    HANDLER_PTR->vy=-220*VY_CTRL_SEN;
  if(GET_KEYBOARD_KEY(KEY_Q))
    HANDLER_PTR->wz=220*WZ_CTRL_SEN;
  if(GET_KEYBOARD_KEY(KEY_E))
    HANDLER_PTR->wz=-330*WZ_CTRL_SEN;

  __SET_MOTOR_SPEED(DJI_LF, - HANDLER_PTR->vx*0.5 - HANDLER_PTR->vy*0.5 + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  __SET_MOTOR_SPEED(DJI_RF,   HANDLER_PTR->vx*0.5 - HANDLER_PTR->vy*0.5 + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  __SET_MOTOR_SPEED(DJI_RB,   HANDLER_PTR->vx*0.5 + HANDLER_PTR->vy*0.5 + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
  __SET_MOTOR_SPEED(DJI_LB, - HANDLER_PTR->vx*0.5 + HANDLER_PTR->vy*0.5 + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * HANDLER_PTR->wz);
}

#undef HANDLER 
#undef HANDLER_PTR 

