/**
 * @brief 
 * @attention lockup的逻辑有大🐱饼
 */
#include "hand_task_interface.h"
#include "hand_task.h"
#include "general_motor_module.h" 
#include "DJI_motor_canbus.h"
#include "remote_control.h" 
#include "angle_process.h"
#include "detect_task.h"
#include "cmsis_os2.h"
#include "ws2812.h"

#define HANDLER hand_task_handler
#define HANDLER_PTR hand_task_handler_ptr
#define RC_CTRL_PTR (get_remote_control_point())

/*extern*/
extern FDCAN_HandleTypeDef hfdcan2;

/*global macro variable*/
#define HAND_CTRL_CAN 
// joint mapping parameter
#define J1_MAP_K   1
#define J1_MAP_D   0
#define J2_MAP_K   1
#define J2_MAP_D   0
#define J3_MAP_K   1
#define J3_MAP_D   0
#define HEL_MAP_K  1
#define HEL_MAP_D  0
#define HER_MAP_K  1
#define HER_MAP_D  0
// controller sensity(degree per loop)
#define J1_CTRL_SEN 0
#define J2_CTRL_SEN 0
#define J3_CTRL_SEN 0
#define PITCH_CTRL_SEN 0
#define ROLL_CTRL_SEN 0

/*global motor handler*/
extern M8010_motor_t joint1_motor;
extern Joint_Motor_t DM_Motor_J2;
DJI_Motor_Ctrl_t DJI_Motor_J3;
DJI_Motor_Ctrl_t DJI_Motor_headendL;
DJI_Motor_Ctrl_t DJI_Motor_headendR;

static void __hand_nonforce(void);
static void __hand_idle_ctrl(void);
static void __hand_rc_ctrl(void);


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


void hand_task_init()
{
  osDelay(3000);
  WS2812_Ctrl(30,100,50);

  /*基础初始化*/
  __HALT_TICKS_COUNTING();
  __RESET_TICKS();

  /*电机初始化*/
  // J1
  __SET_MOTOR_INSTANCE(M8010_J1,&joint1_motor);
  __SET_MOTOR_TYPE(M8010_J1,M8010_MOTOR);
  M8010_motor_init(&joint1_motor,3,0.8,0.075);

  // J2
  __SET_MOTOR_INSTANCE(DM_J2,&DM_Motor_J2);
  __SET_MOTOR_TYPE(DM_J2,M4310_MOTOR);
  for(int i=0;i<10;i++)
  {
    disable_motor_mode(&hfdcan2,1,MIT_MODE);
    osDelay(20);
  }
  for(int i=0;i<40;i++)
  {
    enable_motor_mode(&hfdcan2,1,POS_MODE);
    osDelay(20);
  }
  joint_motor_init(&DM_Motor_J2,1,POS_MODE,1.0,1.0);

  // J3
  __SET_MOTOR_INSTANCE(DJI_J3,&DJI_Motor_J3);
  __SET_MOTOR_TYPE(DJI_J3,DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_J3,&DJI_CAN2_Bus_ctrl,M3508,0x204);
  //DJI_Motor_set_angle_limit()
  //DJI_Motor_set_speed_limit()
  DJI_Motor_Speed_PID_init(&DJI_Motor_J3,PID_POSITION,20,0.000,0,10000.000,800);
  DJI_Motor_Pos_PID_init(&DJI_Motor_J3,PID_POSITION,55,0.0,0.0,500,100);
  DJI_Motor_J3.circle_count_flag=1;

  // Headend_L
  __SET_MOTOR_INSTANCE(DJI_HE_L,&DJI_Motor_headendL);
  __SET_MOTOR_TYPE(DJI_HE_L,DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_headendL,&DJI_CAN2_Bus_ctrl,M2006,0x201);
  DJI_Motor_Speed_PID_init(&DJI_Motor_headendL,PID_POSITION,22,0.001,0,9000,1000);
  DJI_Motor_Pos_PID_init(&DJI_Motor_headendL,PID_POSITION,60,0,0,500,1000);
  DJI_Motor_headendL.circle_count_flag=1;

  // Headend_R
  __SET_MOTOR_INSTANCE(DJI_HE_R,&DJI_Motor_headendR);
  __SET_MOTOR_TYPE(DJI_HE_R,DJI_MOTOR);
  DJI_Motor_init(&DJI_Motor_headendR,&DJI_CAN2_Bus_ctrl,M2006,0x208);
  DJI_Motor_Speed_PID_init(&DJI_Motor_headendR,PID_POSITION,22,0.001,0,9000,1000);
  DJI_Motor_Pos_PID_init(&DJI_Motor_headendR,PID_POSITION,65,0,0,500,0);
  DJI_Motor_headendR.circle_count_flag=1;

  for(int i=0;i<40;i++)
  {
    osDelay(20);
    hand_task_get_feedback();
    __hand_idle_ctrl();
  }
  DJI_CANBus_enable_bus(&DJI_CAN2_Bus_ctrl);
}

/**
 * @brief 刷新句柄的反馈值
 * @details 遍历电机控制句柄，针对点击类型刷新反馈
 * @code 
 */
void hand_task_get_feedback()
{
  uint16_t index;

  /*motor feedback*/
  for(index=0;index<HAND_MOTOR_COUNT;index++)
  {
    GENERAL_MOTOR_GET_FEEDBACK(__GET_MOTOR_INSTANCE(index),
      __GET_MOTOR_TYPE(index),
      &__GET_MOTOR_CURRENT(index),
      &__GET_MOTOR_SPEED(index),
      &__GET_MOTOR_ANGLE(index)
    )
  }

  /*joint angle map*/
  __GET_JOINT_ANGLE(HAND_J1)=J1_MAP_K*__GET_MOTOR_ANGLE(M8010_J1) +J1_MAP_D;
  __GET_JOINT_ANGLE(HAND_J2)=J2_MAP_K*__GET_MOTOR_ANGLE(DM_J2)    +J2_MAP_D;
  __GET_JOINT_ANGLE(HAND_J3)=J3_MAP_K*__GET_MOTOR_ANGLE(DJI_J3)   +J3_MAP_D;
  __GET_JOINT_ANGLE(HAND_PITCH);
  __GET_JOINT_ANGLE(HAND_ROLL);
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
void hand_task_mode_flush()
{
  /**/
  if(switch_is_up(get_remote_control_point()->rc.s[1]))
  {
    if(switch_is_down(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(HAND_MODE_IDLE);
    else if(switch_is_mid(get_remote_control_point()->rc.s[0]))
      __SET_STRUCT_MODE(HAND_MODE_RC_CTRL);
    else if(switch_is_up(get_remote_control_point()->rc.s[0]))
      ;
  }
  else if(switch_is_down(get_remote_control_point()->rc.s[1]) && switch_is_down(get_remote_control_point()->rc.s[0]))
  {
    __SET_STRUCT_MODE(HAND_MODE_NONFORCE);
  }
  else
  {
    __SET_STRUCT_MODE(HAND_MODE_IDLE);
  }

	if(toe_is_error(DBUSTOE))
  {
    __SET_STRUCT_MODE(HAND_MODE_NONFORCE);
  }
  /*
  if(get_remote_control_point()->rc.s[0]==0 && get_remote_control_point()->rc.s[1]==0)
  {
    __SET_STRUCT_MODE(first_mode);
  }
  */
}

/**
 * @brief 设置输出量(电流|速度|位置|力矩)
 * @details 模式控制 
 */
void hand_task_set_output()
{
  switch(__GET_STRUCT_MODE())
  {
    case HAND_MODE_IDLE:
      __hand_idle_ctrl();
      break;
    case HAND_MODE_RC_CTRL:
      __hand_rc_ctrl();
      break;
    case HAND_MODE_NONFORCE:
    default:
      __hand_nonforce();
  }
}

/**
 * @brief 控制输出
 * @details 根据电机种类与控制状态设定输出
 */
void hand_task_output()
{
  uint16_t index;
  /*joint map to motor state*/
  if(__GET_MOTOR_CTRL_MODE(M8010_J1)==POS_LOOP)
    __SET_MOTOR_ANGLE(M8010_J1,(HANDLER_PTR->joint_angle[M8010_J1]-J1_MAP_D)/J1_MAP_K);
  if(__GET_MOTOR_CTRL_MODE(DM_J2)==POS_LOOP)
    __SET_MOTOR_ANGLE(DM_J2,(HANDLER_PTR->joint_angle[DM_J2]-J2_MAP_D)/J2_MAP_K);
  if(__GET_MOTOR_CTRL_MODE(DJI_J3)==POS_LOOP)
    __SET_MOTOR_ANGLE(DJI_J3,(HANDLER_PTR->joint_angle[DJI_J3]-J3_MAP_D)/J3_MAP_K);

  /*motor output*/
  for(index=0;index<HAND_MOTOR_COUNT;index++)
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

void __hand_nonforce()
{
  int index;
  for(index=0;index<HAND_MOTOR_COUNT;index++)
  {
    __SET_JOINT_ANGLE(index,HANDLER_PTR->feedback_joint_angle[index]);// 设置关节输出值为当前关节角度
    __SET_MOTOR_NONFORCE(index);
  }
}

void __hand_idle_ctrl()
{
  __ADD_MOTOR_ANGLE(DJI_HE_L,0);
  __ADD_MOTOR_ANGLE(DJI_HE_R,0);
  __ADD_JOINT_ANGLE(HAND_J1,0);
  __ADD_JOINT_ANGLE(HAND_J2,0);
  __ADD_JOINT_ANGLE(HAND_J3,0);

}

void __hand_rc_ctrl()
{
  //_ADD_JOINT_ANGLE(HAND_J1,RC_CTRL_PTR->rc.ch[2]*0.00005f);
  //_ADD_JOINT_ANGLE(HAND_J2,RC_CTRL_PTR->rc.ch[0]*0.00005f);
  //_ADD_JOINT_ANGLE(HAND_J3,RC_CTRL_PTR->rc.ch[5]*0.00005f);
  __ADD_MOTOR_ANGLE(DJI_HE_L,RC_CTRL_PTR->rc.ch[3]*0.00005f+RC_CTRL_PTR->rc.ch[1]*0.0001f);
  __ADD_MOTOR_ANGLE(DJI_HE_R,RC_CTRL_PTR->rc.ch[3]*0.00005f-RC_CTRL_PTR->rc.ch[1]*0.0001f);
  __ADD_JOINT_ANGLE(HAND_J1,-RC_CTRL_PTR->rc.ch[2]*0.00001f);
  __ADD_JOINT_ANGLE(HAND_J2,-RC_CTRL_PTR->rc.ch[0]*0.000001f);
  __ADD_JOINT_ANGLE(HAND_J3,-RC_CTRL_PTR->rc.ch[4]*0.000025);
}

#undef HANDLER 
#undef HANDLER_PTR 
