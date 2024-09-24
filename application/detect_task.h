/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       detect_task.c/h
  * @brief      detect error task, judged by receiving data time. provide detect
                hook function, error exist function.
  *             检测错误任务， 通过接收数据时间来判断.提供 检测钩子函数,错误存在函数.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add oled, gyro accel and mag sensors
  *
  @verbatim
  ==============================================================================
    add a sensor 
    1. in detect_task.h, add the sensor name at the end of errorList,like
    enum errorList
    {
        ...
        XXX_TOE,    //new sensor
        errorList_LENGHT,
    };
    2.in detect_init function, add the offlineTime, onlinetime, priority params,like
        uint16_t set_item[errorList_LENGHT][3] =
        {
            ...
            {n,n,n}, //XX_TOE
        };
    3. if XXX_TOE has data_is_error_fun ,solve_lost_fun,solve_data_error_fun function, 
        please assign to function pointer.
    4. when XXX_TOE sensor data come, add the function detect_hook(XXX_TOE) function.
    如果要添加一个新设备
    1.第一步在detect_task.h，添加设备名字在errorList的最后，像
    enum errorList
    {
        ...
        XXX_TOE,    //新设备
        errorList_LENGHT,
    };
    2.在detect_init函数,添加offlineTime, onlinetime, priority参数
        uint16_t set_item[errorList_LENGHT][3] =
        {
            ...
            {n,n,n}, //XX_TOE
        };
    3.如果有data_is_error_fun ,solve_lost_fun,solve_data_error_fun函数，赋值到函数指针
    4.在XXX_TOE设备数据来的时候, 添加函数detect_hook(XXX_TOE).
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
  
#ifndef DETECT_TASK_H
#define DETECT_TASK_H
#include "struct_typedef.h"
#include "main.h"

#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 10

//错误码以及对应设备顺序
enum errorList
{
    DBUSTOE = 0,
	RefereeSystemTOE,
	
	
	TOE_EXTER_ENCODER_pitch_2,
	TOE_EXTER_ENCODER_roll_3,
	
	TOE_EXTER_ENCODER_1,
	TOE_EXTER_ENCODER_2,
    TOE_EXTER_ENCODER_3,
	
	TOE_2006_HAND_x_4_ID,
	TOE_2006_HAND_pitch_2_ID,
	
	TOE_2006_roll_1_ID,
	
	TOE_3508_M1_ID,
    TOE_3508_M2_ID,
    TOE_3508_M3_ID,
    TOE_3508_M4_ID,

    TOE_3508_RISE1_height_L_5_ID,
    TOE_3508_RISE2_height_R_6_ID,
	TOE_3508_LeftRight_7_ID,
    TOE_encoder_HAND_x_4_ID,


	TOE_2006_VIEW_pitch_6_ID,
	TOE_encoder_x_4,

	

    errorListLength,
};


typedef __packed struct
{
    uint32_t newTime;
    uint32_t lastTime;
    uint32_t Losttime;
    uint32_t worktime;
    uint16_t setOfflineTime : 12;  //表示setOfflineTime这个变量占12位
    uint16_t setOnlineTime : 12;
    uint8_t enable : 1;
    uint8_t Priority : 4;
    uint8_t errorExist : 1;
    uint8_t isLost : 1;
    uint8_t dataIsError : 1;

    fp32 frequency;
    bool_t (*dataIsErrorFun)(void);
    void (*solveLostFun)(void);
    void (*solveDataErrorFun)(void);
} error_t;


extern const error_t *get_errorList_point(void);
static void DetectInit(uint32_t time);
void DetectTask(void const *pvParameters);
bool_t toe_is_error(uint8_t err);
void DetectHook(uint8_t toe);
const error_t *get_errorList_point(void);
static void DetectInit(uint32_t time);

#endif
