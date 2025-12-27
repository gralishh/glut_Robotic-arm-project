#include "detect_task.h"
#include "main.h"

#include "remote_control.h"
#include "cmsis_os.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


/**
  * @brief          init errorList, assign  offline_time, online_time, priority.
  * @param[in]      time: system time
  * @retval         none
  */
/**
  * @brief          初始化errorList,赋值 offline_time, online_time, priority
  * @param[in]      time:系统时间
  * @retval         none
  */
// //红灯闪，灭函数，切换闪灭
// #define DETECT_LED_R_TOGGLE() led_red_toggle()
// #define DETECT_LED_R_ON() led_red_on()
// #define DETECT_LED_R_OFF() led_red_off()
//流水灯闪灭函数
#define DETECT_FLOW_LED_ON(i) flow_led_on(i)
#define DETECT_FLOW_LED_OFF(i) flow_led_off(i)

#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 10

extern void aRGB_led_show(uint32_t aRGB);


static void DetectInit(uint32_t time);
void DetectTask(void const *pvParameters);
bool_t toe_is_error(uint8_t err);
void DetectHook(uint8_t toe);
const error_t *get_errorList_point(void);


// static void DetectDisplay(uint8_t num);
error_t errorList[errorListLength + 1];
extern void OLED_com_reset(void);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t detect_task_stack;
#endif


uint32_t aRGB = 0xFF0000FF;
/**
  * @brief          检测任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void DetectTask(void const *pvParameters)
{
    static uint32_t systemTime;
    systemTime = xTaskGetTickCount();
    //init,初始化
    DetectInit(systemTime);
    //wait a time.空闲一段时间
    vTaskDelay(DETECT_TASK_INIT_TIME);

    while (1)
    {
        aRGB_led_show(aRGB);

        static uint8_t error_num_display = 0;
        systemTime = xTaskGetTickCount();

        error_num_display = errorListLength;
        errorList[errorListLength].isLost = 0;
        errorList[errorListLength].errorExist = 0;

        for (int i = 0; i < errorListLength; i++)
        {
            //disable, continue
            //未使能，跳过
            if (errorList[i].enable == 0)
            {
                continue;
            }

            //judge offline.判断掉线
            if (systemTime - errorList[i].newTime > errorList[i].setOfflineTime)
            {
                if (errorList[i].errorExist == 0)
                {
                    //record error and time
                    //记录错误以及掉线时间
                    errorList[i].isLost = 1;
                    errorList[i].errorExist = 1;
                    errorList[i].Losttime = systemTime;
                }
                //judge the priority,save the highest priority ,
                //判断错误优先级， 保存优先级最高的错误码
                if (errorList[i].Priority > errorList[error_num_display].Priority)
                {
                    error_num_display = i;
                }
                

                errorList[errorListLength].isLost = 1;
                errorList[errorListLength].errorExist = 1;
                //if solve_lost_fun != NULL, run it
                //如果提供解决函数，运行解决函数
                if (errorList[i].solveLostFun != NULL)
                {
                  errorList[i].solveLostFun();
                }
            }
             else if (systemTime - errorList[i].worktime < errorList[i].setOnlineTime)//5//3
            {
                //刚刚上线，可能存在数据不稳定，只记录不丢失，
                errorList[i].isLost = 0;
                errorList[i].errorExist = 1;
            }
            else
            {
                errorList[i].isLost = 0;  //设备没有丢失
                //判断是否存在数据错误
                if (errorList[i].dataIsError)
                {
                    errorList[i].errorExist = 1;  //can返回的数据有错
                }
                else
                {
                    errorList[i].errorExist = 0;
                }
                //计算频率
                if (errorList[i].newTime > errorList[i].lastTime)
                {
                    errorList[i].frequency = configTICK_RATE_HZ / (fp32)(errorList[i].newTime - errorList[i].lastTime);
                }
            }
        }

//        DetectDisplay(error_num_display + 1);
        vTaskDelay(DETECT_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        detect_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          获取设备对应的错误状态
  * @param[in]      err:设备目录
  * @retval         true(错误) 或者false(没错误)
  */
bool_t toe_is_error(uint8_t err)
{
    return (errorList[err].errorExist == 1);
}

void DetectHook(uint8_t toe)
{
    errorList[toe].lastTime = errorList[toe].newTime;
    errorList[toe].newTime = xTaskGetTickCount();  //这个时间就是can接收设备数据的那一刻的时间
    //更新丢失情况
   if (errorList[toe].isLost)  //初始化时一开始errorList[toe].isLost=1
    {
        errorList[toe].isLost = 0;
        errorList[toe].worktime = errorList[toe].newTime;  //这个就是设备刚上电的时间
    }
    //判断数据是否错误
    if (errorList[toe].dataIsErrorFun != NULL)  //初始化时一开始errorList[i].dataIsErrorFun = NULL
    {
        if (errorList[toe].dataIsErrorFun())
        {
            errorList[toe].errorExist = 1;
            errorList[toe].dataIsError = 1;

            if (errorList[toe].solveDataErrorFun != NULL)
            {
                errorList[toe].solveDataErrorFun();
            }
        }
        else
        {
            errorList[toe].dataIsError = 0;
        }
    }
    else
    {
        errorList[toe].dataIsError = 0;
    }
}


/**
  * @brief          得到错误列表
  * @param[in]      none
  * @retval         errorList的指针
  */
const error_t *get_errorList_point(void)
{
    return errorList;
}


// static void DetectDisplay(uint8_t num)
// {
//     static uint8_t last_num = errorListLength + 1;
//     uint8_t i = 0;

//     //8个流水显示 除底盘电机、升降电机的其他电机错误码的情况
//     for (i = DBUSTOE; i <= TOE_2006_HAND_yaw_5_ID; i++)
//     {
//         if (errorList[i].errorExist)
//         {
//             DETECT_FLOW_LED_OFF(i);
//         }
//         else
//         {
//             DETECT_FLOW_LED_ON(i);
//         }
//     }
//
//     //错误码 通过红灯闪烁次数来判断
//     if (num == errorListLength + 1)
//     {
//         DETECT_LED_R_OFF();
//         last_num = errorListLength + 1;
//     }
//     else
//     {
//         static uint8_t i = 0, led_flag = 0, cnt_num = 0, time = 0;
//         //记录最新的最高优先级的错误码，等下一轮闪烁
//         if (last_num != num)
//         {
//             last_num = num;
//         }

//         if (cnt_num == 0)
//         {
//             //cnt_num 记录还有几次闪烁，到0后，灭一段时间才开始下一轮
//             time++;
//             if (time > 50)
//             {
//                 time = 0;
//                 cnt_num = last_num;
//             }
//             return;
//         }

//         if (i == 0)
//         {

//             DETECT_LED_R_TOGGLE();
//             if (led_flag)
//             {
//                 //红灯闪灭各一次，将要剩余次数减一
//                 led_flag = 0;
//                 cnt_num--;
//             }
//             else
//             {
//                 led_flag = 1;
//             }
//         }

//         //i为计时次数，20为半个周期，切换一次红灯闪灭
//         i++;

//         if (i > 20)
//         {
//             i = 0;
//         }
//     }
// }
static void DetectInit(uint32_t time)
{
    //设置离线时间，上线稳定工作时间，优先级 offlineTime onlinetime priority
    uint16_t setItem[errorListLength][3] =
        {
            // errorListLength为枚举类型，它在枚举中的值为9，说明有8组3为数组
            {30, 40, 15}, // SBUS
            {90, 80, 14}, // CAMERA
            {20, 20, 13}, //  TOE_3508_M1_ID,

            {20, 20, 12}, // TOE_3508_M2_ID,
            {20, 20, 11}, // TOE_3508_M3_ID,
            {20, 20, 10}, // TOE_3508_M4_ID,
            {30, 40, 9},  // TOE_J1,
            {30, 40, 8},  // TOE_J2,
            {30, 40, 7},  // TOE_J3,
            //{20, 20, 6},//  //TOE_HE_L,
            //{20, 20, 5},// TOE_HE_R
            {20, 20, 4}, // TOE_UPLIFT,
            {20, 20, 3}, // TOE_UPLIFT_ECD,

        };

    for (uint8_t i = 0; i < errorListLength; i++)
    {
			//errorList为用结构体error_t定义的结构体数组   //                Y  P
        errorList[i].setOfflineTime = setItem[i][0]; //掉线判断时间30，2，2，10，10，10，10，10，100 
        errorList[i].setOnlineTime = setItem[i][1];  //刚上线判断时间40，3，3，10，10，10，10，10，40
        errorList[i].Priority = setItem[i][2]; //      优先级15，14，13，12，11，10，9，8，7，
        errorList[i].dataIsErrorFun = NULL;
        errorList[i].solveLostFun = NULL;
        errorList[i].solveDataErrorFun = NULL;

        errorList[i].enable = 1;
        errorList[i].errorExist = 1;
        errorList[i].isLost = 1;
        errorList[i].dataIsError = 1;
        errorList[i].frequency = 0.0f;
        errorList[i].newTime = time;  //把系统的时钟节拍赋给下面三个
        errorList[i].lastTime = time;
        errorList[i].Losttime = time;
        errorList[i].worktime = time;
    }

    // errorList[OLED_TOE].data_is_error_fun = NULL;
    // errorList[OLED_TOE].solve_lost_fun = OLED_com_reset;
    // errorList[OLED_TOE].solve_data_error_fun = NULL;
    errorList[DBUSTOE].dataIsErrorFun = RC_data_is_error;  //RC_data_is_error为1时就是遥控数据出错
    errorList[DBUSTOE].solveLostFun = slove_RC_lost;
    errorList[DBUSTOE].solveDataErrorFun = slove_data_error;

}
