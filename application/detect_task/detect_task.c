#include "detect_task.h"
#include "main.h"

#include "remote_control.h"
#include "cmsis_os.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include <string.h>

/**
 * @brief          init errorList, assign  offline_time, online_time, priority.
 * @param[in]      time: system time
 * @retval         none
 */

// //红灯闪，灭函数，切换闪灭
// #define DETECT_LED_R_TOGGLE() led_red_toggle()
// #define DETECT_LED_R_ON() led_red_on()
// #define DETECT_LED_R_OFF() led_red_off()
// 流水灯闪灭函数
#define DETECT_FLOW_LED_ON(i) flow_led_on(i)
#define DETECT_FLOW_LED_OFF(i) flow_led_off(i)

// #define DETECT_TASK_INIT_TIME 57
// #define DETECT_CONTROL_TIME 10

extern void aRGB_led_show(uint32_t aRGB);

// static void DetectInit(uint32_t time);
// void DetectTask(void const *pvParameters);
// bool_t toe_is_error(uint8_t err);
// void DetectHook(uint8_t toe);
// const error_t *get_errorList_point(void);

// static void DetectDisplay(uint8_t num);
error_t errorList[errorListLength + 1];
// extern void OLED_com_reset(void);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t detect_task_stack;
#endif

uint32_t aRGB = 0xFF0000FF;

/************堵转检测************/
//  仿照上方大疆掉线检测代码
//  初始化后使能
//  在电机刷新数据后才开始检测

static stall_error_t stall_list[STALL_MOTOR_COUNT];

#define CURRENT_HISTORY_SIZE 10
#define judge_sign(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))
// 堵转判断参数
#define CURRENT_SURGE_FACTOR 2.1f   // 电流突增倍数（当前电流 > avg * factor）
// #define SPEED_SLOPE_THRESHOLD -2.0f // 速度斜率阈值 (rpm/s) 负值表示急剧减速
#define STALL_CONFIRM_COUNT 60      // 连续满足条件次数才确认堵转(持续300ms才判断为堵转)

/**
 * @brief          检测任务
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void DetectTask(void const *pvParameters)
{
     StallDetectInit();//堵转初始化

    static uint32_t systemTime;
    systemTime = xTaskGetTickCount();
    // init,初始化
    DetectInit(systemTime);
    // wait a time.空闲一段时间
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
            // disable, continue
            // 未使能，跳过
            if (errorList[i].enable == 0)
            {
                continue;
            }

            // judge offline.判断掉线
            if (systemTime - errorList[i].newTime > errorList[i].setOfflineTime)
            {
                if (errorList[i].errorExist == 0)
                {
                    // record error and time
                    // 记录错误以及掉线时间
                    errorList[i].isLost = 1;
                    errorList[i].errorExist = 1;
                    errorList[i].Losttime = systemTime;
                }
                // judge the priority,save the highest priority ,
                // 判断错误优先级， 保存优先级最高的错误码
                if (errorList[i].Priority > errorList[error_num_display].Priority)
                {
                    error_num_display = i;
                }

                errorList[errorListLength].isLost = 1;
                errorList[errorListLength].errorExist = 1;
                // if solve_lost_fun != NULL, run it
                // 如果提供解决函数，运行解决函数
                if (errorList[i].solveLostFun != NULL)
                {
                    errorList[i].solveLostFun();
                }
            }
            else if (systemTime - errorList[i].worktime < errorList[i].setOnlineTime) // 5//3
            {
                // 刚刚上线，可能存在数据不稳定，只记录不丢失，
                errorList[i].isLost = 0;
                errorList[i].errorExist = 1;
            }
            else
            {
                errorList[i].isLost = 0; // 设备没有丢失
                // 判断是否存在数据错误
                if (errorList[i].dataIsError)
                {
                    errorList[i].errorExist = 1; // can返回的数据有错
                }
                else
                {
                    errorList[i].errorExist = 0;
                }
                // 计算频率
                if (errorList[i].newTime > errorList[i].lastTime)
                {
                    errorList[i].frequency = configTICK_RATE_HZ / (fp32)(errorList[i].newTime - errorList[i].lastTime);
                }
            }
        }

         StallDetectTask();//
        // DetectDisplay(error_num_display + 1);
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
    errorList[toe].newTime = xTaskGetTickCount(); // 这个时间就是can接收设备数据的那一刻的时间
    // 更新丢失情况
    if (errorList[toe].isLost) // 初始化时一开始errorList[toe].isLost=1
    {
        errorList[toe].isLost = 0;
        errorList[toe].worktime = errorList[toe].newTime; // 这个就是设备刚上电的时间
    }
    // 判断数据是否错误
    if (errorList[toe].dataIsErrorFun != NULL) // 初始化时一开始errorList[i].dataIsErrorFun = NULL
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
/**
 * @brief          初始化errorList,赋值 offline_time, online_time, priority
 * @param[in]      time:系统时间
 * @retval         none
 */
static void DetectInit(uint32_t time)
{
    // 设置离线时间，上线稳定工作时间，优先级 offlineTime onlinetime priority
    uint16_t setItem[errorListLength][3] =
        {
            // errorListLength为枚举类型，它在枚举中的值为9，说明有8组3为数组
            {30, 40, 15}, // SBUS
            {90, 80, 14}, // CAMERA
            {20, 20, 13}, //  TOE_3508_M1_ID,
            {20, 20, 12}, // TOE_3508_M2_ID,
            {20, 20, 11}, // TOE_3508_M3_ID,
            {20, 20, 10}, // TOE_3508_M4_ID,
            {30, 20, 9},  // TOE_J1,
            {50, 40, 8},  // TOE_J2,
            {50, 20, 7},  // TOE_J3,
            {30, 20, 6},  //  //TOE_J4,
            {50, 40, 5},  // TOE_J5,
            {50, 40, 4},  // TOE_G,
            {20, 20, 3},  // TOE_UPLIFT,
            {20, 20, 2},  // TOE_UPLIFT_ECD,

        };

    for (uint8_t i = 0; i < errorListLength; i++)
    {
        // errorList为用结构体error_t定义的结构体数组   //                Y  P
        errorList[i].setOfflineTime = setItem[i][0]; // 掉线判断时间30，2，2，10，10，10，10，10，100
        errorList[i].setOnlineTime = setItem[i][1];  // 刚上线判断时间40，3，3，10，10，10，10，10，40
        errorList[i].Priority = setItem[i][2];       //      优先级15，14，13，12，11，10，9，8，7，
        errorList[i].dataIsErrorFun = NULL;
        errorList[i].solveLostFun = NULL;
        errorList[i].solveDataErrorFun = NULL;

        errorList[i].enable = 1;
        errorList[i].errorExist = 1;
        errorList[i].isLost = 1;
        errorList[i].dataIsError = 1;
        errorList[i].frequency = 0.0f;
        errorList[i].newTime = time; // 把系统的时钟节拍赋给下面三个
        errorList[i].lastTime = time;
        errorList[i].Losttime = time;
        errorList[i].worktime = time;
    }

    // 可加自定义控制器数据流
    //  errorList[OLED_TOE].data_is_error_fun = NULL;
    //  errorList[OLED_TOE].solve_lost_fun = OLED_com_reset;
    //  errorList[OLED_TOE].solve_data_error_fun = NULL;
    errorList[DBUSTOE].dataIsErrorFun = RC_data_is_error; // RC_data_is_error为1时就是遥控数据出错
    errorList[DBUSTOE].solveLostFun = slove_RC_lost;
    errorList[DBUSTOE].solveDataErrorFun = slove_data_error;
}

/************堵转检测************/

// 堵转时：电流符号 × 速度斜率（加速度） < 0（异号）
void StallDetectInit(void)
{
    memset(stall_list, 0, sizeof(stall_list));
    // 设置历史均值下限，斜率阈值（加速度），优先级(优先级暂未使用)
    // 由于电机常态转速不同所以加速度判定大小可能不同和历史均值由于反馈的电流基数不同，推荐自己设置
    //历史均值设置正常运动反馈电流的0.5倍数即可
    //速度阈值设置为0即关闭速度判断，放在低速堵转时取消这个判断
    uint16_t setItem[STALL_MOTOR_COUNT][3] =
        {
            // {2, 40, 15}, // SBUS
            // {90, 80, 14}, // CAMERA
            // {20, 20, 13}, //  TOE_3508_M1_ID,
            // {20, 20, 12}, // TOE_3508_M2_ID,
            // {20, 20, 11}, // TOE_3508_M3_ID,
            // {20, 20, 10}, // TOE_3508_M4_ID,
            {300, 2.5, 9}, // TOE_J1,
            {500, 1.2, 8}, // TOE_J2,
            {500, 1.2, 7}, // TOE_J3,
            {200, 0.8, 6}, //  //TOE_J4,
            {300, 1.2, 5}, // TOE_J5,
            {250, 0.4, 4}, // TOE_G,
                         // {20, 20, 3},  // TOE_UPLIFT,
                         // {20, 20, 2},  // TOE_UPLIFT_ECD,

        };

    for (int i = 0; i < STALL_MOTOR_COUNT; i++)
    {
        // min_avg_current 一般设为空载电流的 1.2 倍即可
        stall_list[i].min_avg_current = setItem[i][0];
        stall_list[i].speed_slope_threshold = setItem[i][1];
        stall_list[i].Priority = setItem[i][2];

        stall_list[i].enable = 0;
        stall_list[i].recoverable = 0;
        stall_list[i].stall_flag = 0;
        stall_list[i].avg_current = 0.0f;
        stall_list[i].current_idx = 0;
        stall_list[i].stall_counter = 0;
        stall_list[i].solveStallFun = NULL; // 可选处理函数
        for (int j = 0; j < CURRENT_HISTORY_SIZE; j++)
        {
            stall_list[i].current_history[j] = 0.0f;
        }
    }
}

void StallDetectEnable(uint8_t motor_idx, uint8_t enable)
{
    if (motor_idx >= STALL_MOTOR_COUNT)
        return;
    stall_list[motor_idx].enable = enable;
    if (!enable)
    {
        stall_list[motor_idx].stall_flag = 0;
    }
}

// 在任务feedback函数内调用
void StallUpdateHook(uint8_t motor_idx, fp32 angle, fp32 speed, fp32 current)
{
    if (motor_idx >= STALL_MOTOR_COUNT)
        return;

    stall_list[motor_idx].lastTime = stall_list[motor_idx].newTime;
    stall_list[motor_idx].newTime = xTaskGetTickCount();

    stall_list[motor_idx].feedback_angle = angle;
    stall_list[motor_idx].last_speed = stall_list[motor_idx].feedback_speed;
    stall_list[motor_idx].feedback_speed = speed;
    stall_list[motor_idx].feedback_current = current;

    //时间单位为(tick）
    if (stall_list[motor_idx].newTime > stall_list[motor_idx].lastTime)
    {
        fp32 delta_time = (fp32)(stall_list[motor_idx].newTime - stall_list[motor_idx].lastTime);
        if (delta_time < 1.0f)
            delta_time = 1.0f;
        stall_list[motor_idx].speed_slope = (stall_list[motor_idx].feedback_speed - stall_list[motor_idx].last_speed) / delta_time;
    }
    else
    {
        stall_list[motor_idx].speed_slope = 0.0f;
    }

    // 环形更新当前历史电流值
    stall_list[motor_idx].current_history[stall_list[motor_idx].current_idx] = current;
    stall_list[motor_idx].current_idx = (stall_list[motor_idx].current_idx + 1) % CURRENT_HISTORY_SIZE;

    fp32 sum = 0.0f;
    for (int i = 0; i < CURRENT_HISTORY_SIZE; i++)
    {
        sum += stall_list[motor_idx].current_history[i];
    }
    stall_list[motor_idx].avg_current = sum / CURRENT_HISTORY_SIZE;

    // // 如果之前堵转，且被置恢复标志位//判断条件太宽松了
    // if (stall_list[motor_idx].stall_flag && stall_list[motor_idx].recoverable)
    // {
    //     // 恢复条件：电流回到平均附近，且速度斜率不异常，可清除
    //     if (fabsf(stall_list[motor_idx].feedback_current) < stall_list[motor_idx].avg_current * 1.5f &&
    //         stall_list[motor_idx].speed_slope > stall_list[motor_idx].speed_slope_threshold)
    //     {
    //         stall_list[motor_idx].stall_flag = 0;
    //         stall_list[motor_idx].worktime = stall_list[motor_idx].newTime;
    //     }
    // }
}

void StallDetectTask(void)
{

    for (int i = 0; i < STALL_MOTOR_COUNT; i++)
    {
        if (!stall_list[i].enable)
            continue;

        if (stall_list[i].stall_flag)
            continue;

        // 堵转条件：
        // 1. 当前电流超过平均电流的倍数（突增）（绝对值）
        // 2. 速度斜率（加速度）绝对值大于阈值,堵转时和速度的负方向的加速度急剧增大//防止加大负载时误判堵转
        // 3.电流方向和加速度方向的乘积小于0
        // 4. 均电流有效且数据历史大于最小阈值，避免在开始时环形电流历史值未满得到的均值是当前值的1/倍数
        // 速度斜率需要根据实际调整，电机的反馈的单位不同计算出的结果不同

        if (fabsf(stall_list[i].avg_current) > stall_list[i].min_avg_current &&
            fabsf(stall_list[i].feedback_current) > fabsf(stall_list[i].avg_current) * CURRENT_SURGE_FACTOR &&
            judge_sign(stall_list[i].feedback_current) * judge_sign(stall_list[i].speed_slope) < 0)
        {
            if (stall_list[i].speed_slope_threshold!=0)
            {
                if (fabsf(stall_list[i].speed_slope) > stall_list[i].speed_slope_threshold)
                {
                    stall_list[i].stall_counter++;
                    if (stall_list[i].stall_counter >= STALL_CONFIRM_COUNT)
                    {
                        stall_list[i].stall_flag = 1; // 确认堵转
                        if (stall_list[i].solveStallFun != NULL)
                        {
                            stall_list[i].solveStallFun(); // 仿照大疆代码(这里预留)
                        }
                    }
                }
                else if (stall_list[i].speed_slope_threshold == 0)
                {
                    stall_list[i].stall_counter++;
                    if (stall_list[i].stall_counter >= STALL_CONFIRM_COUNT)
                    {
                        stall_list[i].stall_flag = 1; // 确认堵转
                        if (stall_list[i].solveStallFun != NULL)
                        {
                            stall_list[i].solveStallFun(); // 仿照大疆代码(这里预留)
                        }
                    }
                }
            }
        }
        else
        {
            // 不满足条件，递减（防抖）
            if (stall_list[i].stall_counter > 0)
                stall_list[i].stall_counter--;
        
        }
    }
}

// 查询堵转状态
bool_t toe_is_stall(uint8_t motor_idx)
{
    return stall_list[motor_idx].stall_flag == 1;
}

// 清除标志
void StallClearFlag(uint8_t motor_idx)
{
    stall_list[motor_idx].stall_flag = 0;
}