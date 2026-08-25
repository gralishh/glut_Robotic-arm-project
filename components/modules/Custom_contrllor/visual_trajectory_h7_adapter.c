#include "visual_trajectory_h7_adapter.h"

#include "Custom_ctrl.h"
#include "arm_visual_control.h"
#include "bsp_usart.h"
#include "hand_task.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

#define URAD_PER_RAD                    1000000.0f
#define TRAJECTORY_MAX_VELOCITY_URAD_S  3000000
#define TRAJECTORY_MAX_ACCEL_URAD_S2    10000000
#define TRAJECTORY_START_TOLERANCE_URAD 50000
#define TRAJECTORY_FOLLOWING_ERROR_URAD 300000
#define TRAJECTORY_GOAL_TOLERANCE_URAD  50000

static arm_visual_control_t trajectory_control;
static const int8_t joint_direction[6] = {1, -1, 1, -1, -1, 1};

static int32_t rad_to_urad(fp32 value)
{
    return (int32_t)(value * URAD_PER_RAD);
}

static uint32_t h7_now_ms(void *user)
{
    (void)user;
    return HAL_GetTick();
}

static void h7_uart_transmit(void *user, const uint8_t *data, size_t length)
{
    (void)user;
    (void)USART1_Send((uint8_t *)data, (unsigned short)length);
}

static bool h7_read_feedback(
    void *user,
    int32_t position_urad[6],
    int32_t velocity_urad_s[6])
{
    static int32_t previous_position[6];
    static int32_t previous_velocity[6];
    static uint32_t previous_ms;
    static bool initialized;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - previous_ms;
    uint8_t i;
    (void)user;

    for (i = 0u; i < 6u; ++i) {
        position_urad[i] = rad_to_urad(hand_task_handler_ptr->feedback_joint_angle[i]);
    }
    if (initialized && (elapsed != 0u)) {
        for (i = 0u; i < 6u; ++i) {
            previous_velocity[i] = (int32_t)(
                ((int64_t)position_urad[i] - previous_position[i]) * 1000 / elapsed);
        }
    } else if (!initialized) {
        memset(previous_velocity, 0, sizeof(previous_velocity));
        initialized = true;
    }
    memcpy(velocity_urad_s, previous_velocity, sizeof(previous_velocity));
    if (elapsed != 0u) {
        memcpy(previous_position, position_urad, sizeof(previous_position));
        previous_ms = now;
    }
    return true;
}

static void h7_set_target(
    void *user,
    const int32_t position_urad[6],
    const int32_t velocity_urad_s[6])
{
    (void)user;
    (void)velocity_urad_s;
    taskENTER_CRITICAL();
    CC_handler.joint_angle[0] = (fp32)position_urad[0] / URAD_PER_RAD;
    CC_handler.joint_angle[1] = (fp32)position_urad[1] / URAD_PER_RAD;
    CC_handler.joint_angle[2] = (fp32)position_urad[2] / URAD_PER_RAD;
    CC_handler.joint_angle[3] = (fp32)position_urad[3] / URAD_PER_RAD;
    CC_handler.joint_angle[4] = (fp32)position_urad[4] / URAD_PER_RAD;
    CC_handler.G_angle = (fp32)position_urad[5] / URAD_PER_RAD;
    CC_handler.get_cc_data_flag = 1;
    taskEXIT_CRITICAL();
}

static void h7_request_stop(void *user)
{
    int32_t position[6];
    int32_t velocity[6];
    (void)user;
    if (h7_read_feedback(NULL, position, velocity)) {
        memset(velocity, 0, sizeof(velocity));
        h7_set_target(NULL, position, velocity);
    }
}

static bool h7_stop_complete(void *user)
{
    int32_t position[6];
    int32_t velocity[6];
    uint8_t i;
    (void)user;
    if (!h7_read_feedback(NULL, position, velocity)) return false;
    for (i = 0u; i < 6u; ++i) {
        if ((velocity[i] > 50000) || (velocity[i] < -50000)) return false;
    }
    return true;
}

static bool h7_estop_active(void *user)
{
    (void)user;
    return false;
}

static bool h7_driver_fault_active(void *user)
{
    uint8_t i;
    (void)user;
    for (i = 0u; i < HAND_MOTOR_COUNT; ++i) {
        if ((hand_task_handler_ptr->motor_offline_flag[i] != 0u) ||
            (hand_task_handler_ptr->motor_stall_flag[i] != 0u)) return true;
    }
    return false;
}

static bool h7_claw_action(void *user, arm_claw_action_t action)
{
    (void)user;
    (void)action;
    return false;
}

static void set_ros_limits(
    arm_joint_safety_config_t *config,
    uint8_t index,
    int32_t controller_min,
    int32_t controller_max,
    int32_t zero)
{
    config->direction = joint_direction[index];
    config->zero_offset_urad = zero;
    if (config->direction > 0) {
        config->min_position_urad = controller_min - zero;
        config->max_position_urad = controller_max - zero;
    } else {
        config->min_position_urad = zero - controller_max;
        config->max_position_urad = zero - controller_min;
    }
    config->max_velocity_urad_s = TRAJECTORY_MAX_VELOCITY_URAD_S;
    config->max_acceleration_urad_s2 = TRAJECTORY_MAX_ACCEL_URAD_S2;
    config->start_tolerance_urad = TRAJECTORY_START_TOLERANCE_URAD;
    config->following_error_urad = TRAJECTORY_FOLLOWING_ERROR_URAD;
    config->goal_tolerance_urad = TRAJECTORY_GOAL_TOLERANCE_URAD;
}

static bool h7_trajectory_init(void)
{
    arm_visual_config_t config;
    arm_visual_hooks_t hooks;
    uint8_t i;
    memset(&config, 0, sizeof(config));
    memset(&hooks, 0, sizeof(hooks));

    for (i = 0u; i < 6u; ++i) {
        int32_t zero = i == HAND_J2
            ? -rad_to_urad(hand_task_handler_ptr->min_joint_angle[i]) : 0;
        set_ros_limits(&config.joint[i], i,
            rad_to_urad(hand_task_handler_ptr->min_joint_angle[i]),
            rad_to_urad(hand_task_handler_ptr->max_joint_angle[i]), zero);
    }
    config.max_trajectory_points = ARM_VISUAL_MAX_TRAJECTORY_POINTS;
    config.communication_timeout_ms = 1000u;
    config.execution_timeout_margin_ms = 1000u;
    config.following_error_duration_ms = 300u;
    config.completion_stable_ms = 200u;
    config.stopped_velocity_urad_s = 50000u;
    config.state_period_ms = 50u;
    config.result_retry_ms = 100u;
    config.claw_open_duration_ms = 1500u;
    config.claw_close_duration_ms = 1500u;
    config.result_max_retries = 5u;

    hooks.get_monotonic_ms = h7_now_ms;
    hooks.uart_transmit = h7_uart_transmit;
    hooks.read_joint_feedback = h7_read_feedback;
    hooks.set_joint_target = h7_set_target;
    hooks.request_controlled_stop = h7_request_stop;
    hooks.controlled_stop_complete = h7_stop_complete;
    hooks.estop_active = h7_estop_active;
    hooks.driver_fault_active = h7_driver_fault_active;
    hooks.request_claw_action = h7_claw_action;
    return arm_visual_control_init(&trajectory_control, &config, &hooks, NULL);
}

void Visual_Trajectory_H7_Task(void *argument)
{
    uint8_t rx_data[128];
    (void)argument;
    osDelay(1500u);
    if (!h7_trajectory_init()) {
        for (;;) osDelay(1000u);
    }

    for (;;) {
        bool permitted =
            (hand_task_handler_ptr->ctrl_mode == HAND_MODE_CUSTOM_CTRL) &&
            !h7_driver_fault_active(NULL);
        unsigned int available = USART1_GetDataCount();
        arm_visual_control_set_ready(&trajectory_control, permitted);
        if (available > sizeof(rx_data)) available = sizeof(rx_data);
        if (available != 0u) {
            unsigned int received = USART1_Recv(rx_data, (unsigned short)available);
            arm_visual_control_feed(&trajectory_control, rx_data, received);
        }
        arm_visual_control_tick(&trajectory_control);
        arm_visual_control_service(&trajectory_control);
        osDelay(1u);
    }
}
