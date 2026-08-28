#ifndef ARM_VISUAL_CONTROL_H
#define ARM_VISUAL_CONTROL_H

#include "arm_serial_protocol.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
#ifndef ARM_VISUAL_MAX_TRAJECTORY_POINTS
#define ARM_VISUAL_MAX_TRAJECTORY_POINTS 100u
#endif

#define ARM_VISUAL_SEQUENCE_CACHE_SIZE    16u

/* Example error codes. The motor project may extend this table. */
#define ARM_ERROR_NONE                    0x0000u
#define ARM_ERROR_BAD_TRAJECTORY          0x0101u
#define ARM_ERROR_START_MISMATCH           0x0102u
#define ARM_ERROR_RUNTIME_LIMIT            0x0103u
#define ARM_ERROR_FOLLOWING                0x0104u
#define ARM_ERROR_COMM_TIMEOUT             0x0201u
#define ARM_ERROR_EXECUTION_TIMEOUT        0x0202u
#define ARM_ERROR_FEEDBACK                 0x0301u
#define ARM_ERROR_DRIVER_FAULT             0x0302u
#define ARM_ERROR_ESTOP                    0x0401u

typedef struct {
    uint32_t time_ms;
    int32_t position_urad[ARM_PROTOCOL_JOINT_COUNT];
    int32_t velocity_urad_s[ARM_PROTOCOL_JOINT_COUNT];
} arm_trajectory_point_t;

typedef struct {
    int8_t direction; /* +1 or -1: ROS positive direction -> controller direction. */
    int32_t zero_offset_urad; /* Controller feedback value at ROS zero pose. */
    int32_t min_position_urad;
    int32_t max_position_urad;
    int32_t max_velocity_urad_s;
    int32_t max_acceleration_urad_s2;
    int32_t start_tolerance_urad;
    int32_t following_error_urad;
    int32_t goal_tolerance_urad;
} arm_joint_safety_config_t;

typedef struct {
    arm_joint_safety_config_t joint[ARM_PROTOCOL_JOINT_COUNT];
    uint16_t max_trajectory_points;
    uint32_t communication_timeout_ms;
    uint32_t execution_timeout_margin_ms;
    uint32_t following_error_duration_ms;
    uint32_t completion_stable_ms;
    uint32_t stopped_velocity_urad_s;
    uint32_t state_period_ms;
    uint32_t result_retry_ms;
    uint32_t claw_state_period_ms;
    uint32_t claw_communication_timeout_ms;
    uint32_t claw_action_timeout_ms;
    uint8_t result_max_retries;
} arm_visual_config_t;

typedef struct {
    uint32_t (*get_monotonic_ms)(void *user);
    void (*uart_transmit)(void *user, const uint8_t *data, size_t length);
    bool (*read_joint_feedback)(
        void *user,
        /* Controller coordinates; this module converts them to ROS coordinates. */
        int32_t position_urad[ARM_PROTOCOL_JOINT_COUNT],
        int32_t velocity_urad_s[ARM_PROTOCOL_JOINT_COUNT]);
    void (*set_joint_target)(
        void *user,
        /* Controller coordinates after direction/zero conversion. */
        const int32_t position_urad[ARM_PROTOCOL_JOINT_COUNT],
        const int32_t velocity_urad_s[ARM_PROTOCOL_JOINT_COUNT]);
    void (*request_controlled_stop)(void *user);
    bool (*controlled_stop_complete)(void *user);
    bool (*estop_active)(void *user);
    bool (*driver_fault_active)(void *user);
    bool (*request_claw_action)(void *user, arm_claw_action_t action);
    bool (*read_claw_state)(
        void *user,
        arm_claw_state_t *state,
        uint8_t *flags);
} arm_visual_hooks_t;

typedef enum {
    ARM_TRAJECTORY_IDLE = 0,
    ARM_TRAJECTORY_RECEIVING,
    ARM_TRAJECTORY_EXECUTING,
    ARM_TRAJECTORY_STOPPING
} arm_trajectory_state_t;

typedef struct {
    bool valid;
    uint8_t type;
    uint16_t sequence;
    uint16_t payload_crc;
    arm_ack_status_t status;
} arm_sequence_record_t;

typedef struct {
    bool active;
    bool sent_once;
    uint16_t sequence;
    uint8_t encoded_frame[16];
    uint8_t encoded_length;
    uint8_t retries;
    uint32_t last_send_ms;
} arm_pending_result_t;

typedef struct {
    arm_protocol_parser_t parser;
    arm_visual_config_t config;
    arm_visual_hooks_t hooks;
    void *hooks_user;

    arm_public_state_t public_state;
    uint16_t error_code;
    bool ready_permitted;

    arm_trajectory_state_t trajectory_state;
    uint16_t expected_points;
    uint16_t received_points;
    uint16_t current_segment;
    arm_trajectory_point_t points[ARM_VISUAL_MAX_TRAJECTORY_POINTS];
    arm_trajectory_point_t execution_start_point;
    bool use_execution_start_segment;
    uint16_t trajectory_result_sequence;

    uint32_t execution_start_ms;
    uint32_t stable_since_ms;
    uint32_t following_error_since_ms;
    uint32_t last_valid_rx_ms;
    uint32_t last_trajectory_progress_ms;
    uint32_t last_state_tx_ms;
    uint32_t last_claw_state_tx_ms;

    arm_motion_result_t stopping_result;
    uint16_t stopping_error_code;

    uint16_t tx_sequence;
    arm_sequence_record_t sequence_cache[ARM_VISUAL_SEQUENCE_CACHE_SIZE];
    uint8_t sequence_cache_write;
    arm_pending_result_t pending_result;
    bool claw_busy;
    arm_claw_action_t claw_action;
    uint16_t claw_command_sequence;
    uint32_t claw_action_start_ms;
    arm_claw_state_t last_claw_state;
    uint8_t last_claw_state_flags;
    bool claw_state_sent;
} arm_visual_control_t;

bool arm_visual_control_init(
    arm_visual_control_t *control,
    const arm_visual_config_t *config,
    const arm_visual_hooks_t *hooks,
    void *hooks_user);

void arm_visual_control_set_ready(arm_visual_control_t *control, bool ready);

/* Call only from a local, deliberate reset path after ESTOP/fault is cleared. */
bool arm_visual_control_clear_fault(arm_visual_control_t *control);

void arm_visual_control_feed(
    arm_visual_control_t *control,
    const uint8_t *data,
    size_t length);

/* Call frequently from a normal-priority communication task. */
void arm_visual_control_service(arm_visual_control_t *control);

/* Call at a fixed 1-2 ms period from the real-time control task. */
void arm_visual_control_tick(arm_visual_control_t *control);

/* May be called by the local motor/safety layer on an unrecoverable fault. */
void arm_visual_control_report_fault(
    arm_visual_control_t *control,
    uint16_t error_code);

#ifdef __cplusplus
}
#endif

#endif
