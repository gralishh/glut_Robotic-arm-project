#include "arm_visual_control.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint32_t now;
    arm_claw_state_t claw_state;
    arm_claw_action_t last_action;
    unsigned action_count;
    unsigned frame_count[256];
    uint8_t last_payload[256][ARM_PROTOCOL_MAX_PAYLOAD];
    uint16_t last_sequence[256];
} fake_hardware_t;

static uint32_t fake_now(void *user)
{
    return ((fake_hardware_t *)user)->now;
}

static void fake_uart(void *user, const uint8_t *data, size_t length)
{
    fake_hardware_t *fake = (fake_hardware_t *)user;
    uint8_t type;
    int16_t payload_length;
    assert(length >= 7u);
    assert(data[0] == ARM_PROTOCOL_MAGIC_1 && data[1] == ARM_PROTOCOL_MAGIC_2);
    type = data[2];
    payload_length = arm_protocol_payload_length(type);
    assert(payload_length >= 0);
    assert(length == (size_t)payload_length + 7u);
    assert(arm_read_u16_le(&data[length - 2u]) ==
        arm_crc16_ccitt_false(&data[2], length - 4u));
    fake->frame_count[type]++;
    fake->last_sequence[type] = arm_read_u16_le(&data[3]);
    if (payload_length > 0) {
        memcpy(fake->last_payload[type], &data[5], (size_t)payload_length);
    }
}

static bool fake_feedback(void *user, int32_t position[6], int32_t velocity[6])
{
    (void)user;
    memset(position, 0, sizeof(int32_t) * 6u);
    memset(velocity, 0, sizeof(int32_t) * 6u);
    return true;
}

static void fake_target(void *user, const int32_t position[6], const int32_t velocity[6])
{
    (void)user;
    (void)position;
    (void)velocity;
}

static void fake_stop(void *user) {(void)user;}
static bool fake_stop_complete(void *user) {(void)user; return true;}
static bool fake_safety_clear(void *user) {(void)user; return false;}

static bool fake_claw_request(void *user, arm_claw_action_t action)
{
    fake_hardware_t *fake = (fake_hardware_t *)user;
    fake->last_action = action;
    fake->action_count++;
    if (action == ARM_CLAW_OPEN) fake->claw_state = ARM_CLAW_STATE_OPENING;
    if (action == ARM_CLAW_CLOSE) fake->claw_state = ARM_CLAW_STATE_CLOSING;
    if (action == ARM_CLAW_STOP) fake->claw_state = ARM_CLAW_STATE_UNKNOWN;
    return true;
}

static bool fake_claw_state(void *user, arm_claw_state_t *state, uint8_t *flags)
{
    fake_hardware_t *fake = (fake_hardware_t *)user;
    *state = fake->claw_state;
    *flags = 0u;
    return true;
}

static void feed_frame(
    arm_visual_control_t *control,
    uint8_t type,
    uint16_t sequence,
    const uint8_t *payload,
    uint8_t payload_length)
{
    uint8_t frame[ARM_PROTOCOL_MAX_FRAME_SIZE];
    size_t length = arm_protocol_encode(
        type, sequence, payload, payload_length, frame, sizeof(frame));
    assert(length != 0u);
    arm_visual_control_feed(control, frame, length);
}

static void ack_result(arm_visual_control_t *control, uint16_t sequence)
{
    uint8_t ok = ARM_ACK_OK;
    feed_frame(control, ARM_MSG_ACK, sequence, &ok, 1u);
}

static void init_control(arm_visual_control_t *control, fake_hardware_t *fake)
{
    arm_visual_config_t config;
    arm_visual_hooks_t hooks;
    unsigned joint;
    memset(&config, 0, sizeof(config));
    memset(&hooks, 0, sizeof(hooks));
    for (joint = 0u; joint < 6u; ++joint) {
        config.joint[joint].direction = 1;
        config.joint[joint].min_position_urad = -4000000;
        config.joint[joint].max_position_urad = 4000000;
        config.joint[joint].max_velocity_urad_s = 4000000;
        config.joint[joint].max_acceleration_urad_s2 = 10000000;
        config.joint[joint].start_tolerance_urad = 100000;
        config.joint[joint].following_error_urad = 100000;
        config.joint[joint].goal_tolerance_urad = 100000;
    }
    config.max_trajectory_points = 4u;
    config.communication_timeout_ms = 5000u;
    config.execution_timeout_margin_ms = 1000u;
    config.following_error_duration_ms = 100u;
    config.completion_stable_ms = 100u;
    config.stopped_velocity_urad_s = 10000u;
    config.state_period_ms = 50u;
    config.result_retry_ms = 100u;
    config.claw_state_period_ms = 100u;
    config.claw_communication_timeout_ms = 5000u;
    config.claw_action_timeout_ms = 2000u;
    config.result_max_retries = 5u;

    hooks.get_monotonic_ms = fake_now;
    hooks.uart_transmit = fake_uart;
    hooks.read_joint_feedback = fake_feedback;
    hooks.set_joint_target = fake_target;
    hooks.request_controlled_stop = fake_stop;
    hooks.controlled_stop_complete = fake_stop_complete;
    hooks.estop_active = fake_safety_clear;
    hooks.driver_fault_active = fake_safety_clear;
    hooks.request_claw_action = fake_claw_request;
    hooks.read_claw_state = fake_claw_state;
    assert(arm_visual_control_init(control, &config, &hooks, fake));
}

int main(void)
{
    arm_visual_control_t control;
    fake_hardware_t fake;
    uint8_t action;
    unsigned result_frames;
    unsigned state_frames;

    memset(&fake, 0, sizeof(fake));
    fake.claw_state = ARM_CLAW_STATE_UNKNOWN;
    assert(arm_crc16_ccitt_false((const uint8_t *)"123456789", 9u) == 0x29B1u);
    init_control(&control, &fake);

    /* Initial state is UNKNOWN and is emitted immediately, verified=0. */
    arm_visual_control_service(&control);
    assert(fake.frame_count[ARM_MSG_CLAW_STATE] == 1u);
    assert(fake.last_payload[ARM_MSG_CLAW_STATE][0] == ARM_CLAW_STATE_UNKNOWN);
    assert(fake.last_payload[ARM_MSG_CLAW_STATE][1] == 0u);

    /* Claw command is independent of six-axis ready/CUSTOM_CTRL state. */
    action = ARM_CLAW_OPEN;
    feed_frame(&control, ARM_MSG_CLAW_COMMAND, 10u, &action, 1u);
    assert(fake.last_payload[ARM_MSG_ACK][0] == ARM_ACK_OK);
    assert(fake.last_action == ARM_CLAW_OPEN);
    assert(fake.action_count == 1u);
    arm_visual_control_service(&control);
    assert(fake.last_payload[ARM_MSG_CLAW_STATE][0] == ARM_CLAW_STATE_OPENING);

    /* Replayed command is ACKed from the sequence cache and is not executed. */
    feed_frame(&control, ARM_MSG_CLAW_COMMAND, 10u, &action, 1u);
    assert(fake.action_count == 1u);

    fake.claw_state = ARM_CLAW_STATE_OPEN;
    arm_visual_control_tick(&control);
    arm_visual_control_service(&control);
    assert(fake.last_payload[ARM_MSG_CLAW_RESULT][0] == ARM_CLAW_COMPLETED_UNVERIFIED);
    assert(fake.last_sequence[ARM_MSG_CLAW_RESULT] == 10u);
    assert(fake.last_payload[ARM_MSG_CLAW_STATE][0] == ARM_CLAW_STATE_OPEN);
    result_frames = fake.frame_count[ARM_MSG_CLAW_RESULT];
    fake.now += 100u;
    arm_visual_control_service(&control);
    assert(fake.frame_count[ARM_MSG_CLAW_RESULT] == result_frames + 1u);
    assert(fake.last_sequence[ARM_MSG_CLAW_RESULT] == 10u);
    ack_result(&control, 10u);

    /* State is periodic at exactly the configured 10 Hz interval. */
    state_frames = fake.frame_count[ARM_MSG_CLAW_STATE];
    fake.now += 99u;
    arm_visual_control_service(&control);
    assert(fake.frame_count[ARM_MSG_CLAW_STATE] == state_frames);
    fake.now += 1u;
    arm_visual_control_service(&control);
    assert(fake.frame_count[ARM_MSG_CLAW_STATE] == state_frames + 1u);

    /* STOP during motion drives the fake outputs low and reports interruption. */
    action = ARM_CLAW_CLOSE;
    feed_frame(&control, ARM_MSG_CLAW_COMMAND, 11u, &action, 1u);
    assert(fake.claw_state == ARM_CLAW_STATE_CLOSING);
    action = ARM_CLAW_STOP;
    feed_frame(&control, ARM_MSG_CLAW_COMMAND, 12u, &action, 1u);
    arm_visual_control_service(&control);
    assert(fake.last_action == ARM_CLAW_STOP);
    assert(fake.claw_state == ARM_CLAW_STATE_UNKNOWN);
    assert(fake.last_payload[ARM_MSG_CLAW_RESULT][0] == ARM_CLAW_INTERRUPTED);
    ack_result(&control, 11u);

    /* A motion that never reaches a stable endpoint is stopped after 2 s. */
    action = ARM_CLAW_CLOSE;
    feed_frame(&control, ARM_MSG_CLAW_COMMAND, 13u, &action, 1u);
    fake.now += 2000u;
    arm_visual_control_tick(&control);
    arm_visual_control_service(&control);
    assert(fake.last_action == ARM_CLAW_STOP);
    assert(fake.last_payload[ARM_MSG_CLAW_RESULT][0] == ARM_CLAW_TIMEOUT);
    ack_result(&control, 13u);

    /* Loss of 10 Hz heartbeats stops an active pulse before its 700 ms end. */
    control.config.claw_communication_timeout_ms = 500u;
    action = ARM_CLAW_OPEN;
    feed_frame(&control, ARM_MSG_CLAW_COMMAND, 14u, &action, 1u);
    fake.now += 500u;
    arm_visual_control_tick(&control);
    arm_visual_control_service(&control);
    assert(fake.last_action == ARM_CLAW_STOP);
    assert(fake.claw_state == ARM_CLAW_STATE_UNKNOWN);
    assert(fake.last_payload[ARM_MSG_CLAW_RESULT][0] == ARM_CLAW_TIMEOUT);

    puts("arm_visual claw host tests passed");
    return 0;
}
