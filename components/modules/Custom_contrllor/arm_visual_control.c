#include "arm_visual_control.h"

#include <string.h>

static uint32_t now_ms(const arm_visual_control_t *control)
{
    return control->hooks.get_monotonic_ms(control->hooks_user);
}
static int64_t abs_i64(int64_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t round_to_i32(double value)
{
    if (value > 2147483647.0) {
        return INT32_MAX;
    }
    if (value < -2147483648.0) {
        return INT32_MIN;
    }
    return (int32_t)((value >= 0.0) ? (value + 0.5) : (value - 0.5));
}

static bool elapsed_at_least(uint32_t now, uint32_t then, uint32_t interval)
{
    return (uint32_t)(now - then) >= interval;
}

static int32_t clamp_i64_to_i32(int64_t value)
{
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)value;
}

static bool read_feedback_ros(
    arm_visual_control_t *control,
    int32_t position[ARM_PROTOCOL_JOINT_COUNT],
    int32_t velocity[ARM_PROTOCOL_JOINT_COUNT])
{
    uint8_t joint;
    int32_t controller_position[ARM_PROTOCOL_JOINT_COUNT];
    int32_t controller_velocity[ARM_PROTOCOL_JOINT_COUNT];

    if (!control->hooks.read_joint_feedback(
            control->hooks_user, controller_position, controller_velocity)) {
        return false;
    }
    for (joint = 0u; joint < ARM_PROTOCOL_JOINT_COUNT; ++joint) {
        const arm_joint_safety_config_t *cfg = &control->config.joint[joint];
        position[joint] = clamp_i64_to_i32(
            (int64_t)cfg->direction *
            ((int64_t)controller_position[joint] - cfg->zero_offset_urad));
        velocity[joint] = clamp_i64_to_i32(
            (int64_t)cfg->direction * controller_velocity[joint]);
    }
    return true;
}

static void set_target_from_ros(
    arm_visual_control_t *control,
    const int32_t position[ARM_PROTOCOL_JOINT_COUNT],
    const int32_t velocity[ARM_PROTOCOL_JOINT_COUNT])
{
    uint8_t joint;
    int32_t controller_position[ARM_PROTOCOL_JOINT_COUNT];
    int32_t controller_velocity[ARM_PROTOCOL_JOINT_COUNT];

    for (joint = 0u; joint < ARM_PROTOCOL_JOINT_COUNT; ++joint) {
        const arm_joint_safety_config_t *cfg = &control->config.joint[joint];
        controller_position[joint] = clamp_i64_to_i32(
            (int64_t)cfg->direction * position[joint] + cfg->zero_offset_urad);
        controller_velocity[joint] = clamp_i64_to_i32(
            (int64_t)cfg->direction * velocity[joint]);
    }
    control->hooks.set_joint_target(
        control->hooks_user, controller_position, controller_velocity);
}

static void transmit(
    arm_visual_control_t *control,
    const uint8_t *data,
    size_t length)
{
    control->hooks.uart_transmit(control->hooks_user, data, length);
}

static void send_ack(
    arm_visual_control_t *control,
    uint16_t command_sequence,
    arm_ack_status_t status)
{
    uint8_t payload[ARM_PAYLOAD_ACK];
    uint8_t frame[8];
    size_t length;

    payload[0] = (uint8_t)status;
    length = arm_protocol_encode(
        ARM_MSG_ACK,
        command_sequence,
        payload,
        sizeof(payload),
        frame,
        sizeof(frame));
    if (length != 0u) {
        transmit(control, frame, length);
    }
}

static arm_sequence_record_t *find_sequence(
    arm_visual_control_t *control,
    uint8_t type,
    uint16_t sequence)
{
    uint8_t i;

    for (i = 0u; i < ARM_VISUAL_SEQUENCE_CACHE_SIZE; ++i) {
        arm_sequence_record_t *record = &control->sequence_cache[i];
        if (record->valid &&
            (record->type == type) &&
            (record->sequence == sequence)) {
            return record;
        }
    }
    return NULL;
}

static void save_sequence(
    arm_visual_control_t *control,
    const arm_protocol_frame_t *frame,
    arm_ack_status_t status)
{
    arm_sequence_record_t *record =
        &control->sequence_cache[control->sequence_cache_write];

    record->valid = true;
    record->type = frame->type;
    record->sequence = frame->sequence;
    record->payload_crc = arm_crc16_ccitt_false(
        frame->payload,
        frame->payload_length);
    record->status = status;

    control->sequence_cache_write = (uint8_t)(
        (control->sequence_cache_write + 1u) %
        ARM_VISUAL_SEQUENCE_CACHE_SIZE);
}

static bool point_within_waypoint_limits(
    const arm_visual_control_t *control,
    const arm_trajectory_point_t *point)
{
    uint8_t joint;

    for (joint = 0u; joint < ARM_PROTOCOL_JOINT_COUNT; ++joint) {
        const arm_joint_safety_config_t *limit = &control->config.joint[joint];
        if ((point->position_urad[joint] < limit->min_position_urad) ||
            (point->position_urad[joint] > limit->max_position_urad) ||
            (abs_i64(point->velocity_urad_s[joint]) > limit->max_velocity_urad_s)) {
            return false;
        }
    }
    return true;
}

static bool decode_point(
    const arm_protocol_frame_t *frame,
    uint16_t *point_index,
    arm_trajectory_point_t *point)
{
    uint8_t joint;
    uint8_t offset = 0u;

    if ((frame == NULL) ||
        (point_index == NULL) ||
        (point == NULL) ||
        (frame->payload_length != ARM_PAYLOAD_TRAJECTORY_POINT)) {
        return false;
    }

    *point_index = arm_read_u16_le(&frame->payload[offset]);
    offset += 2u;
    point->time_ms = arm_read_u32_le(&frame->payload[offset]);
    offset += 4u;

    for (joint = 0u; joint < ARM_PROTOCOL_JOINT_COUNT; ++joint) {
        point->position_urad[joint] = arm_read_i32_le(&frame->payload[offset]);
        offset += 4u;
    }
    for (joint = 0u; joint < ARM_PROTOCOL_JOINT_COUNT; ++joint) {
        point->velocity_urad_s[joint] = arm_read_i32_le(&frame->payload[offset]);
        offset += 4u;
    }
    return offset == ARM_PAYLOAD_TRAJECTORY_POINT;
}

static void interpolate_segment(
    const arm_trajectory_point_t *p0,
    const arm_trajectory_point_t *p1,
    uint32_t trajectory_time_ms,
    double position[ARM_PROTOCOL_JOINT_COUNT],
    double velocity[ARM_PROTOCOL_JOINT_COUNT],
    double acceleration[ARM_PROTOCOL_JOINT_COUNT])
{
    uint8_t joint;
    double segment_ms = (double)(p1->time_ms - p0->time_ms);
    double duration_s = segment_ms / 1000.0;
    double s = ((double)trajectory_time_ms - (double)p0->time_ms) / segment_ms;
    double s2;
    double s3;
    double h00;
    double h10;
    double h01;
    double h11;
    double dh00;
    double dh10;
    double dh01;
    double dh11;
    double d2h00;
    double d2h10;
    double d2h01;
    double d2h11;

    if (s < 0.0) {
        s = 0.0;
    } else if (s > 1.0) {
        s = 1.0;
    }

    s2 = s * s;
    s3 = s2 * s;
    h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    h10 = s3 - 2.0 * s2 + s;
    h01 = -2.0 * s3 + 3.0 * s2;
    h11 = s3 - s2;

    dh00 = 6.0 * s2 - 6.0 * s;
    dh10 = 3.0 * s2 - 4.0 * s + 1.0;
    dh01 = -6.0 * s2 + 6.0 * s;
    dh11 = 3.0 * s2 - 2.0 * s;

    d2h00 = 12.0 * s - 6.0;
    d2h10 = 6.0 * s - 4.0;
    d2h01 = -12.0 * s + 6.0;
    d2h11 = 6.0 * s - 2.0;

    for (joint = 0u; joint < ARM_PROTOCOL_JOINT_COUNT; ++joint) {
        double q0 = p0->position_urad[joint];
        double q1 = p1->position_urad[joint];
        double v0 = p0->velocity_urad_s[joint];
        double v1 = p1->velocity_urad_s[joint];

        position[joint] =
            h00 * q0 + h10 * duration_s * v0 +
            h01 * q1 + h11 * duration_s * v1;
        velocity[joint] =
            dh00 * q0 / duration_s + dh10 * v0 +
            dh01 * q1 / duration_s + dh11 * v1;
        acceleration[joint] =
            d2h00 * q0 / (duration_s * duration_s) +
            d2h10 * v0 / duration_s +
            d2h01 * q1 / (duration_s * duration_s) +
            d2h11 * v1 / duration_s;
    }
}

static bool sampled_trajectory_is_safe(const arm_visual_control_t *control)
{
    uint16_t segment;
    uint8_t sample;
    uint8_t joint;
    double position[ARM_PROTOCOL_JOINT_COUNT];
    double velocity[ARM_PROTOCOL_JOINT_COUNT];
    double acceleration[ARM_PROTOCOL_JOINT_COUNT];

    if (control->received_points < 2u) {
        return control->received_points == 1u;
    }

    for (segment = 0u; segment + 1u < control->received_points; ++segment) {
        const arm_trajectory_point_t *p0 = &control->points[segment];
        const arm_trajectory_point_t *p1 = &control->points[segment + 1u];
        uint32_t duration = p1->time_ms - p0->time_ms;

        if (duration == 0u) {
            return false;
        }

        /* Catch spline overshoot before any motor is enabled. */
        for (sample = 0u; sample <= 16u; ++sample) {
            uint32_t t = p0->time_ms + (uint32_t)(
                ((uint64_t)duration * sample) / 16u);
            interpolate_segment(p0, p1, t, position, velocity, acceleration);

            for (joint = 0u; joint < ARM_PROTOCOL_JOINT_COUNT; ++joint) {
                const arm_joint_safety_config_t *limit =
                    &control->config.joint[joint];
                if ((position[joint] < limit->min_position_urad) ||
                    (position[joint] > limit->max_position_urad) ||
                    ((velocity[joint] < 0.0 ? -velocity[joint] : velocity[joint]) >
                     limit->max_velocity_urad_s) ||
                    ((acceleration[joint] < 0.0 ? -acceleration[joint] : acceleration[joint]) >
                     limit->max_acceleration_urad_s2)) {
                    return false;
                }
            }
        }
    }
    return true;
}

static bool first_point_matches_feedback(arm_visual_control_t *control)
{
    uint8_t joint;
    int32_t actual_position[ARM_PROTOCOL_JOINT_COUNT];
    int32_t actual_velocity[ARM_PROTOCOL_JOINT_COUNT];

    if (!read_feedback_ros(control, actual_position, actual_velocity)) {
        control->error_code = ARM_ERROR_FEEDBACK;
        return false;
    }

    for (joint = 0u; joint < ARM_PROTOCOL_JOINT_COUNT; ++joint) {
        if (abs_i64((int64_t)control->points[0].position_urad[joint] -
                    actual_position[joint]) >
            control->config.joint[joint].start_tolerance_urad) {
            control->error_code = ARM_ERROR_START_MISMATCH;
            return false;
        }
    }
    return true;
}

static arm_ack_status_t handle_begin(
    arm_visual_control_t *control,
    const arm_protocol_frame_t *frame)
{
    uint16_t point_count = arm_read_u16_le(frame->payload);

    if (!control->ready_permitted ||
        (control->public_state == ARM_STATE_ESTOP) ||
        (control->public_state == ARM_STATE_ERROR)) {
        return ARM_ACK_FAULT;
    }
    if (control->trajectory_state != ARM_TRAJECTORY_IDLE) {
        return ARM_ACK_BUSY;
    }
    if (control->pending_result.active) {
        return ARM_ACK_BUSY;
    }
    if ((point_count == 0u) ||
        (point_count > control->config.max_trajectory_points) ||
        (point_count > ARM_VISUAL_MAX_TRAJECTORY_POINTS)) {
        return ARM_ACK_OUT_OF_RANGE;
    }

    control->expected_points = point_count;
    control->received_points = 0u;
    control->current_segment = 0u;
    control->stable_since_ms = 0u;
    control->following_error_since_ms = 0u;
    control->error_code = ARM_ERROR_NONE;
    control->trajectory_state = ARM_TRAJECTORY_RECEIVING;
    return ARM_ACK_OK;
}

static arm_ack_status_t handle_point(
    arm_visual_control_t *control,
    const arm_protocol_frame_t *frame)
{
    uint16_t index;
    arm_trajectory_point_t point;

    if (control->trajectory_state != ARM_TRAJECTORY_RECEIVING) {
        return ARM_ACK_BUSY;
    }
    if (!decode_point(frame, &index, &point)) {
        return ARM_ACK_BAD_DATA;
    }
    if (index != control->received_points) {
        return ARM_ACK_MISSING_POINT;
    }
    if ((index >= control->expected_points) ||
        (index >= control->config.max_trajectory_points)) {
        return ARM_ACK_OUT_OF_RANGE;
    }
    if ((index > 0u) &&
        (point.time_ms <= control->points[index - 1u].time_ms)) {
        return ARM_ACK_BAD_DATA;
    }
    if (!point_within_waypoint_limits(control, &point)) {
        return ARM_ACK_OUT_OF_RANGE;
    }

    control->points[index] = point;
    control->received_points++;
    return ARM_ACK_OK;
}

static arm_ack_status_t handle_end(arm_visual_control_t *control)
{
    if (control->trajectory_state != ARM_TRAJECTORY_RECEIVING) {
        return ARM_ACK_BUSY;
    }
    if (control->received_points != control->expected_points) {
        return ARM_ACK_MISSING_POINT;
    }
    if (!sampled_trajectory_is_safe(control)) {
        control->trajectory_state = ARM_TRAJECTORY_IDLE;
        control->error_code = ARM_ERROR_BAD_TRAJECTORY;
        return ARM_ACK_OUT_OF_RANGE;
    }
    if (!first_point_matches_feedback(control)) {
        control->trajectory_state = ARM_TRAJECTORY_IDLE;
        return ARM_ACK_OUT_OF_RANGE;
    }

    control->execution_start_ms = now_ms(control);
    control->current_segment = 0u;
    control->trajectory_state = ARM_TRAJECTORY_EXECUTING;
    control->public_state = ARM_STATE_MOVING;
    return ARM_ACK_OK;
}

static void start_controlled_stop(
    arm_visual_control_t *control,
    arm_motion_result_t result,
    uint16_t error_code)
{
    if (control->trajectory_state == ARM_TRAJECTORY_STOPPING) {
        return;
    }
    control->hooks.request_controlled_stop(control->hooks_user);
    control->trajectory_state = ARM_TRAJECTORY_STOPPING;
    control->stopping_result = result;
    control->stopping_error_code = error_code;
}

static void send_motion_done(
    arm_visual_control_t *control,
    arm_motion_result_t result,
    uint16_t error_code)
{
    uint8_t payload[ARM_PAYLOAD_MOTION_DONE];
    size_t length;

    payload[0] = (uint8_t)result;
    arm_write_u16_le(&payload[1], error_code);

    control->pending_result.sequence = control->tx_sequence++;
    length = arm_protocol_encode(
        ARM_MSG_MOTION_DONE,
        control->pending_result.sequence,
        payload,
        sizeof(payload),
        control->pending_result.encoded_frame,
        sizeof(control->pending_result.encoded_frame));
    if (length == 0u) {
        return;
    }

    control->pending_result.active = true;
    control->pending_result.sent_once = false;
    control->pending_result.encoded_length = (uint8_t)length;
    control->pending_result.retries = 0u;
    control->pending_result.last_send_ms = now_ms(control);
}

static bool queue_claw_result(
    arm_visual_control_t *control,
    uint16_t command_sequence,
    arm_claw_result_t result)
{
    uint8_t payload[ARM_PAYLOAD_CLAW_RESULT];
    size_t length;

    if (control->pending_result.active) {
        return false;
    }
    payload[0] = (uint8_t)result;
    /* The result uses the original command SEQ, so no action/id field is needed. */
    control->pending_result.sequence = command_sequence;
    length = arm_protocol_encode(
        ARM_MSG_CLAW_RESULT,
        control->pending_result.sequence,
        payload,
        sizeof(payload),
        control->pending_result.encoded_frame,
        sizeof(control->pending_result.encoded_frame));
    if (length == 0u) {
        return false;
    }
    control->pending_result.active = true;
    control->pending_result.sent_once = false;
    control->pending_result.encoded_length = (uint8_t)length;
    control->pending_result.retries = 0u;
    control->pending_result.last_send_ms = now_ms(control);
    return true;
}

static arm_ack_status_t handle_claw_command(
    arm_visual_control_t *control,
    const arm_protocol_frame_t *frame)
{
    arm_claw_action_t action = (arm_claw_action_t)frame->payload[0];

    if ((action != ARM_CLAW_OPEN) &&
        (action != ARM_CLAW_CLOSE) &&
        (action != ARM_CLAW_STOP)) {
        return ARM_ACK_BAD_DATA;
    }
    if (!control->ready_permitted ||
        (control->public_state == ARM_STATE_ESTOP) ||
        (control->public_state == ARM_STATE_ERROR)) {
        return ARM_ACK_FAULT;
    }
    if ((control->trajectory_state != ARM_TRAJECTORY_IDLE) ||
        control->pending_result.active ||
        (control->claw_busy && (action != ARM_CLAW_STOP))) {
        return ARM_ACK_BUSY;
    }
    if ((control->hooks.request_claw_action == NULL) ||
        !control->hooks.request_claw_action(control->hooks_user, action)) {
        return ARM_ACK_FAULT;
    }
    if (action == ARM_CLAW_STOP) {
        if (control->claw_busy) {
            (void)queue_claw_result(
                control,
                control->claw_command_sequence,
                ARM_CLAW_INTERRUPTED);
        }
        control->claw_busy = false;
        control->public_state = control->ready_permitted
            ? ARM_STATE_READY
            : ARM_STATE_NOT_READY;
        return ARM_ACK_OK;
    }
    control->claw_action = action;
    control->claw_busy = true;
    control->claw_command_sequence = frame->sequence;
    control->claw_action_start_ms = now_ms(control);
    control->public_state = ARM_STATE_MOVING;
    return ARM_ACK_OK;
}

static void handle_ack_from_vision(
    arm_visual_control_t *control,
    const arm_protocol_frame_t *frame)
{
    if (control->pending_result.active &&
        (frame->sequence == control->pending_result.sequence) &&
        (frame->payload[0] == ARM_ACK_OK)) {
        control->pending_result.active = false;
    }
}

static void handle_frame(
    void *user,
    const arm_protocol_frame_t *frame)
{
    arm_visual_control_t *control = (arm_visual_control_t *)user;
    arm_sequence_record_t *previous;
    arm_ack_status_t status;
    uint16_t payload_crc;
    bool command_requires_ack;

    if ((control == NULL) || (frame == NULL)) {
        return;
    }

    control->last_valid_rx_ms = now_ms(control);

    if (frame->type == ARM_MSG_HEARTBEAT) {
        return;
    }
    if (frame->type == ARM_MSG_ACK) {
        handle_ack_from_vision(control, frame);
        return;
    }

    command_requires_ack =
        (frame->type == ARM_MSG_TRAJECTORY_BEGIN) ||
        (frame->type == ARM_MSG_TRAJECTORY_POINT) ||
        (frame->type == ARM_MSG_TRAJECTORY_END) ||
        (frame->type == ARM_MSG_STOP) ||
        (frame->type == ARM_MSG_CLAW_COMMAND);
    if (!command_requires_ack) {
        return;
    }

    previous = find_sequence(control, frame->type, frame->sequence);
    if (previous != NULL) {
        payload_crc = arm_crc16_ccitt_false(frame->payload, frame->payload_length);
        send_ack(
            control,
            frame->sequence,
            (payload_crc == previous->payload_crc)
                ? previous->status
                : ARM_ACK_BAD_DATA);
        return;
    }

    switch (frame->type) {
    case ARM_MSG_TRAJECTORY_BEGIN:
        status = handle_begin(control, frame);
        break;
    case ARM_MSG_TRAJECTORY_POINT:
        status = handle_point(control, frame);
        break;
    case ARM_MSG_TRAJECTORY_END:
        status = handle_end(control);
        break;
    case ARM_MSG_STOP:
        status = ARM_ACK_OK;
        if (control->claw_busy) {
            (void)control->hooks.request_claw_action(
                control->hooks_user, ARM_CLAW_STOP);
            (void)queue_claw_result(
                control,
                control->claw_command_sequence,
                ARM_CLAW_INTERRUPTED);
            control->claw_busy = false;
        }
        if ((control->trajectory_state == ARM_TRAJECTORY_EXECUTING) ||
            (control->trajectory_state == ARM_TRAJECTORY_STOPPING)) {
            start_controlled_stop(control, ARM_RESULT_CANCELLED, ARM_ERROR_NONE);
        } else {
            control->trajectory_state = ARM_TRAJECTORY_IDLE;
            control->expected_points = 0u;
            control->received_points = 0u;
            control->public_state = control->ready_permitted
                ? ARM_STATE_READY
                : ARM_STATE_NOT_READY;
        }
        break;
    case ARM_MSG_CLAW_COMMAND:
        status = handle_claw_command(control, frame);
        break;
    default:
        status = ARM_ACK_BAD_DATA;
        break;
    }

    save_sequence(control, frame, status);
    send_ack(control, frame->sequence, status);
}

static bool runtime_target_is_safe(
    const arm_visual_control_t *control,
    const double position[ARM_PROTOCOL_JOINT_COUNT],
    const double velocity[ARM_PROTOCOL_JOINT_COUNT],
    const double acceleration[ARM_PROTOCOL_JOINT_COUNT])
{
    uint8_t joint;

    for (joint = 0u; joint < ARM_PROTOCOL_JOINT_COUNT; ++joint) {
        const arm_joint_safety_config_t *limit = &control->config.joint[joint];
        double speed = (velocity[joint] < 0.0) ? -velocity[joint] : velocity[joint];
        double accel = (acceleration[joint] < 0.0) ? -acceleration[joint] : acceleration[joint];
        if ((position[joint] < limit->min_position_urad) ||
            (position[joint] > limit->max_position_urad) ||
            (speed > limit->max_velocity_urad_s) ||
            (accel > limit->max_acceleration_urad_s2)) {
            return false;
        }
    }
    return true;
}

static bool read_feedback(
    arm_visual_control_t *control,
    int32_t position[ARM_PROTOCOL_JOINT_COUNT],
    int32_t velocity[ARM_PROTOCOL_JOINT_COUNT])
{
    if (!read_feedback_ros(control, position, velocity)) {
        control->error_code = ARM_ERROR_FEEDBACK;
        control->public_state = ARM_STATE_ERROR;
        return false;
    }
    return true;
}

static bool following_error_ok(
    arm_visual_control_t *control,
    const int32_t target[ARM_PROTOCOL_JOINT_COUNT],
    uint32_t now)
{
    uint8_t joint;
    bool outside = false;
    int32_t actual_position[ARM_PROTOCOL_JOINT_COUNT];
    int32_t actual_velocity[ARM_PROTOCOL_JOINT_COUNT];

    if (!read_feedback(control, actual_position, actual_velocity)) {
        return false;
    }

    for (joint = 0u; joint < ARM_PROTOCOL_JOINT_COUNT; ++joint) {
        if (abs_i64((int64_t)target[joint] - actual_position[joint]) >
            control->config.joint[joint].following_error_urad) {
            outside = true;
            break;
        }
    }

    if (!outside) {
        control->following_error_since_ms = 0u;
        return true;
    }
    if (control->following_error_since_ms == 0u) {
        control->following_error_since_ms = now;
        return true;
    }
    return !elapsed_at_least(
        now,
        control->following_error_since_ms,
        control->config.following_error_duration_ms);
}

static bool completion_reached(arm_visual_control_t *control, uint32_t now)
{
    uint8_t joint;
    const arm_trajectory_point_t *last =
        &control->points[control->expected_points - 1u];
    int32_t actual_position[ARM_PROTOCOL_JOINT_COUNT];
    int32_t actual_velocity[ARM_PROTOCOL_JOINT_COUNT];

    if (!read_feedback(control, actual_position, actual_velocity)) {
        return false;
    }

    for (joint = 0u; joint < ARM_PROTOCOL_JOINT_COUNT; ++joint) {
        if ((abs_i64((int64_t)last->position_urad[joint] - actual_position[joint]) >
             control->config.joint[joint].goal_tolerance_urad) ||
            (abs_i64(actual_velocity[joint]) > control->config.stopped_velocity_urad_s)) {
            control->stable_since_ms = 0u;
            return false;
        }
    }

    if (control->stable_since_ms == 0u) {
        control->stable_since_ms = now;
        return false;
    }
    return elapsed_at_least(now, control->stable_since_ms, control->config.completion_stable_ms);
}

static void finish_success(arm_visual_control_t *control)
{
    control->trajectory_state = ARM_TRAJECTORY_IDLE;
    control->public_state = ARM_STATE_READY;
    control->error_code = ARM_ERROR_NONE;
    send_motion_done(control, ARM_RESULT_SUCCESS, ARM_ERROR_NONE);
}

bool arm_visual_control_init(
    arm_visual_control_t *control,
    const arm_visual_config_t *config,
    const arm_visual_hooks_t *hooks,
    void *hooks_user)
{
    uint8_t joint;

    if ((control == NULL) || (config == NULL) || (hooks == NULL) ||
        (hooks->get_monotonic_ms == NULL) ||
        (hooks->uart_transmit == NULL) ||
        (hooks->read_joint_feedback == NULL) ||
        (hooks->set_joint_target == NULL) ||
        (hooks->request_controlled_stop == NULL) ||
        (hooks->controlled_stop_complete == NULL) ||
        (hooks->estop_active == NULL) ||
        (hooks->driver_fault_active == NULL) ||
        (config->max_trajectory_points == 0u) ||
        (config->max_trajectory_points > ARM_VISUAL_MAX_TRAJECTORY_POINTS) ||
        (config->communication_timeout_ms == 0u) ||
        (config->execution_timeout_margin_ms == 0u) ||
        (config->following_error_duration_ms == 0u) ||
        (config->completion_stable_ms == 0u) ||
        (config->stopped_velocity_urad_s == 0u) ||
        (config->state_period_ms == 0u) ||
        (config->result_retry_ms == 0u) ||
        (config->claw_open_duration_ms == 0u) ||
        (config->claw_close_duration_ms == 0u) ||
        (config->result_max_retries == 0u)) {
        return false;
    }

    for (joint = 0u; joint < ARM_PROTOCOL_JOINT_COUNT; ++joint) {
        const arm_joint_safety_config_t *limit = &config->joint[joint];
        if (((limit->direction != 1) && (limit->direction != -1)) ||
            (limit->min_position_urad >= limit->max_position_urad) ||
            (limit->max_velocity_urad_s <= 0) ||
            (limit->max_acceleration_urad_s2 <= 0) ||
            (limit->start_tolerance_urad <= 0) ||
            (limit->following_error_urad <= 0) ||
            (limit->goal_tolerance_urad <= 0)) {
            return false;
        }
    }

    memset(control, 0, sizeof(*control));
    control->config = *config;
    control->hooks = *hooks;
    control->hooks_user = hooks_user;
    control->public_state = ARM_STATE_NOT_READY;
    control->trajectory_state = ARM_TRAJECTORY_IDLE;
    arm_protocol_parser_init(&control->parser, handle_frame, control);
    control->last_valid_rx_ms = now_ms(control);
    control->last_state_tx_ms = now_ms(control);
    return true;
}

void arm_visual_control_set_ready(arm_visual_control_t *control, bool ready)
{
    if (control == NULL) {
        return;
    }
    control->ready_permitted = ready;
    if (!ready) {
        control->public_state = ARM_STATE_NOT_READY;
    } else if ((control->trajectory_state == ARM_TRAJECTORY_IDLE) &&
               (control->error_code == ARM_ERROR_NONE)) {
        control->public_state = ARM_STATE_READY;
    }
}

bool arm_visual_control_clear_fault(arm_visual_control_t *control)
{
    if ((control == NULL) ||
        (control->trajectory_state != ARM_TRAJECTORY_IDLE) ||
        control->hooks.estop_active(control->hooks_user) ||
        control->hooks.driver_fault_active(control->hooks_user)) {
        return false;
    }

    control->error_code = ARM_ERROR_NONE;
    control->public_state = control->ready_permitted
        ? ARM_STATE_READY
        : ARM_STATE_NOT_READY;
    return true;
}

void arm_visual_control_feed(
    arm_visual_control_t *control,
    const uint8_t *data,
    size_t length)
{
    if (control == NULL) {
        return;
    }
    arm_protocol_parser_feed(&control->parser, data, length);
}

static void send_robot_state(arm_visual_control_t *control)
{
    uint8_t joint;
    uint8_t offset = 0u;
    uint8_t payload[ARM_PAYLOAD_ROBOT_STATE];
    uint8_t frame[34];
    size_t length;
    int32_t position[ARM_PROTOCOL_JOINT_COUNT];
    int32_t velocity[ARM_PROTOCOL_JOINT_COUNT];

    if (!read_feedback_ros(control, position, velocity)) {
        memset(position, 0, sizeof(position));
        control->public_state = ARM_STATE_ERROR;
        control->error_code = ARM_ERROR_FEEDBACK;
    }

    payload[offset++] = (uint8_t)control->public_state;
    arm_write_u16_le(&payload[offset], control->error_code);
    offset += 2u;
    for (joint = 0u; joint < ARM_PROTOCOL_JOINT_COUNT; ++joint) {
        arm_write_i32_le(&payload[offset], position[joint]);
        offset += 4u;
    }

    length = arm_protocol_encode(
        ARM_MSG_ROBOT_STATE,
        control->tx_sequence++,
        payload,
        sizeof(payload),
        frame,
        sizeof(frame));
    if (length != 0u) {
        transmit(control, frame, length);
    }
}

void arm_visual_control_service(arm_visual_control_t *control)
{
    uint32_t now;

    if (control == NULL) {
        return;
    }
    now = now_ms(control);

    if (elapsed_at_least(now, control->last_state_tx_ms, control->config.state_period_ms)) {
        control->last_state_tx_ms = now;
        send_robot_state(control);
    }

    if (control->pending_result.active &&
        !control->pending_result.sent_once) {
        control->pending_result.sent_once = true;
        control->pending_result.last_send_ms = now;
        transmit(
            control,
            control->pending_result.encoded_frame,
            control->pending_result.encoded_length);
    } else if (control->pending_result.active &&
               elapsed_at_least(
            now,
            control->pending_result.last_send_ms,
            control->config.result_retry_ms)) {
        if (control->pending_result.retries >= control->config.result_max_retries) {
            control->pending_result.active = false;
        } else {
            control->pending_result.retries++;
            control->pending_result.last_send_ms = now;
            transmit(
                control,
                control->pending_result.encoded_frame,
                control->pending_result.encoded_length);
        }
    }
}

void arm_visual_control_tick(arm_visual_control_t *control)
{
    uint8_t joint;
    uint32_t now;
    uint32_t trajectory_time;
    double position[ARM_PROTOCOL_JOINT_COUNT];
    double velocity[ARM_PROTOCOL_JOINT_COUNT];
    double acceleration[ARM_PROTOCOL_JOINT_COUNT];
    int32_t target_position[ARM_PROTOCOL_JOINT_COUNT];
    int32_t target_velocity[ARM_PROTOCOL_JOINT_COUNT];

    if (control == NULL) {
        return;
    }
    now = now_ms(control);

    if (control->hooks.estop_active(control->hooks_user)) {
        if (control->claw_busy) {
            (void)control->hooks.request_claw_action(
                control->hooks_user, ARM_CLAW_STOP);
            (void)queue_claw_result(
                control,
                control->claw_command_sequence,
                ARM_CLAW_INTERRUPTED);
            control->claw_busy = false;
        }
        if (control->public_state != ARM_STATE_ESTOP) {
            bool motion_was_active =
                (control->trajectory_state == ARM_TRAJECTORY_EXECUTING) ||
                (control->trajectory_state == ARM_TRAJECTORY_STOPPING);
            control->public_state = ARM_STATE_ESTOP;
            control->error_code = ARM_ERROR_ESTOP;
            control->trajectory_state = ARM_TRAJECTORY_IDLE;
            if (motion_was_active) {
                send_motion_done(control, ARM_RESULT_ESTOP, control->error_code);
            }
        }
        return;
    }
    if (control->hooks.driver_fault_active(control->hooks_user)) {
        if (control->claw_busy) {
            (void)control->hooks.request_claw_action(
                control->hooks_user, ARM_CLAW_STOP);
            (void)queue_claw_result(
                control,
                control->claw_command_sequence,
                ARM_CLAW_FAULT);
            control->claw_busy = false;
        }
        if (control->public_state != ARM_STATE_ERROR) {
            control->public_state = ARM_STATE_ERROR;
            control->error_code = ARM_ERROR_DRIVER_FAULT;
            if (control->trajectory_state == ARM_TRAJECTORY_EXECUTING) {
                start_controlled_stop(control, ARM_RESULT_FAILED, control->error_code);
            } else {
                control->trajectory_state = ARM_TRAJECTORY_IDLE;
            }
        }
    }

    if (control->claw_busy &&
        elapsed_at_least(
            now,
            control->last_valid_rx_ms,
            control->config.communication_timeout_ms)) {
        (void)control->hooks.request_claw_action(
            control->hooks_user, ARM_CLAW_STOP);
        (void)queue_claw_result(
            control,
            control->claw_command_sequence,
            ARM_CLAW_TIMEOUT);
        control->claw_busy = false;
        control->public_state = ARM_STATE_ERROR;
        control->error_code = ARM_ERROR_COMM_TIMEOUT;
        return;
    }

    if (control->claw_busy &&
        elapsed_at_least(
            now,
            control->claw_action_start_ms,
            (control->claw_action == ARM_CLAW_OPEN)
                ? control->config.claw_open_duration_ms
                : control->config.claw_close_duration_ms)) {
        (void)queue_claw_result(
            control,
            control->claw_command_sequence,
            ARM_CLAW_COMPLETED_UNVERIFIED);
        control->claw_busy = false;
        control->public_state = control->ready_permitted
            ? ARM_STATE_READY
            : ARM_STATE_NOT_READY;
    }

    if (control->trajectory_state == ARM_TRAJECTORY_STOPPING) {
        if (control->hooks.controlled_stop_complete(control->hooks_user)) {
            control->trajectory_state = ARM_TRAJECTORY_IDLE;
            control->public_state =
                (control->stopping_error_code == ARM_ERROR_NONE)
                ? ARM_STATE_READY
                : ARM_STATE_ERROR;
            control->error_code = control->stopping_error_code;
            send_motion_done(
                control,
                control->stopping_result,
                control->stopping_error_code);
        }
        return;
    }

    if ((control->trajectory_state == ARM_TRAJECTORY_RECEIVING) &&
        elapsed_at_least(
            now,
            control->last_valid_rx_ms,
            control->config.communication_timeout_ms)) {
        /* No motor has started: discard the partial trajectory safely. */
        control->trajectory_state = ARM_TRAJECTORY_IDLE;
        control->expected_points = 0u;
        control->received_points = 0u;
        control->public_state = control->ready_permitted
            ? ARM_STATE_READY
            : ARM_STATE_NOT_READY;
        return;
    }

    if (control->trajectory_state != ARM_TRAJECTORY_EXECUTING) {
        return;
    }

    if (elapsed_at_least(
            now,
            control->last_valid_rx_ms,
            control->config.communication_timeout_ms)) {
        control->error_code = ARM_ERROR_COMM_TIMEOUT;
        start_controlled_stop(control, ARM_RESULT_TIMEOUT, control->error_code);
        return;
    }

    trajectory_time = now - control->execution_start_ms;
    while ((control->current_segment + 1u < control->expected_points) &&
           (trajectory_time > control->points[control->current_segment + 1u].time_ms)) {
        control->current_segment++;
    }

    if (control->current_segment + 1u >= control->expected_points) {
        const arm_trajectory_point_t *last =
            &control->points[control->expected_points - 1u];
        for (joint = 0u; joint < ARM_PROTOCOL_JOINT_COUNT; ++joint) {
            target_position[joint] = last->position_urad[joint];
            target_velocity[joint] = 0;
        }
        set_target_from_ros(control, target_position, target_velocity);

        if (completion_reached(control, now)) {
            finish_success(control);
            return;
        }
        if (elapsed_at_least(
                trajectory_time,
                last->time_ms,
                control->config.execution_timeout_margin_ms)) {
            control->error_code = ARM_ERROR_EXECUTION_TIMEOUT;
            start_controlled_stop(control, ARM_RESULT_TIMEOUT, control->error_code);
        }
        return;
    }

    if (trajectory_time <= control->points[0].time_ms) {
        for (joint = 0u; joint < ARM_PROTOCOL_JOINT_COUNT; ++joint) {
            target_position[joint] = control->points[0].position_urad[joint];
            target_velocity[joint] = control->points[0].velocity_urad_s[joint];
        }
    } else {
        interpolate_segment(
            &control->points[control->current_segment],
            &control->points[control->current_segment + 1u],
            trajectory_time,
            position,
            velocity,
            acceleration);
        if (!runtime_target_is_safe(control, position, velocity, acceleration)) {
            control->error_code = ARM_ERROR_RUNTIME_LIMIT;
            start_controlled_stop(control, ARM_RESULT_FAILED, control->error_code);
            return;
        }
        for (joint = 0u; joint < ARM_PROTOCOL_JOINT_COUNT; ++joint) {
            target_position[joint] = round_to_i32(position[joint]);
            target_velocity[joint] = round_to_i32(velocity[joint]);
        }
    }

    if (!following_error_ok(control, target_position, now)) {
        control->error_code = ARM_ERROR_FOLLOWING;
        start_controlled_stop(control, ARM_RESULT_FAILED, control->error_code);
        return;
    }
    set_target_from_ros(control, target_position, target_velocity);
}

void arm_visual_control_report_fault(
    arm_visual_control_t *control,
    uint16_t error_code)
{
    if (control == NULL) {
        return;
    }
    control->error_code = error_code;
    control->public_state = ARM_STATE_ERROR;
    if (control->trajectory_state == ARM_TRAJECTORY_EXECUTING) {
        start_controlled_stop(control, ARM_RESULT_FAILED, error_code);
    }
}
