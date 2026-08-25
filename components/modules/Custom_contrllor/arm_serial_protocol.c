#include "arm_serial_protocol.h"

#include <string.h>

static uint16_t crc16_update(uint16_t crc, uint8_t byte)
{
    uint8_t bit;

    crc ^= (uint16_t)byte << 8;
    for (bit = 0; bit < 8u; ++bit) {
        if ((crc & 0x8000u) != 0u) {
            crc = (uint16_t)((crc << 1) ^ 0x1021u);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}
uint16_t arm_read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

uint32_t arm_read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0]
        | ((uint32_t)data[1] << 8)
        | ((uint32_t)data[2] << 16)
        | ((uint32_t)data[3] << 24);
}

int32_t arm_read_i32_le(const uint8_t *data)
{
    return (int32_t)arm_read_u32_le(data);
}

void arm_write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)(value >> 8);
}

void arm_write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)((value >> 8) & 0xFFu);
    data[2] = (uint8_t)((value >> 16) & 0xFFu);
    data[3] = (uint8_t)((value >> 24) & 0xFFu);
}

void arm_write_i32_le(uint8_t *data, int32_t value)
{
    arm_write_u32_le(data, (uint32_t)value);
}

uint16_t arm_crc16_ccitt_false(const uint8_t *data, size_t length)
{
    size_t i;
    uint16_t crc = 0xFFFFu;

    if ((data == NULL) && (length != 0u)) {
        return 0u;
    }

    for (i = 0; i < length; ++i) {
        crc = crc16_update(crc, data[i]);
    }
    return crc;
}

int16_t arm_protocol_payload_length(uint8_t type)
{
    switch (type) {
    case ARM_MSG_TRAJECTORY_BEGIN:
        return ARM_PAYLOAD_TRAJECTORY_BEGIN;
    case ARM_MSG_TRAJECTORY_POINT:
        return ARM_PAYLOAD_TRAJECTORY_POINT;
    case ARM_MSG_TRAJECTORY_END:
        return ARM_PAYLOAD_TRAJECTORY_END;
    case ARM_MSG_STOP:
        return ARM_PAYLOAD_STOP;
    case ARM_MSG_HEARTBEAT:
        return ARM_PAYLOAD_HEARTBEAT;
    case ARM_MSG_CLAW_COMMAND:
        return ARM_PAYLOAD_CLAW_COMMAND;
    case ARM_MSG_ACK:
        return ARM_PAYLOAD_ACK;
    case ARM_MSG_ROBOT_STATE:
        return ARM_PAYLOAD_ROBOT_STATE;
    case ARM_MSG_MOTION_DONE:
        return ARM_PAYLOAD_MOTION_DONE;
    case ARM_MSG_CLAW_RESULT:
        return ARM_PAYLOAD_CLAW_RESULT;
    default:
        return -1;
    }
}

void arm_protocol_parser_reset(arm_protocol_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }

    parser->state = ARM_RX_WAIT_AA;
    parser->payload_index = 0u;
    parser->received_crc = 0u;
    parser->running_crc = 0xFFFFu;
    parser->frame.type = 0u;
    parser->frame.sequence = 0u;
    parser->frame.payload_length = 0u;
}

void arm_protocol_parser_init(
    arm_protocol_parser_t *parser,
    arm_protocol_frame_callback_t callback,
    void *callback_user)
{
    if (parser == NULL) {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->callback = callback;
    parser->callback_user = callback_user;
    arm_protocol_parser_reset(parser);
}

void arm_protocol_parser_feed_byte(arm_protocol_parser_t *parser, uint8_t byte)
{
    int16_t expected_length;

    if (parser == NULL) {
        return;
    }

    switch (parser->state) {
    case ARM_RX_WAIT_AA:
        if (byte == ARM_PROTOCOL_MAGIC_1) {
            parser->state = ARM_RX_WAIT_55;
        }
        break;

    case ARM_RX_WAIT_55:
        if (byte == ARM_PROTOCOL_MAGIC_2) {
            parser->state = ARM_RX_READ_TYPE;
        } else if (byte != ARM_PROTOCOL_MAGIC_1) {
            parser->state = ARM_RX_WAIT_AA;
        }
        break;

    case ARM_RX_READ_TYPE:
        expected_length = arm_protocol_payload_length(byte);
        if ((expected_length < 0) ||
            (expected_length > (int16_t)ARM_PROTOCOL_MAX_PAYLOAD)) {
            parser->unknown_type_errors++;
            arm_protocol_parser_reset(parser);
            break;
        }

        parser->frame.type = byte;
        parser->frame.payload_length = (uint8_t)expected_length;
        parser->running_crc = crc16_update(0xFFFFu, byte);
        parser->state = ARM_RX_READ_SEQ_LOW;
        break;

    case ARM_RX_READ_SEQ_LOW:
        parser->frame.sequence = byte;
        parser->running_crc = crc16_update(parser->running_crc, byte);
        parser->state = ARM_RX_READ_SEQ_HIGH;
        break;

    case ARM_RX_READ_SEQ_HIGH:
        parser->frame.sequence |= (uint16_t)byte << 8;
        parser->running_crc = crc16_update(parser->running_crc, byte);
        parser->payload_index = 0u;
        parser->state = (parser->frame.payload_length == 0u)
            ? ARM_RX_READ_CRC_LOW
            : ARM_RX_READ_PAYLOAD;
        break;

    case ARM_RX_READ_PAYLOAD:
        parser->frame.payload[parser->payload_index++] = byte;
        parser->running_crc = crc16_update(parser->running_crc, byte);
        if (parser->payload_index >= parser->frame.payload_length) {
            parser->state = ARM_RX_READ_CRC_LOW;
        }
        break;

    case ARM_RX_READ_CRC_LOW:
        parser->received_crc = byte;
        parser->state = ARM_RX_READ_CRC_HIGH;
        break;

    case ARM_RX_READ_CRC_HIGH:
        parser->received_crc |= (uint16_t)byte << 8;
        if (parser->received_crc == parser->running_crc) {
            parser->valid_frames++;
            if (parser->callback != NULL) {
                parser->callback(parser->callback_user, &parser->frame);
            }
        } else {
            parser->crc_errors++;
        }
        arm_protocol_parser_reset(parser);
        break;

    default:
        arm_protocol_parser_reset(parser);
        break;
    }
}

void arm_protocol_parser_feed(
    arm_protocol_parser_t *parser,
    const uint8_t *data,
    size_t length)
{
    size_t i;

    if ((parser == NULL) || ((data == NULL) && (length != 0u))) {
        return;
    }

    for (i = 0; i < length; ++i) {
        arm_protocol_parser_feed_byte(parser, data[i]);
    }
}

size_t arm_protocol_encode(
    uint8_t type,
    uint16_t sequence,
    const uint8_t *payload,
    uint8_t payload_length,
    uint8_t *output,
    size_t output_capacity)
{
    int16_t expected_length;
    uint16_t crc;
    size_t frame_length;

    expected_length = arm_protocol_payload_length(type);
    if ((expected_length < 0) ||
        ((uint8_t)expected_length != payload_length) ||
        ((payload == NULL) && (payload_length != 0u)) ||
        (output == NULL)) {
        return 0u;
    }

    frame_length = (size_t)payload_length + 7u;
    if (output_capacity < frame_length) {
        return 0u;
    }

    output[0] = ARM_PROTOCOL_MAGIC_1;
    output[1] = ARM_PROTOCOL_MAGIC_2;
    output[2] = type;
    arm_write_u16_le(&output[3], sequence);
    if (payload_length != 0u) {
        memcpy(&output[5], payload, payload_length);
    }

    crc = arm_crc16_ccitt_false(&output[2], (size_t)payload_length + 3u);
    arm_write_u16_le(&output[5u + payload_length], crc);
    return frame_length;
}
