#ifndef ARM_SERIAL_PROTOCOL_H
#define ARM_SERIAL_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
#define ARM_PROTOCOL_MAGIC_1              0xAAu
#define ARM_PROTOCOL_MAGIC_2              0x55u
#define ARM_PROTOCOL_JOINT_COUNT           6u
#define ARM_PROTOCOL_MAX_PAYLOAD           54u
#define ARM_PROTOCOL_MAX_FRAME_SIZE        61u

/* Vision -> controller. */
#define ARM_MSG_TRAJECTORY_BEGIN           0x01u
#define ARM_MSG_TRAJECTORY_POINT           0x02u
#define ARM_MSG_TRAJECTORY_END             0x03u
#define ARM_MSG_STOP                       0x04u
#define ARM_MSG_HEARTBEAT                  0x05u
#define ARM_MSG_CLAW_COMMAND               0x06u

/* Bidirectional / controller -> vision. */
#define ARM_MSG_ACK                        0x80u
#define ARM_MSG_ROBOT_STATE                0x81u
#define ARM_MSG_MOTION_DONE                0x82u
#define ARM_MSG_CLAW_RESULT                0x83u

#define ARM_PAYLOAD_TRAJECTORY_BEGIN       2u
#define ARM_PAYLOAD_TRAJECTORY_POINT       54u
#define ARM_PAYLOAD_TRAJECTORY_END         0u
#define ARM_PAYLOAD_STOP                   0u
#define ARM_PAYLOAD_HEARTBEAT              0u
#define ARM_PAYLOAD_CLAW_COMMAND           1u
#define ARM_PAYLOAD_ACK                     1u
#define ARM_PAYLOAD_ROBOT_STATE            27u
#define ARM_PAYLOAD_MOTION_DONE            3u
#define ARM_PAYLOAD_CLAW_RESULT            1u

typedef enum {
    ARM_ACK_OK = 0x00,
    ARM_ACK_BAD_DATA = 0x01,
    ARM_ACK_BUSY = 0x02,
    ARM_ACK_OUT_OF_RANGE = 0x03,
    ARM_ACK_MISSING_POINT = 0x04,
    ARM_ACK_FAULT = 0x05
} arm_ack_status_t;

typedef enum {
    ARM_STATE_NOT_READY = 0x00,
    ARM_STATE_READY = 0x01,
    ARM_STATE_MOVING = 0x02,
    ARM_STATE_ERROR = 0x03,
    ARM_STATE_ESTOP = 0x04
} arm_public_state_t;

typedef enum {
    ARM_RESULT_SUCCESS = 0x00,
    ARM_RESULT_FAILED = 0x01,
    ARM_RESULT_CANCELLED = 0x02,
    ARM_RESULT_ESTOP = 0x03,
    ARM_RESULT_TIMEOUT = 0x04
} arm_motion_result_t;

typedef enum {
    ARM_CLAW_OPEN = 0x01,
    ARM_CLAW_CLOSE = 0x02,
    ARM_CLAW_STOP = 0x03
} arm_claw_action_t;

typedef enum {
    ARM_CLAW_COMPLETED_UNVERIFIED = 0x00,
    ARM_CLAW_INTERRUPTED = 0x01,
    ARM_CLAW_TIMEOUT = 0x02,
    ARM_CLAW_FAULT = 0x03
} arm_claw_result_t;

typedef struct {
    uint8_t type;
    uint16_t sequence;
    uint8_t payload[ARM_PROTOCOL_MAX_PAYLOAD];
    uint8_t payload_length;
} arm_protocol_frame_t;

typedef void (*arm_protocol_frame_callback_t)(
    void *user,
    const arm_protocol_frame_t *frame);

typedef enum {
    ARM_RX_WAIT_AA = 0,
    ARM_RX_WAIT_55,
    ARM_RX_READ_TYPE,
    ARM_RX_READ_SEQ_LOW,
    ARM_RX_READ_SEQ_HIGH,
    ARM_RX_READ_PAYLOAD,
    ARM_RX_READ_CRC_LOW,
    ARM_RX_READ_CRC_HIGH
} arm_protocol_rx_state_t;

typedef struct {
    arm_protocol_rx_state_t state;
    arm_protocol_frame_t frame;
    uint8_t payload_index;
    uint16_t received_crc;
    uint16_t running_crc;

    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t unknown_type_errors;

    arm_protocol_frame_callback_t callback;
    void *callback_user;
} arm_protocol_parser_t;

uint16_t arm_read_u16_le(const uint8_t *data);
uint32_t arm_read_u32_le(const uint8_t *data);
int32_t arm_read_i32_le(const uint8_t *data);
void arm_write_u16_le(uint8_t *data, uint16_t value);
void arm_write_u32_le(uint8_t *data, uint32_t value);
void arm_write_i32_le(uint8_t *data, int32_t value);

uint16_t arm_crc16_ccitt_false(const uint8_t *data, size_t length);
int16_t arm_protocol_payload_length(uint8_t type);

void arm_protocol_parser_init(
    arm_protocol_parser_t *parser,
    arm_protocol_frame_callback_t callback,
    void *callback_user);

void arm_protocol_parser_reset(arm_protocol_parser_t *parser);
void arm_protocol_parser_feed_byte(arm_protocol_parser_t *parser, uint8_t byte);
void arm_protocol_parser_feed(
    arm_protocol_parser_t *parser,
    const uint8_t *data,
    size_t length);

size_t arm_protocol_encode(
    uint8_t type,
    uint16_t sequence,
    const uint8_t *payload,
    uint8_t payload_length,
    uint8_t *output,
    size_t output_capacity);

#ifdef __cplusplus
}
#endif

#endif
