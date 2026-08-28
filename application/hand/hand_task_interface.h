#ifndef __HAND_TASK_INTERFACE__
#define __HAND_TASK_INTERFACE__

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  HAND_CLAW_REQUEST_OPEN = 0x01,
  HAND_CLAW_REQUEST_CLOSE = 0x02,
  HAND_CLAW_REQUEST_STOP = 0x03
} hand_claw_request_t;

typedef enum
{
  HAND_CLAW_REPORTED_UNKNOWN = 0x00,
  HAND_CLAW_REPORTED_OPENING = 0x01,
  HAND_CLAW_REPORTED_OPEN = 0x02,
  HAND_CLAW_REPORTED_CLOSING = 0x03,
  HAND_CLAW_REPORTED_CLOSED = 0x04,
  HAND_CLAW_REPORTED_FAULT = 0x05
} hand_claw_reported_state_t;

void hand_task_init(void);
void hand_task_get_feedback(void);
void hand_task_mode_flush(void);
void hand_task_set_output(void);
void hand_task_output(void);
void hand_claw_poll(void);

/*
 * Thread-safe request mailbox. The caller never drives PE13/PE9 directly;
 * HandTask consumes the request from hand_claw_poll(). OPEN/CLOSE are rejected
 * while RC2 manual claw control owns the outputs. STOP remains available as a
 * safety request.
 */
bool hand_claw_request(hand_claw_request_t request);
hand_claw_reported_state_t hand_claw_get_reported_state(void);

/*根据每个模式定义*/
//void __first_mode_ctrl_func(void);



#endif
