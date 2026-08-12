/**
 * @file host_protocol.h
 * @brief Pi ↔ arm controller host link: frame dispatch + orchestration.
 *
 * Parses host frames with the shared framing (shared/protocol.h), routes
 * them to arm_control / safety_monitor, and encodes CMD_ARM_STATE feedback.
 * Response frames are returned via out-params; on real firmware they are
 * written to the UART TX ring instead.
 */

#ifndef HOST_PROTOCOL_H
#define HOST_PROTOCOL_H

#include <stdint.h>
#include "protocol.h"       /* shared framing, ARM_* cmd IDs */
#include "arm_control.h"
#include "safety_monitor.h"

typedef struct {
    arm_control_t    ctrl;
    safety_monitor_t safety;
} arm_controller_t;

int arm_controller_init(arm_controller_t *ac, const arm_joint_limit_t *limits,
                        uint32_t host_timeout_ms);
/* Handles one host frame. Writes a response to tx_frame/tx_len when one is
 * due (max PROTO_MAX_FRAME). Returns 0 on handled frame, -1 on malformed
 * or rejected command. */
int arm_controller_handle_frame(arm_controller_t *ac, const uint8_t *frame, uint8_t len,
                                uint32_t now_ms, uint8_t *tx_frame, uint8_t *tx_len);
/* Periodic control tick; pushes joint targets to the servo bus. */
void arm_controller_tick(arm_controller_t *ac, uint32_t now_ms);

#endif /* HOST_PROTOCOL_H */
