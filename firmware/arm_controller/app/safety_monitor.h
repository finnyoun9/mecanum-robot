/**
 * @file safety_monitor.h
 * @brief Arm safety state machine: soft limits, host timeout, e-stop, FAULT.
 *
 * State machine maps to the architecture doc:
 *   BOOT -> DISCOVERY -> CALIBRATION -> IDLE -> ACTIVE
 *                                    |       |
 *                                    +-> FAULT <-+
 *
 * FAULT latches until an explicit CMD_ARM_RESET; on entering FAULT the
 * controller drops all torque (see host_protocol.c).
 */

#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <stdint.h>
#include "protocol.h"   /* ARM_JOINT_COUNT, ARM_FAULT_* */

typedef enum {
    SAFETY_BOOT = 0,
    SAFETY_DISCOVERY,     /* reserved: vendor servo scan */
    SAFETY_CALIBRATION,   /* reserved: joint zero calibration */
    SAFETY_IDLE,          /* armed, no bus writes yet */
    SAFETY_ACTIVE,        /* normal operation */
    SAFETY_FAULT
} safety_state_t;

typedef struct {
    float min_rad;
    float max_rad;
} arm_joint_limit_t;

typedef struct {
    safety_state_t        state;
    uint8_t               fault_flags;      /* ARM_FAULT_* bitmask, latched */
    uint32_t              last_host_ms;     /* last valid host frame time */
    uint32_t              host_timeout_ms;
    const arm_joint_limit_t *limits;        /* per-joint soft limits */
} safety_monitor_t;

void safety_monitor_init(safety_monitor_t *sm, const arm_joint_limit_t *limits,
                         uint32_t host_timeout_ms);
void safety_monitor_on_host_frame(safety_monitor_t *sm, uint32_t now_ms);
int  safety_monitor_validate_targets(safety_monitor_t *sm,
                                     const float pos[ARM_JOINT_COUNT]); /* 0 ok, -1 → FAULT */
void safety_monitor_trigger_estop(safety_monitor_t *sm);
void safety_monitor_reset(safety_monitor_t *sm);
void safety_monitor_tick(safety_monitor_t *sm, uint32_t now_ms); /* host-timeout check */

#endif /* SAFETY_MONITOR_H */
