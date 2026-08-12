#include "safety_monitor.h"

void safety_monitor_init(safety_monitor_t *sm, const arm_joint_limit_t *limits,
                         uint32_t host_timeout_ms) {
    sm->state           = SAFETY_IDLE;
    sm->fault_flags     = ARM_FAULT_NONE;
    sm->last_host_ms    = 0;
    sm->host_timeout_ms = host_timeout_ms;
    sm->limits          = limits;
}

void safety_monitor_on_host_frame(safety_monitor_t *sm, uint32_t now_ms) {
    sm->last_host_ms = now_ms;
}

int safety_monitor_validate_targets(safety_monitor_t *sm,
                                    const float pos[ARM_JOINT_COUNT]) {
    for (uint8_t j = 0; j < ARM_JOINT_COUNT; j++) {
        if (pos[j] < sm->limits[j].min_rad || pos[j] > sm->limits[j].max_rad) {
            sm->state = SAFETY_FAULT;
            sm->fault_flags |= ARM_FAULT_SOFT_LIMIT;
            return -1;
        }
    }
    return 0;
}

void safety_monitor_trigger_estop(safety_monitor_t *sm) {
    sm->state = SAFETY_FAULT;
    sm->fault_flags |= ARM_FAULT_ESTOP;
}

void safety_monitor_reset(safety_monitor_t *sm) {
    sm->state       = SAFETY_IDLE;
    sm->fault_flags = ARM_FAULT_NONE;
}

void safety_monitor_tick(safety_monitor_t *sm, uint32_t now_ms) {
    if (sm->state == SAFETY_FAULT) {
        return; /* latched until explicit reset */
    }
    /* Unsigned arithmetic wraps safely across the 2^32 ms epoch. */
    if ((now_ms - sm->last_host_ms) > sm->host_timeout_ms) {
        sm->state = SAFETY_FAULT;
        sm->fault_flags |= ARM_FAULT_HOST_TIMEOUT;
    }
}
