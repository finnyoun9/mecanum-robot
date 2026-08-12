#include "host_protocol.h"
#include <string.h>

int arm_controller_init(arm_controller_t *ac, const arm_joint_limit_t *limits,
                        uint32_t host_timeout_ms) {
    arm_control_init(&ac->ctrl);
    safety_monitor_init(&ac->safety, limits, host_timeout_ms);
    return servo_bus_init();
}

static void arm_controller_fault(arm_controller_t *ac) {
    /* On FAULT: drop all torque; bus writes stop at the next tick. */
    arm_control_torque_off(&ac->ctrl);
}

int arm_controller_handle_frame(arm_controller_t *ac, const uint8_t *frame, uint8_t len,
                                uint32_t now_ms, uint8_t *tx_frame, uint8_t *tx_len) {
    uint8_t cmd, payload[PROTO_MAX_PAYLOAD], pay_len, seq;
    if (proto_decode(frame, len, &cmd, payload, &pay_len, &seq) < 0) {
        return -1;
    }

    /* Any valid frame proves the host is alive. */
    safety_monitor_on_host_frame(&ac->safety, now_ms);

    safety_monitor_t *sm = &ac->safety;
    arm_control_t   *ct = &ac->ctrl;

    switch (cmd) {
    case CMD_ARM_SET_POS: {
        if (pay_len != sizeof(arm_set_pos_t)) {
            return -1;
        }
        if (sm->state == SAFETY_FAULT) {
            return -1; /* locked out until CMD_ARM_RESET */
        }
        arm_set_pos_t *sp = (arm_set_pos_t *)payload;
        if (safety_monitor_validate_targets(sm, sp->joint) < 0) {
            arm_controller_fault(ac);
            return 0; /* fault latched; SET_POS rejected */
        }
        arm_control_set_targets(ct, sp->joint);
        if (ct->torque_mask != 0 && sm->state == SAFETY_IDLE) {
            sm->state = SAFETY_ACTIVE;
        }
        return 0;
    }
    case CMD_ARM_TORQUE: {
        if (pay_len != sizeof(arm_torque_cmd_t)) {
            return -1;
        }
        if (sm->state == SAFETY_FAULT) {
            return -1;
        }
        arm_control_set_torque(ct, ((arm_torque_cmd_t *)payload)->mask);
        sm->state = (ct->torque_mask == 0) ? SAFETY_IDLE : SAFETY_ACTIVE;
        return 0;
    }
    case CMD_ARM_ESTOP:
        safety_monitor_trigger_estop(sm);
        arm_controller_fault(ac);
        return 0;
    case CMD_ARM_RESET:
        safety_monitor_reset(sm); /* torque stays off until host re-energizes */
        return 0;
    case CMD_ARM_GET_STATE: {
        arm_state_t st;
        memset(&st, 0, sizeof(st));
        memcpy(st.joint_pos, ct->target, sizeof(ct->target));
        st.torque_mask = ct->torque_mask;
        st.fault_flags = sm->fault_flags;
        st.temperature_c = 0; /* sensor not wired in skeleton */
        if (proto_encode(CMD_ARM_STATE, (const uint8_t *)&st, sizeof(st),
                         tx_frame, tx_len, 0) < 0) {
            return -1;
        }
        return 0;
    }
    case CMD_HEARTBEAT: /* generic keep-alive, reused from base protocol */
        return 0;
    default:
        return -1;
    }
}

void arm_controller_tick(arm_controller_t *ac, uint32_t now_ms) {
    safety_monitor_tick(&ac->safety, now_ms);
    if (ac->safety.state == SAFETY_FAULT) {
        /* Uniform rule: every FAULT entry de-energizes. Host must re-energize
         * via CMD_ARM_TORQUE after CMD_ARM_RESET. */
        arm_control_torque_off(&ac->ctrl);
        return; /* no bus writes while in FAULT */
    }
    /* A failed servo write latches a comm FAULT (real driver would retry
     * N times before declaring a fault; single-shot is the skeleton rule). */
    if (arm_control_tick(&ac->ctrl) > 0) {
        ac->safety.state = SAFETY_FAULT;
        ac->safety.fault_flags |= ARM_FAULT_SERVO_COMM;
        arm_control_torque_off(&ac->ctrl);
    }
}
