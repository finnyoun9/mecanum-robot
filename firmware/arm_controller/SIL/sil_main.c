/**
 * @file sil_main.c — Software-in-the-Loop entry for the LeArm arm controller.
 *
 * Compiles the arm controller core (app/drivers) as a Linux binary with a
 * mock servo bus, feeds protocol frames, and verifies:
 *   - SET_POS limit validation + bus writes
 *   - TORQUE on/off → ACTIVE/IDLE
 *   - ESTOP → FAULT + torque-off
 *   - servo write failure → ARM_FAULT_SERVO_COMM
 *   - host timeout → ARM_FAULT_HOST_TIMEOUT
 *   - RESET recovers to IDLE
 *   - GET_STATE encodes a clean CMD_ARM_STATE report
 *
 * Usage:
 *   ./build/sil_arm_controller --ci   # run scenario, exit PASS/FAIL
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "protocol.h"
#include "host_protocol.h"
#include "mock_servo_bus.h"

static int pass = 0, fail = 0;

#define CHECK(cond, msg) do {                                          \
    if (cond) { pass++; printf("PASS: %s\n", msg); }                   \
    else      { fail++; printf("FAIL: %s\n", msg); }                   \
} while (0)

/* Joint soft limits: [0..4] axes, [5] gripper pinch range. */
static const arm_joint_limit_t DEFAULT_LIMITS[ARM_JOINT_COUNT] = {
    { -3.14f, 3.14f }, { -3.14f, 3.14f }, { -2.0f, 2.0f },
    { -2.0f, 2.0f },   { -3.14f, 3.14f }, { 0.0f, 0.1f },
};

/* Encode a host frame and push it into the controller. */
static void feed(arm_controller_t *ac, uint32_t now,
                 uint8_t cmd, const void *payload, uint8_t pay_len,
                 uint8_t *tx, uint8_t *tx_len) {
    uint8_t frame[PROTO_MAX_FRAME];
    uint8_t frm_len;
    static uint8_t seq = 0;
    if (proto_encode(cmd, (const uint8_t *)payload, pay_len, frame, &frm_len, seq++) < 0) {
        printf("setup error: encode failed\n");
        exit(2);
    }
    arm_controller_handle_frame(ac, frame, frm_len, now, tx, tx_len);
}

static void reset_to_idle(arm_controller_t *ac, uint32_t now) {
    uint8_t tx[PROTO_MAX_FRAME], tx_len = 0;
    feed(ac, now, CMD_ARM_RESET, NULL, 0, tx, &tx_len);
}

int main(int argc, char **argv) {
    (void)argc; (void)argv;

    arm_controller_t ac;
    arm_controller_init(&ac, DEFAULT_LIMITS, 500u); /* 500 ms host timeout */

    uint32_t now = 1000;
    uint8_t tx[PROTO_MAX_FRAME], tx_len = 0;

    arm_set_pos_t sp = {{ 0.5f, 0.2f, 0.1f, 0.0f, -0.2f, 0.05f }};

    /* 1. ESTOP → FAULT_ESTOP, torque off, no bus writes */
    mock_bus_reset();
    feed(&ac, now, CMD_ARM_ESTOP, NULL, 0, tx, &tx_len);
    CHECK(ac.safety.state == SAFETY_FAULT, "ESTOP latches FAULT state");
    CHECK(ac.safety.fault_flags & ARM_FAULT_ESTOP, "ESTOP sets ARM_FAULT_ESTOP");
    arm_controller_tick(&ac, now);
    CHECK(mock_bus_write_count() == 0, "no bus writes while in FAULT");

    /* 2. SET_POS rejected while FAULT latched */
    feed(&ac, now, CMD_ARM_SET_POS, &sp, sizeof(sp), tx, &tx_len);
    CHECK(ac.safety.state == SAFETY_FAULT, "SET_POS rejected while FAULT latched");

    /* 3. RESET → IDLE, faults cleared */
    reset_to_idle(&ac, now);
    CHECK(ac.safety.state == SAFETY_IDLE, "RESET returns to IDLE");
    CHECK(ac.safety.fault_flags == ARM_FAULT_NONE, "RESET clears fault flags");

    /* 4. torque all on → ACTIVE */
    arm_torque_cmd_t tc = { .mask = 0x3F };
    feed(&ac, now, CMD_ARM_TORQUE, &tc, sizeof(tc), tx, &tx_len);
    CHECK(ac.safety.state == SAFETY_ACTIVE, "TORQUE on → ACTIVE");

    /* 5. valid SET_POS → all 6 targets written to the bus */
    mock_bus_reset();
    feed(&ac, now, CMD_ARM_SET_POS, &sp, sizeof(sp), tx, &tx_len);
    arm_controller_tick(&ac, now);
    CHECK(mock_bus_write_count() == ARM_JOINT_COUNT, "all 6 joints written to bus");
    CHECK(mock_bus_last_joint() == 5, "gripper is joint 5 (last written)");
    CHECK(fabsf(mock_bus_last_pos() - sp.joint[5]) < 1e-6f, "gripper target written");

    /* 6. out-of-limit SET_POS (joint 2 > 2.0 rad) → FAULT_SOFT_LIMIT + torque off */
    arm_set_pos_t bad = {{ 0.5f, 0.2f, 5.0f, 0.0f, -0.2f, 0.05f }};
    mock_bus_reset();
    feed(&ac, now, CMD_ARM_SET_POS, &bad, sizeof(bad), tx, &tx_len);
    CHECK(ac.safety.state == SAFETY_FAULT, "out-of-limit SET_POS → FAULT");
    CHECK(ac.safety.fault_flags & ARM_FAULT_SOFT_LIMIT, "ARM_FAULT_SOFT_LIMIT set");
    CHECK(ac.ctrl.torque_mask == 0, "torque dropped on soft-limit FAULT");

    /* 7. servo write failure → ARM_FAULT_SERVO_COMM */
    reset_to_idle(&ac, now);
    arm_torque_cmd_t tc1 = { .mask = 0x01 };
    feed(&ac, now, CMD_ARM_TORQUE, &tc1, sizeof(tc1), tx, &tx_len);
    feed(&ac, now, CMD_ARM_SET_POS, &sp, sizeof(sp), tx, &tx_len);
    mock_bus_reset();
    mock_bus_fail_next();
    arm_controller_tick(&ac, now);
    CHECK(ac.safety.state == SAFETY_FAULT, "servo write failure → FAULT");
    CHECK(ac.safety.fault_flags & ARM_FAULT_SERVO_COMM, "ARM_FAULT_SERVO_COMM set");
    CHECK(ac.ctrl.torque_mask == 0, "torque dropped on servo-comm FAULT");

    /* 8. host timeout (> 500 ms without a frame) → ARM_FAULT_HOST_TIMEOUT */
    reset_to_idle(&ac, now);
    feed(&ac, now, CMD_ARM_TORQUE, &tc1, sizeof(tc1), tx, &tx_len);
    now += 1000;
    arm_controller_tick(&ac, now);
    CHECK(ac.safety.state == SAFETY_FAULT, "host timeout → FAULT");
    CHECK(ac.safety.fault_flags & ARM_FAULT_HOST_TIMEOUT, "ARM_FAULT_HOST_TIMEOUT set");

    /* 9. GET_STATE returns a clean CMD_ARM_STATE report after RESET */
    reset_to_idle(&ac, now);
    feed(&ac, now, CMD_ARM_GET_STATE, NULL, 0, tx, &tx_len);
    CHECK(tx_len >= 7 + sizeof(arm_state_t), "GET_STATE response correctly sized");
    {
        uint8_t cmd, payload[PROTO_MAX_PAYLOAD], pay_len, seq;
        CHECK(proto_decode(tx, tx_len, &cmd, payload, &pay_len, &seq) >= 0,
              "GET_STATE response decodes cleanly");
        CHECK(cmd == CMD_ARM_STATE, "response cmd is CMD_ARM_STATE");
        arm_state_t st;
        memcpy(&st, payload, sizeof(st));
        CHECK(st.fault_flags == ARM_FAULT_NONE, "state fault flags clean after RESET");
        CHECK(st.torque_mask == 0, "state reports torque off after RESET");
    }

    /* 10. corrupted frame is rejected at the framing layer */
    {
        uint8_t frame[PROTO_MAX_FRAME], frm_len;
        proto_encode(CMD_ARM_SET_POS, (const uint8_t *)&sp, sizeof(sp), frame, &frm_len, 0);
        frame[frm_len - 1] ^= 0xFF; /* flip a CRC byte */
        int r = arm_controller_handle_frame(&ac, frame, frm_len, now, tx, &tx_len);
        CHECK(r == -1, "corrupted frame rejected");
    }

    printf("\narm_controller SIL: %d passed, %d failed\n", pass, fail);
    return (fail == 0) ? 0 : 1;
}
