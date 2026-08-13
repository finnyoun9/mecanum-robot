/**
 * @file test_protocol.c
 * @brief Self-contained unit tests for protocol.c (CRC16, encode/decode).
 * Compiles without dependencies: gcc -std=c11 -I shared shared/tests/test_protocol.c shared/protocol.c -o test_protocol
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include "protocol.h"

/* ===== CRC16 Tests ===== */

void test_crc16_empty_buffer(void) {
    uint16_t crc = proto_crc16(NULL, 0);
    assert(crc == 0xFFFF);
    printf("PASS: CRC-16 empty buffer returns 0xFFFF\n");
}

void test_crc16_determinism(void) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    uint16_t crc1 = proto_crc16(data, sizeof(data));
    uint16_t crc2 = proto_crc16(data, sizeof(data));
    assert(crc1 == crc2);
    printf("PASS: CRC-16 is deterministic (crc=%04x)\n", crc1);
}

void test_crc16_different_inputs(void) {
    uint8_t data1[] = {0x01, 0x02, 0x03};
    uint8_t data2[] = {0x01, 0x02, 0x04};
    uint16_t crc1 = proto_crc16(data1, sizeof(data1));
    uint16_t crc2 = proto_crc16(data2, sizeof(data2));
    assert(crc1 != crc2);
    printf("PASS: CRC-16 differs for different inputs (crc1=%04x, crc2=%04x)\n", crc1, crc2);
}

/* ===== Encode/Decode Round-Trip Tests ===== */

void test_encode_decode_vel_ctrl(void) {
    uint8_t frame[PROTO_MAX_FRAME];
    uint8_t frame_len = 0;
    uint8_t seq_out = 0;

    /* Create a CMD_VEL_CTRL payload with 4 floats */
    cmd_vel_ctrl_t vel_cmd = {
        .w1 = 1.5f,
        .w2 = -2.3f,
        .w3 = 0.7f,
        .w4 = -1.2f
    };

    /* Encode */
    int ret = proto_encode(CMD_VEL_CTRL, (const uint8_t *)&vel_cmd, sizeof(vel_cmd),
                           frame, &frame_len, seq_out);
    assert(ret == sizeof(vel_cmd));
    /* Frame should have: SYNC(2) + LEN(1) + SEQ(1) + CMD(1) + PAYLOAD(16) + CRC(2) = 23 bytes */
    assert(frame_len == 7 + sizeof(vel_cmd));
    assert(frame[0] == PROTO_SYNC0);
    assert(frame[1] == PROTO_SYNC1);

    /* Decode */
    uint8_t cmd_out = 0;
    uint8_t payload_out[PROTO_MAX_PAYLOAD];
    uint8_t pay_len_out = 0;
    uint8_t seq_decoded = 0;

    ret = proto_decode(frame, frame_len, &cmd_out, payload_out, &pay_len_out, &seq_decoded);
    assert(ret >= 0);
    assert(cmd_out == CMD_VEL_CTRL);
    assert(pay_len_out == sizeof(vel_cmd));
    assert(seq_decoded == seq_out);

    /* Verify payload matches */
    cmd_vel_ctrl_t *decoded_vel = (cmd_vel_ctrl_t *)payload_out;
    assert(decoded_vel->w1 == vel_cmd.w1);
    assert(decoded_vel->w2 == vel_cmd.w2);
    assert(decoded_vel->w3 == vel_cmd.w3);
    assert(decoded_vel->w4 == vel_cmd.w4);

    printf("PASS: Encode/decode round-trip for CMD_VEL_CTRL (4 floats)\n");
}

void test_encode_decode_odom_feedback(void) {
    uint8_t frame[PROTO_MAX_FRAME];
    uint8_t frame_len = 0;
    uint8_t seq_out = 42;

    /* Create an ODOM_FEEDBACK payload */
    odom_feedback_t odom = {
        .encoder_counts = {1000, -2000, 3000, -4000},
        .tof_distance_mm = 500,
        .imu_q = {0.7071f, 0.0f, 0.0f, 0.7071f},
        .imu_gyro = {0.1f, -0.2f, 0.3f},
        .battery_pct = 85,
        .error_flags = 0x00
    };

    /* Encode */
    int ret = proto_encode(CMD_ODOM_FEEDBACK, (const uint8_t *)&odom, sizeof(odom),
                           frame, &frame_len, seq_out);
    assert(ret == sizeof(odom));
    /* Frame should have: SYNC(2) + LEN(1) + SEQ(1) + CMD(1) + PAYLOAD + CRC(2) = 7 + payload */
    assert(frame_len == 7 + sizeof(odom));

    /* Decode */
    uint8_t cmd_out = 0;
    uint8_t payload_out[PROTO_MAX_PAYLOAD];
    uint8_t pay_len_out = 0;
    uint8_t seq_decoded = 0;

    ret = proto_decode(frame, frame_len, &cmd_out, payload_out, &pay_len_out, &seq_decoded);
    assert(ret >= 0);
    assert(cmd_out == CMD_ODOM_FEEDBACK);
    assert(pay_len_out == sizeof(odom));
    assert(seq_decoded == seq_out);

    /* Verify payload matches */
    odom_feedback_t *decoded_odom = (odom_feedback_t *)payload_out;
    for (int i = 0; i < 4; i++) {
        assert(decoded_odom->encoder_counts[i] == odom.encoder_counts[i]);
    }
    assert(decoded_odom->tof_distance_mm == odom.tof_distance_mm);
    for (int i = 0; i < 4; i++) {
        assert(decoded_odom->imu_q[i] == odom.imu_q[i]);
    }
    for (int i = 0; i < 3; i++) {
        assert(decoded_odom->imu_gyro[i] == odom.imu_gyro[i]);
    }
    assert(decoded_odom->battery_pct == odom.battery_pct);
    assert(decoded_odom->error_flags == odom.error_flags);

    printf("PASS: Encode/decode round-trip for CMD_ODOM_FEEDBACK\n");
}

/* ===== Full-Load Boundary Test ===== */

void test_encode_decode_full_payload(void) {
    uint8_t frame[PROTO_MAX_FRAME];
    uint8_t frame_len = 0;
    uint8_t payload[PROTO_MAX_PAYLOAD];

    for (uint8_t i = 0; i < PROTO_MAX_PAYLOAD; i++) {
        payload[i] = (uint8_t)(i * 3 + 1);
    }

    int ret = proto_encode(0xAB, payload, sizeof(payload), frame, &frame_len, 0x5A);
    assert(ret == PROTO_MAX_PAYLOAD);
    /* SYNC(2) + LEN(1) + SEQ(1) + CMD(1) + PAYLOAD(64) + CRC(2) = 71 bytes */
    assert(frame_len == PROTO_FRAME_OVERHEAD + PROTO_MAX_PAYLOAD);
    assert(frame_len == 71);

    uint8_t cmd_out = 0;
    uint8_t payload_out[PROTO_MAX_PAYLOAD];
    uint8_t pay_len_out = 0;
    uint8_t seq_out = 0;

    ret = proto_decode(frame, frame_len, &cmd_out, payload_out, &pay_len_out, &seq_out);
    assert(ret >= 0);
    assert(cmd_out == 0xAB);
    assert(pay_len_out == PROTO_MAX_PAYLOAD);
    assert(seq_out == 0x5A);
    assert(memcmp(payload, payload_out, PROTO_MAX_PAYLOAD) == 0);

    printf("PASS: Full-load (64-byte) encode/decode round-trip, frame_len=71\n");
}

/* ===== Corruption Tests ===== */

void test_corruption_detection(void) {
    uint8_t frame[PROTO_MAX_FRAME];
    uint8_t frame_len = 0;

    /* Encode a simple payload */
    uint8_t payload[] = {0x42, 0x43, 0x44};
    int ret = proto_encode(0x99, payload, sizeof(payload), frame, &frame_len, 1);
    assert(ret == sizeof(payload));

    /* Corrupt one payload byte */
    frame[5] ^= 0xFF;  /* Flip all bits in first payload byte */

    /* Decode should fail */
    uint8_t cmd_out = 0;
    uint8_t payload_out[PROTO_MAX_PAYLOAD];
    uint8_t pay_len_out = 0;
    uint8_t seq_out = 0;

    ret = proto_decode(frame, frame_len, &cmd_out, payload_out, &pay_len_out, &seq_out);
    assert(ret == -1);  /* CRC mismatch detected */

    printf("PASS: Corruption detection (CRC mismatch returns -1)\n");
}

/* ===== Truncated Frame Tests ===== */

void test_truncated_frame_too_short(void) {
    uint8_t frame[10];
    frame[0] = PROTO_SYNC0;
    frame[1] = PROTO_SYNC1;

    uint8_t cmd_out = 0;
    uint8_t payload_out[PROTO_MAX_PAYLOAD];
    uint8_t pay_len_out = 0;
    uint8_t seq_out = 0;

    /* Frame shorter than PROTO_FRAME_OVERHEAD (6 bytes) */
    int ret = proto_decode(frame, 4, &cmd_out, payload_out, &pay_len_out, &seq_out);
    assert(ret == -1);  /* Should fail gracefully */

    printf("PASS: Truncated frame (< PROTO_FRAME_OVERHEAD) returns -1\n");
}

void test_truncated_frame_payload_incomplete(void) {
    uint8_t frame[PROTO_MAX_FRAME];
    uint8_t frame_len = 0;

    /* Encode a payload of 10 bytes */
    uint8_t payload[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    int ret = proto_encode(0x77, payload, sizeof(payload), frame, &frame_len, 5);
    assert(ret == sizeof(payload));

    /* Try to decode with truncated frame (missing payload bytes) */
    uint8_t cmd_out = 0;
    uint8_t payload_out[PROTO_MAX_PAYLOAD];
    uint8_t pay_len_out = 0;
    uint8_t seq_out = 0;

    ret = proto_decode(frame, PROTO_FRAME_OVERHEAD + 5, &cmd_out, payload_out, &pay_len_out, &seq_out);
    assert(ret == -1);  /* Should fail, frame shorter than advertised */

    printf("PASS: Truncated frame (incomplete payload) returns -1\n");
}

void test_bad_sync_bytes(void) {
    uint8_t frame[PROTO_MAX_FRAME];
    frame[0] = 0xAA;  /* Wrong SYNC0 */
    frame[1] = PROTO_SYNC1;

    uint8_t cmd_out = 0;
    uint8_t payload_out[PROTO_MAX_PAYLOAD];
    uint8_t pay_len_out = 0;
    uint8_t seq_out = 0;

    int ret = proto_decode(frame, PROTO_MAX_FRAME, &cmd_out, payload_out, &pay_len_out, &seq_out);
    assert(ret == -1);

    printf("PASS: Bad sync bytes returns -1\n");
}

/* ===== Sequence Number Preservation ===== */

void test_sequence_numbers(void) {
    for (uint8_t seq = 0; seq < 255; seq++) {
        uint8_t frame[PROTO_MAX_FRAME];
        uint8_t frame_len = 0;
        uint8_t payload[] = {0x55};

        /* Encode */
        int ret = proto_encode(0x88, payload, sizeof(payload), frame, &frame_len, seq);
        assert(ret == sizeof(payload));

        /* Decode */
        uint8_t cmd_out = 0;
        uint8_t payload_out[PROTO_MAX_PAYLOAD];
        uint8_t pay_len_out = 0;
        uint8_t seq_out = 0;

        ret = proto_decode(frame, frame_len, &cmd_out, payload_out, &pay_len_out, &seq_out);
        assert(ret >= 0);
        assert(seq_out == seq);
    }
    printf("PASS: Sequence number preserved for all values 0-254\n");
}

/* ===== Arm protocol tests ===== */

void test_encode_decode_arm_set_pos(void) {
    uint8_t frame[PROTO_MAX_FRAME];
    uint8_t frame_len = 0;
    arm_set_pos_t sp = { { 0.5f, -0.3f, 1.2f, 0.0f, -1.4f, 0.05f } };

    int ret = proto_encode(CMD_ARM_SET_POS, (const uint8_t *)&sp, sizeof(sp),
                           frame, &frame_len, 7);
    assert(ret == sizeof(sp));
    assert(frame_len == 7 + sizeof(sp));

    uint8_t cmd_out, payload_out[PROTO_MAX_PAYLOAD], pay_len_out, seq_out;
    ret = proto_decode(frame, frame_len, &cmd_out, payload_out, &pay_len_out, &seq_out);
    assert(ret >= 0);
    assert(cmd_out == CMD_ARM_SET_POS);
    assert(pay_len_out == sizeof(sp));
    assert(seq_out == 7);

    arm_set_pos_t *dec = (arm_set_pos_t *)payload_out;
    for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
        assert(fabsf(dec->joint[i] - sp.joint[i]) < 1e-6f);
    }
    printf("PASS: Encode/decode round-trip for CMD_ARM_SET_POS (6 floats, radians)\n");
}

void test_encode_decode_arm_torque(void) {
    uint8_t frame[PROTO_MAX_FRAME];
    uint8_t frame_len = 0;
    arm_torque_cmd_t tc = { .mask = 0x3F };

    int ret = proto_encode(CMD_ARM_TORQUE, (const uint8_t *)&tc, sizeof(tc),
                           frame, &frame_len, 0);
    assert(ret == sizeof(tc));

    uint8_t cmd_out, payload_out[PROTO_MAX_PAYLOAD], pay_len_out, seq_out;
    ret = proto_decode(frame, frame_len, &cmd_out, payload_out, &pay_len_out, &seq_out);
    assert(ret >= 0);
    assert(cmd_out == CMD_ARM_TORQUE);
    assert(((arm_torque_cmd_t *)payload_out)->mask == 0x3F);
    printf("PASS: Encode/decode round-trip for CMD_ARM_TORQUE\n");
}

void test_encode_decode_arm_state(void) {
    uint8_t frame[PROTO_MAX_FRAME];
    uint8_t frame_len = 0;
    arm_state_t st;
    memset(&st, 0, sizeof(st));
    st.joint_pos[2] = 1.0f;
    st.joint_speed[1] = -0.5f;
    st.torque_mask = 0x3F;
    st.fault_flags = ARM_FAULT_SOFT_LIMIT;
    st.temperature_c = 42;

    int ret = proto_encode(CMD_ARM_STATE, (const uint8_t *)&st, sizeof(st),
                           frame, &frame_len, 3);
    assert(ret == sizeof(st));
    assert(sizeof(st) <= PROTO_MAX_PAYLOAD);
    assert(frame_len == 7 + sizeof(st));

    uint8_t cmd_out, payload_out[PROTO_MAX_PAYLOAD], pay_len_out, seq_out;
    ret = proto_decode(frame, frame_len, &cmd_out, payload_out, &pay_len_out, &seq_out);
    assert(ret >= 0);
    assert(cmd_out == CMD_ARM_STATE);

    arm_state_t *dec = (arm_state_t *)payload_out;
    assert(fabsf(dec->joint_pos[2] - 1.0f) < 1e-6f);
    assert(fabsf(dec->joint_speed[1] + 0.5f) < 1e-6f);
    assert(dec->torque_mask == 0x3F);
    assert(dec->fault_flags == ARM_FAULT_SOFT_LIMIT);
    assert(dec->temperature_c == 42);
    printf("PASS: Encode/decode round-trip for CMD_ARM_STATE (52-byte feedback)\n");
}

/* ===== Main ===== */

int main(void) {
    printf("Starting protocol.c unit tests...\n\n");

    /* CRC16 tests */
    test_crc16_empty_buffer();
    test_crc16_determinism();
    test_crc16_different_inputs();

    /* Encode/decode tests */
    test_encode_decode_vel_ctrl();
    test_encode_decode_odom_feedback();

    /* Arm protocol tests */
    test_encode_decode_arm_set_pos();
    test_encode_decode_arm_torque();
    test_encode_decode_arm_state();

    /* Full-load boundary test */
    test_encode_decode_full_payload();

    /* Corruption tests */
    test_corruption_detection();

    /* Truncation tests */
    test_truncated_frame_too_short();
    test_truncated_frame_payload_incomplete();
    test_bad_sync_bytes();

    /* Sequence number tests */
    test_sequence_numbers();

    printf("\nAll protocol tests passed!\n");
    return 0;
}
