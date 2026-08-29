/**
 * @file protocol.h
 * @brief Shared communication protocol between Raspberry Pi (ROS2) and STM32 (FreeRTOS).
 *
 * Frame format:
 *   [SYNC0 0xA5][SYNC1 0x5A][LENGTH][SEQ][CMD_ID][PAYLOAD...][CRC16]
 *
 * - LENGTH: uint8_t, payload bytes only (excludes sync, length, seq, cmd, crc)
 * - SEQ:    uint8_t, rolling sequence number for loss detection
 * - CRC16:  uint16_t (little-endian), CRC-16/MODBUS over [LENGTH..PAYLOAD_END]
 *
 * All multi-byte values are little-endian.
 */

#ifndef SHARED_PROTOCOL_H
#define SHARED_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Frame constants --- */
#define PROTO_SYNC0      0xA5
#define PROTO_SYNC1      0x5A
#define PROTO_MAX_PAYLOAD 64
#define PROTO_FRAME_OVERHEAD 7  /* SYNC0 + SYNC1 + LEN + SEQ + CMD + CRC16(2 bytes) */
#define PROTO_MAX_FRAME   (PROTO_FRAME_OVERHEAD + PROTO_MAX_PAYLOAD)

/* --- Command IDs (Pi → STM32) --- */
#define CMD_VEL_CTRL      0x10  /* Set 4 wheel target velocities */
#define CMD_EMERGENCY_STOP 0x11 /* Immediate motor stop */
#define CMD_PID_TUNE      0x12  /* Set PID gains for one motor */
#define CMD_HEARTBEAT     0x1F  /* Keep-alive ping */

/* --- Command IDs (STM32 → Pi) --- */
#define CMD_ODOM_FEEDBACK 0x20  /* Encoder counts + ToF + IMU + battery */
#define CMD_ACK           0x2E  /* Acknowledge */
#define CMD_ERROR         0x2F  /* Error report */

/* --- Error codes --- */
#define ERR_NONE          0x00
#define ERR_CRC_MISMATCH  0x01
#define ERR_UNKNOWN_CMD   0x02
#define ERR_MOTOR_FAULT   0x03
#define ERR_TOF_TIMEOUT   0x04
#define ERR_TOF_INVALID   0x08  /* measurement completed but has no valid range */

/* --- Payload structures --- */

/* CMD_VEL_CTRL: 4x float wheel target velocities (rad/s) */
typedef struct __attribute__((packed)) {
    float w1;  /* Front-left  */
    float w2;  /* Front-right */
    float w3;  /* Rear-left   */
    float w4;  /* Rear-right  */
} cmd_vel_ctrl_t;

/* CMD_PID_TUNE: set gains for one motor */
typedef struct __attribute__((packed)) {
    uint8_t motor_id;  /* 0=FL, 1=FR, 2=RL, 3=RR */
    float   kp;
    float   ki;
    float   kd;
    float   integral_limit;
} cmd_pid_tune_t;

/* CMD_ODOM_FEEDBACK: sensor data from STM32 */
typedef struct __attribute__((packed)) {
    int32_t  encoder_counts[4];  /* Cumulative encoder counts */
    uint16_t tof_distance_mm;    /* ToF reading in mm */
    float    imu_q[4];           /* Quaternion (w,x,y,z) */
    float    imu_gyro[3];        /* Angular velocity (x,y,z) rad/s */
    uint8_t  battery_pct;        /* Battery percentage 0-100 */
    uint8_t  error_flags;        /* Bitmask of active errors */
} odom_feedback_t;

/* CMD_ERROR payload */
typedef struct __attribute__((packed)) {
    uint8_t error_code;
    uint8_t reserved;
} error_report_t;

/* ========================================================================
 * Manipulator arm commands (LeArm mobile manipulator)
 *
 * Same framing + CRC16 as the base protocol; joint positions are radians
 * throughout (matches ros2_control/MoveIt2). Conversion to vendor servo
 * units happens inside the arm STM32 controller. Joint index 5 is the
 * gripper.
 *
 * Range: 0x40-0x4F Pi → arm MCU, 0x50-0x5F arm MCU → Pi.
 * ====================================================================== */

#define ARM_JOINT_COUNT 6

/* --- Command IDs (Pi → arm STM32) --- */
#define CMD_ARM_SET_POS   0x40  /* Absolute joint targets (radians) */
#define CMD_ARM_GET_STATE 0x41  /* Request a CMD_ARM_STATE report */
#define CMD_ARM_TORQUE    0x42  /* Energize/de-energize joints by bitmask */
#define CMD_ARM_ESTOP     0x43  /* Immediate torque-off + latch FAULT */
#define CMD_ARM_RESET     0x44  /* Clear latched FAULT, return to IDLE */

/* --- Command IDs (arm STM32 → Pi) --- */
#define CMD_ARM_STATE     0x50  /* Joint positions/speeds + fault flags */

/* --- Arm fault flags (bitmask, latched in FAULT state) --- */
#define ARM_FAULT_NONE          0x00
#define ARM_FAULT_HOST_TIMEOUT  0x01
#define ARM_FAULT_SERVO_COMM    0x02
#define ARM_FAULT_SOFT_LIMIT    0x04
#define ARM_FAULT_OVER_TEMP     0x08
#define ARM_FAULT_UNDER_VOLT    0x10
#define ARM_FAULT_OVERLOAD      0x20
#define ARM_FAULT_ESTOP         0x40
#define ARM_FAULT_WATCHDOG      0x80

/* --- Payload structures --- */

/* CMD_ARM_SET_POS: 6× float absolute joint targets (radians).
 * [5] = gripper. Motion interpolation/smoothing is the MCU's job. */
typedef struct __attribute__((packed)) {
    float joint[ARM_JOINT_COUNT];
} arm_set_pos_t;  /* 24 bytes */

/* CMD_ARM_TORQUE: bitmask of joints to energize. 0 = all torque off. */
typedef struct __attribute__((packed)) {
    uint8_t mask;  /* bit0..5 → joint0..5 */
} arm_torque_cmd_t;  /* 1 byte */

/* CMD_ARM_STATE: full feedback report from the arm MCU. */
typedef struct __attribute__((packed)) {
    float    joint_pos[ARM_JOINT_COUNT];   /* radians */
    float    joint_speed[ARM_JOINT_COUNT]; /* rad/s */
    uint8_t  torque_mask;                  /* joints currently energized */
    uint8_t  fault_flags;                  /* ARM_FAULT_* bitmask */
    int8_t   temperature_c;                /* controller temp, ±127 °C */
    uint8_t  reserved;
} arm_state_t;  /* 52 bytes */

/* --- CRC16 (MODBUS) --- */
uint16_t proto_crc16(const uint8_t *data, uint8_t len);

/* --- Packet encode/decode (returns payload length, or -1 on error) --- */
int proto_encode(uint8_t cmd, const uint8_t *payload, uint8_t pay_len,
                 uint8_t *out_frame, uint8_t *out_len, uint8_t seq);

int proto_decode(const uint8_t *frame, uint8_t frame_len,
                 uint8_t *cmd, uint8_t *payload, uint8_t *pay_len, uint8_t *seq);

#ifdef __cplusplus
}
#endif

#endif /* SHARED_PROTOCOL_H */
