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
#define PROTO_FRAME_OVERHEAD 6  /* SYNC0 + SYNC1 + LEN + SEQ + CMD + CRC16 */
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
