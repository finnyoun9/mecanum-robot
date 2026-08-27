/**
 * @file robot_control.h
 * @brief Top-level robot controller — ties together PID, motors, encoders, comms.
 */

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "protocol.h"

/* --- System timing --- */
#define CTRL_LOOP_HZ     100   /* Motor PID loop frequency */
#define ODOM_PUBLISH_HZ  50    /* Odometry update frequency */
#define TOF_READ_HZ      20    /* ToF read frequency */
#define COMM_WATCHDOG_MS 100   /* Communication timeout */

/* --- Default PID gains (tuned per motor, these are starting points) ---
 * Speed loop starts as pure PI (Kd = 0) per the closed-loop roadmap:
 * tune Kp, then Ki on the real chassis first, add D only if needed. */
#define PID_KP_DEFAULT   2.5f
#define PID_KI_DEFAULT   0.8f
#define PID_KD_DEFAULT   0.0f
#define PID_OUT_MAX      1000.0f   /* PWM range */
#define PID_INTEGRAL_MAX 300.0f

/* --- ToF emergency threshold --- */
#define TOF_EMERGENCY_MM 100  /* Brake if obstacle < 10 cm */

/* --- Robot state --- */
typedef struct {
    /* 4 independent PID controllers */
    pid_ctrl_t pids[MOTOR_COUNT];

    /* Target wheel velocities (rad/s) from Pi */
    float target_w[MOTOR_COUNT];

    /* Measured wheel velocities (rad/s) */
    float measured_w[MOTOR_COUNT];

    /* ToF */
    uint16_t tof_distance_mm;
    bool     tof_emergency;

    /* IMU */
    float imu_q[4];      /* Quaternion w,x,y,z */
    float imu_gyro[3];   /* rad/s */

    /* Comms */
    uint8_t  comm_seq_tx;
    bool     comm_timeout;
    uint32_t last_rx_tick;
    /* True when the latched emergency stop was caused by the comm
     * watchdog (or by the boot state): a fresh CMD_VEL_CTRL over a live
     * link may clear it. Explicit e-stop and ToF stops leave this false,
     * so they are NOT auto-resumed by motion commands. */
    bool     comm_stop_latched;

    /* Error flags */
    uint8_t error_flags;

    /* Overall state */
    bool emergency_stop_active;
} robot_state_t;

/* --- Public API --- */

/** Initialise all subsystems: motors, encoders, PID, sensors */
void robot_init(void);

/** Main control loop — call at CTRL_LOOP_HZ from FreeRTOS task */
void robot_ctrl_loop(void);

/** Process an incoming command frame from Pi */
void robot_handle_command(uint8_t cmd, const uint8_t *payload, uint8_t len);

/**
 * @brief Set wheel velocity targets from the wireless remote.
 * Also refreshes the comm watchdog tick, so a live remote keeps the
 * watchdog from firing while the Pi is not sending UART commands.
 */
void robot_set_target_wheels(const float w[4]);

/** Prepare and send odometry feedback frame to Pi */
void robot_send_odometry(void);

/** Get robot state pointer (read-only access for other tasks) */
const robot_state_t* robot_get_state(void);

/**
 * @brief Update IMU orientation quaternion + gyro reading.
 * Called from SensorTask after each AHRS update — avoids reaching into
 * robot_control's static state directly.
 */
void robot_update_imu(const float q[4], const float gyro[3]);

/**
 * @brief Update ToF distance reading. Sets/clears ERR_TOF_TIMEOUT in
 * error_flags. On timeout the last known-good distance is kept rather
 * than overwritten, so robot_ctrl_loop's emergency-stop check doesn't see
 * a bogus 0.
 */
void robot_update_tof(uint16_t distance_mm, bool timed_out);

/** Trigger emergency stop from ToF/comm timeout */
void robot_emergency_stop(void);

/** Clear emergency stop */
void robot_emergency_clear(void);

#endif /* ROBOT_CONTROL_H */
