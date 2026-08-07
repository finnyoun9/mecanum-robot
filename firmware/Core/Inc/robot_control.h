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

/* --- Default PID gains (tuned per motor, these are starting points) --- */
#define PID_KP_DEFAULT   2.5f
#define PID_KI_DEFAULT   0.8f
#define PID_KD_DEFAULT   0.05f
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

/** Prepare and send odometry feedback frame to Pi */
void robot_send_odometry(void);

/** Get robot state pointer (read-only access for other tasks) */
const robot_state_t* robot_get_state(void);

/** Trigger emergency stop from ToF/comm timeout */
void robot_emergency_stop(void);

/** Clear emergency stop */
void robot_emergency_clear(void);

#endif /* ROBOT_CONTROL_H */
