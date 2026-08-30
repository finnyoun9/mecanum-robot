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
#define COMM_WATCHDOG_MS 250   /* Handset sends at 100 ms; allow poll jitter */
#define REMOTE_ACTIVE_TIMEOUT_MS 250  /* Same margin: expire remote priority if it goes quiet */

/* --- Default PID gains (tuned per motor, these are starting points) ---
 * Speed loop starts as pure PI (Kd = 0) per the closed-loop roadmap:
 * tune Kp, then Ki on the real chassis first, add D only if needed. */
/* M3 four-wheel handset gains. The measured inverse plant supplies the
 * feed-forward effort in robot_control.c; PI only corrects tracking error.
 * Pi control may replace these through CMD_PID_TUNE. */
#define PID_KP_DEFAULT   15.0f
#define PID_KI_DEFAULT   35.0f
#define PID_KD_DEFAULT   0.0f
#define PID_OUT_MAX      1000.0f   /* PWM range */
#define PID_CORRECTION_MAX 350.0f
#define PID_INTEGRAL_MAX (PID_CORRECTION_MAX / PID_KI_DEFAULT)

/* --- ToF emergency threshold ---
 * CLEAR is deliberately higher than the trip point (hysteresis): without
 * a gap, a target sitting near 100 mm would trip/clear every ~50 ms tick
 * on sensor noise alone. Reversing away from an obstacle is still allowed
 * while latched (see robot_ctrl_loop) — the trip point only blocks
 * driving further into it. */
#define TOF_EMERGENCY_MM       100  /* Brake if obstacle < 10 cm */
#define TOF_EMERGENCY_CLEAR_MM 180  /* Auto-clear once backed off past 18 cm */

/* --- Stall protection ---
 * A wheel commanded hard but not turning is either jammed, disconnected or
 * a dead motor. With no detection the PI integral winds to PID_INTEGRAL_MAX
 * and pins the output at full duty into a stationary motor: JGA25-370 stall
 * current is ~5-8x its no-load draw and the 3S pack has no current limit,
 * which is enough to cook the windings.
 *
 * Thresholds come from the 2026-08-29 open-loop measurements and the duty
 * sweep in docs/hardware-closed-loop-roadmap.md. The duty floor sits above
 * the measured 5-10% start deadzone (5% = 50: no rotation at all; 10% = 100:
 * 0.32 rev/s = 2.0 rad/s), so a healthy motor driven at 150 always turns
 * comfortably faster than STALL_SPEED_MAX. The failed RR motor, by
 * contrast, managed 0.49 rad/s at 80% duty.
 *
 * 150 rather than something higher because the duty a stall can reach
 * depends on the gains in force: with the PID_*_DEFAULT placeholders below
 * the output saturates at Kp*err + Ki*integral_max = 260 and can never
 * exceed it, so a higher floor would silently disable this protection
 * while still pushing 26% duty into a stationary motor. The M2-validated
 * gains the Pi actually sends (Kp=100/Ki=300) saturate at 1000 instead.
 *
 * The 500 ms window is deliberately longer than the worst-case startup
 * transient: a stopped wheel accelerating past 1.0 rad/s takes well under
 * 100 ms at the real gains, so a genuine start never trips this. */
#define STALL_DUTY_MIN     150.0f  /* |output| above this counts as "driven hard" */
#define STALL_SPEED_MAX    1.0f    /* rad/s below this counts as "not turning" */
#define STALL_TRIP_MS      500     /* sustained duration before tripping */

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
    bool     tof_valid;
    bool     tof_emergency;

    /* IMU */
    float imu_q[4];      /* Quaternion w,x,y,z */
    float imu_gyro[3];   /* rad/s */

    /* 3S battery terminal voltage sampled through PA4/ADC1_IN4. */
    uint16_t battery_mv;
    uint8_t  battery_pct;
    bool     battery_valid;

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

    /* Stall detection: per-wheel accumulated time (ms) spent driven hard
     * while not turning. Reset as soon as the wheel moves or the command
     * backs off; trips ERR_MOTOR_FAULT at STALL_TRIP_MS. */
    uint32_t stall_ms[MOTOR_COUNT];
    /* Bitmask of wheels latched as stalled (bit i = motor i). Latched
     * wheels stay at zero duty until robot_emergency_clear(). */
    uint8_t  stalled_mask;

    /* Overall state */
    bool emergency_stop_active;

    /* True while the handset's K1 is toggled on. Gates CMD_VEL_CTRL from
     * the Pi (see robot_handle_command()): the two writers share
     * target_w[] with no other arbitration, and the Pi's periodic
     * keepalive (see mcr_hardware_interface.cpp) sends often enough to
     * intermittently stomp the handset's own commands otherwise. */
    bool remote_active;
    /* Last tick a remote packet was processed, regardless of its K1
     * state. Lets robot_ctrl_loop() expire remote_active on its own
     * (see REMOTE_ACTIVE_TIMEOUT_MS) if the handset goes silent -- powered
     * off or out of range -- without ever toggling K1 off, so the Pi
     * doesn't stay locked out of a handset that isn't there any more. */
    uint32_t remote_last_rx_tick;
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

/**
 * @brief Record whether the handset is actively driving (its K1 state).
 * Call on every processed remote packet, enabled or not, so the flag
 * tracks the handset live rather than latching stale from one toggle.
 * While true, robot_handle_command() ignores CMD_VEL_CTRL's wheel
 * targets (the handset keeps writing them directly via
 * robot_set_target_wheels()) so the Pi can't intermittently overwrite
 * them with its own keepalive.
 */
void robot_set_remote_active(bool active);

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
 * @brief Update ToF status. Invalid samples never overwrite the last valid
 * distance and are explicitly surfaced to the OLED instead of as 000 mm.
 */
void robot_update_tof(uint16_t distance_mm, bool valid, bool timed_out);

/** Update the filtered battery reading shown locally and sent in odometry. */
void robot_update_battery(uint16_t battery_mv, uint8_t battery_pct, bool valid);

/** Trigger emergency stop from ToF/comm timeout */
void robot_emergency_stop(void);

/** Clear emergency stop */
/** Clear a latched emergency stop (ToF / comm watchdog / boot).
 *  Does NOT release a latched motor fault — see robot_clear_motor_fault(). */
void robot_emergency_clear(void);

/** Release wheels latched by stall detection. Deliberately separate from
 *  robot_emergency_clear(): the deadman recovery path calls that on every
 *  fresh motion command, which would otherwise re-drive a dead motor a
 *  few hundred ms after every comm blip. Call this only after the
 *  mechanical or electrical cause has actually been dealt with. */
void robot_clear_motor_fault(void);

#endif /* ROBOT_CONTROL_H */
