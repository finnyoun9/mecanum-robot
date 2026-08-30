/**
 * @file robot_control.c
 * @brief Top-level robot control logic.
 *
 * Runs at 100Hz. Reads encoders, runs 4 PID loops, updates PWM.
 * Odometry published at 50Hz, ToF read at 20Hz.
 */

#include "robot_control.h"
#include <string.h>

/* UART send function — implemented in main.c or comms HAL wrapper */
extern void comm_send_frame(const uint8_t *frame, uint8_t len);
extern uint32_t hal_get_tick_ms(void);

static robot_state_t g_robot;
static float control_target_w[MOTOR_COUNT];

#define TARGET_SLEW_RAD_S_PER_TICK 1.0f
#define STOP_TARGET_RAD_S          0.25f

static float absf(float value) {
    return value < 0.0f ? -value : value;
}

static float slew_towards(float current, float requested) {
    if (requested > current + TARGET_SLEW_RAD_S_PER_TICK) {
        return current + TARGET_SLEW_RAD_S_PER_TICK;
    }
    if (requested < current - TARGET_SLEW_RAD_S_PER_TICK) {
        return current - TARGET_SLEW_RAD_S_PER_TICK;
    }
    return requested;
}

static float speed_feedforward(float speed) {
    /* Inverse of the lifted-chassis battery sweep used by the M3
     * remote_pid_drive target: rad/s = 0.2875*duty% - 0.79. */
    float magnitude = absf(speed);
    if (magnitude < STOP_TARGET_RAD_S) return 0.0f;

    float duty = (magnitude + 0.79f) * 34.78f;
    if (duty > PID_OUT_MAX) duty = PID_OUT_MAX;
    return speed < 0.0f ? -duty : duty;
}

void robot_init(void) {
    memset(&g_robot, 0, sizeof(g_robot));
    memset(control_target_w, 0, sizeof(control_target_w));

    motor_init();
    encoder_init();

    /* Initialise 4 PID controllers with default gains */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        pid_init(&g_robot.pids[i],
                 PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT,
                 PID_OUT_MAX, PID_INTEGRAL_MAX,
                 1.0f / CTRL_LOOP_HZ);
    }

    /* Default IMU quaternion to identity */
    g_robot.imu_q[0] = 1.0f;
    /* motor_init() deliberately leaves the hardware driver stopped. Keep
     * the public state in sync so the first live motion command can take
     * the boot/deadman recovery path instead of having its PWM discarded
     * forever by motor_set_duty(). */
    g_robot.emergency_stop_active = true;
    g_robot.comm_timeout = true; /* Start in timeout until first RX */
    /* The boot-time stop is recoverable by the first live motion command,
     * exactly like a comm-watchdog trip (deadman-switch semantics). */
    g_robot.comm_stop_latched = true;
}

void robot_ctrl_loop(void) {
    /* --- Read sensors --- */
    uint32_t now = hal_get_tick_ms();
    static uint32_t last_ctrl_ms = 0;
    static uint32_t last_odom_ms = 0;
    static uint32_t last_tof_ms  = 0;

    uint32_t dt_ctrl = now - last_ctrl_ms;
    last_ctrl_ms = now;

    /* Clamp the stall accumulator's timebase to one nominal tick. dt_ctrl
     * is unusable for accumulating time: last_ctrl_ms starts at 0, so the
     * first call reports the whole absolute tick count as its delta (which
     * would trip the 500 ms window instantly), and a scheduling hiccup
     * would likewise inflate it. Counting a fixed tick means the window is
     * measured in control iterations, which is what it is really about. */
    uint32_t stall_dt = dt_ctrl > (1000u / CTRL_LOOP_HZ)
                        ? (1000u / CTRL_LOOP_HZ) : dt_ctrl;

    /* --- Run 4 PID loops ---
     * A stop latched purely by the ToF (no K9, no comm-watchdog trip —
     * those still leave comm_stop_latched/emergency_stop_active with no
     * finer-grained reason to distinguish, so a K9 press coinciding with
     * a ToF trip is not separately tracked and would also be released by
     * the auto-clear below; that compound case is accepted as out of
     * scope here) still runs the loop, but every wheel's requested target
     * is clamped to non-positive: reversing away from the obstacle is
     * allowed, driving further into it is not. Positive-duty-convention
     * is forward (see motor.c), so clamping to <=0 rad/s is "not forward"
     * regardless of which wheels a strafe/rotate command touches. */
    bool tof_only_stop = g_robot.emergency_stop_active && g_robot.tof_emergency;
    if (!g_robot.emergency_stop_active || tof_only_stop) {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            /* A wheel latched as stalled stays dead until an explicit
             * clear: re-driving it is what damages the motor. */
            if (g_robot.stalled_mask & (uint8_t)(1u << i)) {
                pid_reset(&g_robot.pids[i]);
                motor_set_duty((motor_id_t)i, 0);
                g_robot.measured_w[i] = 0.0f;
                continue;
            }

            float measured = encoder_get_speed_rads((motor_id_t)i, dt_ctrl);
            g_robot.measured_w[i] = measured;

            float requested = g_robot.target_w[i];
            if (tof_only_stop && requested > 0.0f) {
                requested = 0.0f;
            }
            if (absf(requested) < STOP_TARGET_RAD_S &&
                absf(control_target_w[i]) < STOP_TARGET_RAD_S) {
                /* Preserve the M3 hardware-accepted stop behaviour: once
                 * the ramp reaches centre, coast with PWM=0 and discard
                 * integral residue. Chasing encoder quantisation around
                 * zero produces the audible forward/reverse clicking. */
                control_target_w[i] = 0.0f;
                pid_reset(&g_robot.pids[i]);
                motor_set_duty((motor_id_t)i, 0);
                g_robot.stall_ms[i] = 0;
                continue;
            }

            control_target_w[i] = slew_towards(control_target_w[i], requested);
            pid_setpoint(&g_robot.pids[i], control_target_w[i]);
            float pid_out = speed_feedforward(control_target_w[i]) +
                            pid_update(&g_robot.pids[i], measured);
            if (pid_out > PID_OUT_MAX) pid_out = PID_OUT_MAX;
            if (pid_out < -PID_OUT_MAX) pid_out = -PID_OUT_MAX;
            motor_set_duty((motor_id_t)i, (int16_t)pid_out);

            /* Stall detection: driven hard but not turning. Both tests use
             * magnitude so this works in either direction. */
            float mag_out   = pid_out < 0.0f ? -pid_out : pid_out;
            float mag_speed = measured < 0.0f ? -measured : measured;
            if (mag_out >= STALL_DUTY_MIN && mag_speed < STALL_SPEED_MAX) {
                g_robot.stall_ms[i] += stall_dt;
                if (g_robot.stall_ms[i] >= STALL_TRIP_MS) {
                    g_robot.stalled_mask |= (uint8_t)(1u << i);
                    g_robot.error_flags   = ERR_MOTOR_FAULT;
                    pid_reset(&g_robot.pids[i]);
                    motor_set_duty((motor_id_t)i, 0);
                    g_robot.measured_w[i] = 0.0f;
                }
            } else {
                /* Moving, or not being driven hard: not a stall. */
                g_robot.stall_ms[i] = 0;
            }
        }
    }

    /* --- Communication timeout check --- */
    if ((now - g_robot.last_rx_tick) > COMM_WATCHDOG_MS) {
        if (!g_robot.comm_timeout) {
            g_robot.comm_timeout = true;
            robot_emergency_stop();
            /* robot_emergency_stop() defaults to an explicit, latched
             * stop. Mark this particular stop recoverable only after the
             * motors have actually been stopped. */
            g_robot.comm_stop_latched = true;
        }
    }

    /* --- Odometry publish (50Hz) --- */
    if ((now - last_odom_ms) >= (1000 / ODOM_PUBLISH_HZ)) {
        robot_send_odometry();
        last_odom_ms = now;
    }

    /* --- ToF read & emergency check (20Hz) --- */
    if ((now - last_tof_ms) >= (1000 / TOF_READ_HZ)) {
        /* g_robot.tof_distance_mm = tof_read_mm(); -- hardware-specific */
        if (g_robot.tof_distance_mm < TOF_EMERGENCY_MM &&
            g_robot.tof_distance_mm > 0) {
            g_robot.tof_emergency = true;
            robot_emergency_stop();
        } else if (g_robot.tof_emergency &&
                   g_robot.tof_distance_mm > TOF_EMERGENCY_CLEAR_MM) {
            /* Backed off past the hysteresis gap: release the ToF latch.
             * Only touches state the ToF path itself set — comm_stop_latched
             * is already false from robot_emergency_stop() and stays that
             * way, and a latched motor fault (stalled_mask) is untouched,
             * so a stall discovered while backing away still needs its own
             * explicit robot_clear_motor_fault(). */
            g_robot.tof_emergency = false;
            if (g_robot.stalled_mask == 0) {
                g_robot.emergency_stop_active = false;
            }
        }
        last_tof_ms = now;
    }

    /* --- IMU read (done inside IMU task in full impl) --- */
}

void robot_set_target_wheels(const float w[4]) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        g_robot.target_w[i] = w[i];
    }
    /* Treat the remote as a live command source: refreshing the watchdog
     * tick keeps the 100ms comm timeout from braking mid-remote-drive. */
    g_robot.last_rx_tick = hal_get_tick_ms();
    g_robot.comm_timeout = false;
    /* Match CMD_VEL_CTRL deadman semantics: a valid wireless motion
     * command may release only the boot/watchdog stop. K9, ToF and motor
     * faults leave comm_stop_latched false and remain latched. */
    if (g_robot.emergency_stop_active && g_robot.comm_stop_latched &&
        !g_robot.tof_emergency) {
        robot_emergency_clear();
    }
}

void robot_handle_command(uint8_t cmd, const uint8_t *payload, uint8_t len) {
    g_robot.last_rx_tick = hal_get_tick_ms();
    g_robot.comm_timeout = false;

    switch (cmd) {
    case CMD_VEL_CTRL: {
        if (len < sizeof(cmd_vel_ctrl_t)) break;
        /* Copy the packed wire payload into an aligned local first:
         * casting a uint8_t buffer straight to a packed float struct is
         * undefined behaviour on ARM (unaligned float access traps). */
        cmd_vel_ctrl_t vel;
        memcpy(&vel, payload, sizeof(vel));
        g_robot.target_w[0] = vel.w1;
        g_robot.target_w[1] = vel.w2;
        g_robot.target_w[2] = vel.w3;
        g_robot.target_w[3] = vel.w4;
        /* Deadman-switch recovery: if the stop was latched by the comm
         * watchdog (or boot) and no ToF obstacle is latched, resume on
         * this fresh motion command over the live link. Explicit e-stop
         * and ToF stops keep comm_stop_latched == false and stay latched. */
        if (g_robot.emergency_stop_active && g_robot.comm_stop_latched &&
            !g_robot.tof_emergency) {
            robot_emergency_clear();
            g_robot.comm_stop_latched = false;
        }
        break;
    }
    case CMD_EMERGENCY_STOP:
        robot_emergency_stop();
        break;
    case CMD_CLEAR_MOTOR_FAULT:
        /* Deliberately a distinct command from the deadman-recovery paths:
         * a stalled wheel is a hardware condition, not a comm blip, so it
         * only clears on an explicit request, never implicitly on the
         * next motion packet. */
        robot_clear_motor_fault();
        break;
    case CMD_PID_TUNE: {
        if (len < sizeof(cmd_pid_tune_t)) break;
        cmd_pid_tune_t tune;
        memcpy(&tune, payload, sizeof(tune));
        if (tune.motor_id < MOTOR_COUNT) {
            pid_set_gains(&g_robot.pids[tune.motor_id],
                          tune.kp, tune.ki, tune.kd);
            g_robot.pids[tune.motor_id].integral_max = tune.integral_limit;
        }
        break;
    }
    case CMD_HEARTBEAT:
        /* Send ACK */
        {
            uint8_t ack_frame[PROTO_MAX_FRAME];
            uint8_t ack_len;
            proto_encode(CMD_ACK, NULL, 0, ack_frame, &ack_len, g_robot.comm_seq_tx++);
            comm_send_frame(ack_frame, ack_len);
        }
        break;
    default:
        break;
    }
}

void robot_send_odometry(void) {
    odom_feedback_t odom;
    for (int i = 0; i < 4; i++) {
        odom.encoder_counts[i] = encoder_get_count((motor_id_t)i);
    }
    odom.tof_distance_mm = g_robot.tof_distance_mm;
    memcpy(odom.imu_q, g_robot.imu_q, sizeof(odom.imu_q));
    memcpy(odom.imu_gyro, g_robot.imu_gyro, sizeof(odom.imu_gyro));
    odom.battery_pct  = 0; /* TODO: ADC read */
    odom.error_flags  = g_robot.error_flags;

    uint8_t frame[PROTO_MAX_FRAME];
    uint8_t frame_len;
    proto_encode(CMD_ODOM_FEEDBACK, (const uint8_t *)&odom, sizeof(odom),
                 frame, &frame_len, g_robot.comm_seq_tx++);
    comm_send_frame(frame, frame_len);
}

const robot_state_t* robot_get_state(void) {
    return &g_robot;
}

void robot_update_imu(const float q[4], const float gyro[3]) {
    memcpy(g_robot.imu_q, q, sizeof(g_robot.imu_q));
    memcpy(g_robot.imu_gyro, gyro, sizeof(g_robot.imu_gyro));
}

void robot_update_tof(uint16_t distance_mm, bool valid, bool timed_out) {
    if (timed_out) {
        g_robot.error_flags |= ERR_TOF_TIMEOUT;
        g_robot.error_flags &= (uint8_t)~ERR_TOF_INVALID;
        g_robot.tof_valid = false;
        return; /* keep last known-good distance */
    }
    g_robot.error_flags &= (uint8_t)~ERR_TOF_TIMEOUT;
    g_robot.tof_valid = valid;
    if (valid) {
        g_robot.error_flags &= (uint8_t)~ERR_TOF_INVALID;
        g_robot.tof_distance_mm = distance_mm;
    } else {
        g_robot.error_flags |= ERR_TOF_INVALID;
    }
}

void robot_emergency_stop(void) {
    g_robot.emergency_stop_active = true;
    /* Safe default: stops requested explicitly (K9, UART e-stop or ToF)
     * are not cleared by the next ordinary motion packet. The comm
     * watchdog sets this back to true at its call site. */
    g_robot.comm_stop_latched = false;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        g_robot.target_w[i] = 0.0f;
        control_target_w[i] = 0.0f;
        pid_reset(&g_robot.pids[i]);
        /* Not a stall any more: nothing is being driven. Keep
         * stalled_mask latched — only an explicit clear releases it. */
        g_robot.stall_ms[i] = 0;
    }
    motor_emergency_stop();
}

void robot_emergency_clear(void) {
    motor_resume();
    g_robot.emergency_stop_active = false;
    g_robot.tof_emergency = false;
    g_robot.comm_stop_latched = false;
    /* Preserve a latched motor fault: a stalled wheel is a hardware
     * problem, not a transient condition, and re-driving it is what
     * damages the motor. Clearing it needs robot_clear_motor_fault().
     *
     * Derived from stalled_mask rather than masking error_flags, because
     * the ERR_* values are sequential codes and not disjoint bits
     * (ERR_MOTOR_FAULT 0x03 overlaps 0x01|0x02), so bit arithmetic over
     * them is a trap. stalled_mask is the authoritative latch. */
    g_robot.error_flags = g_robot.stalled_mask ? ERR_MOTOR_FAULT : ERR_NONE;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        g_robot.stall_ms[i] = 0;
    }
}

void robot_clear_motor_fault(void) {
    g_robot.stalled_mask = 0;
    if (g_robot.error_flags == ERR_MOTOR_FAULT) {
        g_robot.error_flags = ERR_NONE;
    }
    for (int i = 0; i < MOTOR_COUNT; i++) {
        g_robot.stall_ms[i] = 0;
        pid_reset(&g_robot.pids[i]);
    }
}
