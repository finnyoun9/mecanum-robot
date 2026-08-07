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

void robot_init(void) {
    memset(&g_robot, 0, sizeof(g_robot));

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
    g_robot.comm_timeout = true; /* Start in timeout until first RX */
}

void robot_ctrl_loop(void) {
    /* --- Read sensors --- */
    uint32_t now = hal_get_tick_ms();
    static uint32_t last_ctrl_ms = 0;
    static uint32_t last_odom_ms = 0;
    static uint32_t last_tof_ms  = 0;

    uint32_t dt_ctrl = now - last_ctrl_ms;
    last_ctrl_ms = now;

    /* --- Run 4 PID loops --- */
    if (!g_robot.emergency_stop_active) {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            pid_setpoint(&g_robot.pids[i], g_robot.target_w[i]);
            float measured = encoder_get_speed_rads((motor_id_t)i, dt_ctrl);
            g_robot.measured_w[i] = measured;
            float pid_out = pid_update(&g_robot.pids[i], measured);
            motor_set_duty((motor_id_t)i, (int16_t)pid_out);
        }
    }

    /* --- Communication timeout check --- */
    if ((now - g_robot.last_rx_tick) > COMM_WATCHDOG_MS) {
        if (!g_robot.comm_timeout) {
            g_robot.comm_timeout = true;
            robot_emergency_stop();
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
        }
        last_tof_ms = now;
    }

    /* --- IMU read (done inside IMU task in full impl) --- */
}

void robot_handle_command(uint8_t cmd, const uint8_t *payload, uint8_t len) {
    g_robot.last_rx_tick = hal_get_tick_ms();
    g_robot.comm_timeout = false;

    switch (cmd) {
    case CMD_VEL_CTRL: {
        if (len < sizeof(cmd_vel_ctrl_t)) break;
        const cmd_vel_ctrl_t *vel = (const cmd_vel_ctrl_t *)payload;
        g_robot.target_w[0] = vel->w1;
        g_robot.target_w[1] = vel->w2;
        g_robot.target_w[2] = vel->w3;
        g_robot.target_w[3] = vel->w4;
        break;
    }
    case CMD_EMERGENCY_STOP:
        robot_emergency_stop();
        break;
    case CMD_PID_TUNE: {
        if (len < sizeof(cmd_pid_tune_t)) break;
        const cmd_pid_tune_t *tune = (const cmd_pid_tune_t *)payload;
        if (tune->motor_id < MOTOR_COUNT) {
            pid_set_gains(&g_robot.pids[tune->motor_id],
                          tune->kp, tune->ki, tune->kd);
            g_robot.pids[tune->motor_id].integral_max = tune->integral_limit;
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

void robot_emergency_stop(void) {
    g_robot.emergency_stop_active = true;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        g_robot.target_w[i] = 0.0f;
        pid_reset(&g_robot.pids[i]);
    }
    motor_emergency_stop();
}

void robot_emergency_clear(void) {
    motor_resume();
    g_robot.emergency_stop_active = false;
    g_robot.tof_emergency = false;
    g_robot.error_flags = 0;
}
