/**
 * @file sil_main.c — Software-in-the-Loop entry point.
 *
 * Compiles the full firmware as a Linux binary with mock HAL + FreeRTOS,
 * feeds a VEL_CTRL protocol frame, runs the PID loop, and verifies:
 *   1. PID produces non-zero PWM outputs
 *   2. Encoder model integrates PWM → edge counts → speed feedback
 *   3. Odometry frames are published on the mock UART TX
 *
 * Usage:
 *   ./build/sil_firmware              # interactive (runs forever until Ctrl-C)
 *   ./build/sil_firmware --ci         # exit after N ticks, report PASS/FAIL
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "mock_hal.h"
#include "main.h"       /* HAL_Init, HAL_GetTick */
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "robot_control.h"
#include "protocol.h"

/* --- Forward declarations (defined in main.c, linked in) --- */
extern void sil_uart_rx_feed(const uint8_t *data, uint8_t len);
extern void comm_send_frame(const uint8_t *frame, uint8_t len);
extern uint32_t hal_get_tick_ms(void);

/* Semaphore used by comm_send_frame (normally created in firmware_arch_main). */
extern SemaphoreHandle_t xTxComplete;

/* Task functions (non-static in main.c, available for xTaskCreate). */
extern void CtrlTask(void *), CommTask(void *), SensorTask(void *),
            RemoteTask(void *), MonitorTask(void *);

/* The FreeRTOS scheduler loop checks this flag to exit. */
volatile int sil_exit_flag = 0;

/* ========================================================================
 *  Helpers
 * ======================================================================== */

/** Encode and feed a CMD_VEL_CTRL frame (4×float rad/s) into the firmware. */
static void feed_vel_cmd(float w1, float w2, float w3, float w4) {
    float wheels[4] = {w1, w2, w3, w4};
    uint8_t frame[PROTO_MAX_FRAME];
    uint8_t frm_len;
    static uint8_t seq = 0;

    int ret = proto_encode(CMD_VEL_CTRL, (const uint8_t *)wheels,
                           sizeof(wheels), frame, &frm_len, seq++);
    if (ret >= 0) {
        sil_uart_rx_feed(frame, frm_len);
    }
}

/** Drain the mock UART TX ring and print any odo frames found. */
static int drain_tx_frames(void) {
    uint8_t buf[PROTO_MAX_FRAME];
    uint8_t idx = 0;
    int odo_count = 0;

    /* Reconstruct framed bytes from the mock TX ring.
     * We look for SYNC0+SYNC1 markers to find frame starts. */
    enum { IDLE, GOT_SYNC0, READING } state = IDLE;
    uint8_t exp_len = 0;
    uint8_t pay_idx = 0;

    while (1) {
        uint8_t b;
        if (!mock_uart_tx_byte(&b)) break; /* no more data */

        switch (state) {
        case IDLE:
            if (b == PROTO_SYNC0) {
                buf[0] = b;
                idx = 1;
                state = GOT_SYNC0;
            }
            break;
        case GOT_SYNC0:
            if (b == PROTO_SYNC1) {
                buf[1] = b;
                idx = 2;
                state = READING;
            } else {
                state = IDLE;
            }
            break;
        case READING:
            buf[idx++] = b;
            if (idx >= 5) {
                exp_len = buf[2];
                pay_idx = idx;
            }
            if (pay_idx > 0 && idx >= (uint8_t)(5 + exp_len + 2)) {
                /* Full frame */
                uint8_t cmd, payload[PROTO_MAX_PAYLOAD], pay_len, seq;
                int ret = proto_decode(buf, idx, &cmd, payload, &pay_len, &seq);
                if (ret >= 0 && cmd == CMD_ODOM_FEEDBACK) {
                    odo_count++;
                    if (pay_len >= 40) {
                        odom_feedback_t odom;
                        memcpy(&odom, payload, sizeof(odom));
                        printf("  [SIL] ODO frame seq=%d counts=[%d %d %d %d] "
                               "tof=%d mm\n",
                               seq,
                               odom.encoder_counts[0],
                               odom.encoder_counts[1],
                               odom.encoder_counts[2],
                               odom.encoder_counts[3],
                               odom.tof_distance_mm);
                    }
                }
                state = IDLE;
                pay_idx = 0;
            }
            break;
        }
    }
    return odo_count;
}

/** Read the mock PWM values from the 4 TIM CCR registers. */
static void read_pwm(int16_t out[4]) {
    mock_tim_t *tims[4] = {&mock_tim2, &mock_tim3, &mock_tim4, &mock_tim5};
    /* motor_set_tim uses TIM_CHANNEL_1 for all 4 motors */
    for (int i = 0; i < 4; i++) {
        out[i] = (int16_t)tims[i]->ccr[0]; /* CH1 */
    }
}

/* ========================================================================
 *  Main
 * ======================================================================== */

int main(int argc, char **argv) {
    bool ci_mode = (argc > 1 && strcmp(argv[1], "--ci") == 0);

    printf("=== MCR Firmware SIL Test ===\n");

    /* --- 1. Wire mock peripherals before robot_init() ---
     *     encoder_init() preserves htim pointers set by encoder_set_tim(),
     *     and motor_init() finds the htim handles to start PWM. */
    motor_set_tim(MOTOR_FL, &mock_tim2, &mock_gpioa, GPIO_PIN_4, &mock_gpioa, GPIO_PIN_5, TIM_CHANNEL_1);
    motor_set_tim(MOTOR_FR, &mock_tim3, &mock_gpioa, GPIO_PIN_11, &mock_gpioa, GPIO_PIN_12, TIM_CHANNEL_1);
    motor_set_tim(MOTOR_RL, &mock_tim4, &mock_gpiob, GPIO_PIN_1, &mock_gpiob, GPIO_PIN_15, TIM_CHANNEL_1);
    motor_set_tim(MOTOR_RR, &mock_tim5, &mock_gpioc, GPIO_PIN_13, &mock_gpioc, GPIO_PIN_14, TIM_CHANNEL_1);

    encoder_set_tim(MOTOR_FL, &mock_tim2);
    encoder_set_tim(MOTOR_FR, &mock_tim3);
    encoder_set_tim(MOTOR_RL, &mock_tim4);
    encoder_set_tim(MOTOR_RR, &mock_tim5);

    printf("[SIL] Wired motors FL→TIM2, FR→TIM3, RL→TIM4, RR→TIM5\n");

    /* --- 2. Init robot --- */
    robot_init();
    printf("[SIL] robot_init() done. emergency_stop=%d\n",
           motor_is_stopped());

    /* Emergency stop starts true — clear it so PID can run */
    robot_emergency_clear();
    printf("[SIL] Emergency stop cleared. stopped=%d\n", motor_is_stopped());

    /* --- 3. Feed a CMD_VEL_CTRL: 10 rad/s on all 4 wheels --- */
    printf("[SIL] Sending CMD_VEL_CTRL: w=[10.0 10.0 10.0 10.0] rad/s\n");
    feed_vel_cmd(10.0f, 10.0f, 10.0f, 10.0f);

    /* --- 4. Create tasks (same as firmware_arch_main does) --- */
    /* Create semaphore needed by comm_send_frame */
    xTxComplete = xSemaphoreCreateBinary();
    xSemaphoreGive(xTxComplete);

    xTaskCreate(CtrlTask,    "Ctrl",   512, NULL, 4, NULL);
    xTaskCreate(CommTask,    "Comm",   512, NULL, 3, NULL);
    xTaskCreate(SensorTask,  "Sensor", 256, NULL, 2, NULL);
    xTaskCreate(RemoteTask,  "Remote", 256, NULL, 2, NULL);
    xTaskCreate(MonitorTask, "Monitor",128, NULL, 1, NULL);
    printf("[SIL] Registered %d tasks\n", sil_sched_task_count());

    /* --- 5. Run the cooperative scheduler for N ticks --- */
    /* Re-feed VEL_CTRL every 50ms (50 ticks) to prevent the 100ms comm
     * watchdog from triggering an emergency stop. */
    const int RUN_TICKS = ci_mode ? 500 : 0x7FFFFFFF;
    printf("[SIL] Running scheduler for %d ticks...\n", RUN_TICKS);

    for (int tick = 0; tick < RUN_TICKS; tick++) {
        /* Re-feed velocity command periodically */
        if (tick > 0 && tick % 50 == 0) {
            feed_vel_cmd(10.0f, 10.0f, 10.0f, 10.0f);
        }

        /* Advance mock time and run all tasks cooperatively */
        sil_sched_tick();

        /* --- Encoder model: integrate PWM → encoder counts --- */
        int16_t pwm[4];
        read_pwm(pwm);
        for (int m = 0; m < 4; m++) {
            mock_tim_t *tim = NULL;
            switch (m) {
            case MOTOR_FL: tim = &mock_tim2; break;
            case MOTOR_FR: tim = &mock_tim3; break;
            case MOTOR_RL: tim = &mock_tim4; break;
            case MOTOR_RR: tim = &mock_tim5; break;
            }
            if (tim && tim->pwm_started) {
                mock_encoder_integrate(tim, pwm[m], 1,
                                       EDGES_PER_WHEEL_REV);
            }
        }
    }

    /* --- 5. Verify --- */
    printf("\n=== Verification ===\n");

    int16_t pwm[4];
    read_pwm(pwm);
    printf("PWM outputs: FL=%d FR=%d RL=%d RR=%d\n", pwm[0], pwm[1], pwm[2], pwm[3]);

    const robot_state_t *state = robot_get_state();
    printf("Target w:    [%.2f %.2f %.2f %.2f]\n",
           state->target_w[0], state->target_w[1],
           state->target_w[2], state->target_w[3]);
    printf("Measured w:  [%.3f %.3f %.3f %.3f]\n",
           state->measured_w[0], state->measured_w[1],
           state->measured_w[2], state->measured_w[3]);

    /* PID should have produced non-zero output */
    bool pid_ok = true;
    for (int i = 0; i < 4; i++) {
        if (pwm[i] == 0) {
            printf("FAIL: Motor %d PWM is 0 (PID didn't produce output)\n", i);
            pid_ok = false;
        }
    }

    /* Encoder should have accumulated counts */
    bool enc_ok = true;
    for (int i = 0; i < 4; i++) {
        int32_t cnt = encoder_get_count((motor_id_t)i);
        printf("Encoder[%d] count=%d\n", i, cnt);
        if (cnt == 0) {
            printf("FAIL: Encoder %d count is 0 (no integration)\n", i);
            enc_ok = false;
        }
    }

    /* Odometry frames should have been published */
    int odo_count = drain_tx_frames();
    printf("Odometry frames published: %d\n", odo_count);
    bool odo_ok = (odo_count > 0);

    /* Summary */
    printf("\n=== Result ===\n");
    if (pid_ok && enc_ok && odo_ok) {
        printf("PASS: PID closed loop verified — PWM outputs non-zero, "
               "encoders accumulate, odometry published.\n");
        return 0;
    } else {
        printf("FAIL: pid=%s enc=%s odo=%s\n",
               pid_ok ? "PASS" : "FAIL",
               enc_ok ? "PASS" : "FAIL",
               odo_ok ? "PASS" : "FAIL");
        return 1;
    }
}
