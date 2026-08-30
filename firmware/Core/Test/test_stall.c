/**
 * @file test_stall.c
 * @brief Unit tests for the stall protection in robot_control.c.
 *
 * Why this exists: before 2026-08-29 a wheel that was commanded hard but
 * not turning got no special handling at all. The PI integral wound up to
 * PID_INTEGRAL_MAX and pinned the output at full duty into a stationary
 * motor. On the real chassis that happened twice in one session — once
 * because the RR motor had failed, once because its output wire had come
 * loose — and JGA25-370 stall current with an unfused 3S pack is enough to
 * cook the windings.
 *
 * The numbers asserted here come from the open-loop measurements in
 * docs/project-status.md: a healthy motor turns ~7.5 rad/s at 30% duty,
 * while the failed RR motor managed 0.49 rad/s even at 80%.
 *
 * robot_control.c pulls in motor/encoder/protocol, so this test provides
 * its own stubs for those rather than linking the real drivers: the point
 * is to test the stall state machine, and stubs let a test drive
 * "commanded hard, not turning" directly.
 *
 * Build: see firmware/Core/CMakeLists.txt (target test_stall).
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "robot_control.h"

/* ===== Stubs ===== */

static uint32_t g_tick_ms;
static int16_t  g_duty[MOTOR_COUNT];
static bool     g_motor_stopped;
static float    g_fake_speed[MOTOR_COUNT];

uint32_t hal_get_tick_ms(void) { return g_tick_ms; }

void comm_send_frame(const uint8_t *frame, uint8_t len) {
    (void)frame; (void)len;
}

void motor_init(void) {
    memset(g_duty, 0, sizeof(g_duty));
    /* Match the real motor driver: boot starts emergency-stopped. */
    g_motor_stopped = true;
}
void motor_set_duty(motor_id_t id, int16_t duty) { g_duty[id] = duty; }
void motor_emergency_stop(void) { g_motor_stopped = true; }
void motor_resume(void) { g_motor_stopped = false; }
bool motor_is_stopped(void) { return g_motor_stopped; }

void encoder_init(void) { memset(g_fake_speed, 0, sizeof(g_fake_speed)); }

/* The control loop's only speed source. Returning a fixed value per wheel
 * is what lets a test hold a wheel at "not turning" while it is driven. */
float encoder_get_speed_rads(motor_id_t id, uint32_t delta_ms) {
    (void)delta_ms;
    return g_fake_speed[id];
}

int32_t encoder_get_count(motor_id_t id) { (void)id; return 0; }
void encoder_reset_all(void) {}

/* ===== Helpers ===== */

#define TICK_MS (1000 / CTRL_LOOP_HZ)

/* Fresh robot with the boot e-stop cleared and a live comm link, so the
 * PI loops actually run.
 *
 * Gains are set to the M2-validated Kp=100/Ki=300 that the Pi sends over
 * CMD_PID_TUNE before every drive, not the PID_*_DEFAULT placeholders:
 * with the defaults the output saturates at 260 and the loop behaves
 * nothing like it does on the real chassis. */
static void setup(void) {
    g_tick_ms = 1000;
    g_motor_stopped = false;
    memset(g_duty, 0, sizeof(g_duty));
    memset(g_fake_speed, 0, sizeof(g_fake_speed));
    robot_init();
    robot_emergency_clear();
    for (int i = 0; i < MOTOR_COUNT; i++) {
        cmd_pid_tune_t tune = {(uint8_t)i, 100.0f, 300.0f, 0.0f, 1.0f};
        robot_handle_command(CMD_PID_TUNE, (const uint8_t *)&tune,
                             sizeof(tune));
    }
    /* Keep the comm watchdog fed for the whole test. */
    g_tick_ms += 1;
    robot_handle_command(CMD_HEARTBEAT, NULL, 0);
}

/* Keep the link alive: the comm watchdog would otherwise e-stop at 100ms
 * and mask what we are trying to observe. */
static void run_ms_linked(uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += TICK_MS) {
        g_tick_ms += TICK_MS;
        robot_handle_command(CMD_HEARTBEAT, NULL, 0);
        robot_ctrl_loop();
    }
}

static void set_targets(float w) {
    float t[4] = {w, w, w, w};
    robot_set_target_wheels(t);
}

/* ===== Wireless commands recover only the boot/deadman stop ===== */

static void test_remote_command_recovers_boot_stop(void) {
    g_tick_ms = 1000;
    g_motor_stopped = false;
    memset(g_duty, 0, sizeof(g_duty));
    memset(g_fake_speed, 0, sizeof(g_fake_speed));
    robot_init();

    assert(robot_get_state()->emergency_stop_active == true);
    assert(g_motor_stopped == true);

    set_targets(8.0f);
    assert(robot_get_state()->emergency_stop_active == false);
    assert(g_motor_stopped == false);

    printf("PASS test_remote_command_recovers_boot_stop\n");
}

static void test_remote_command_recovers_watchdog_stop(void) {
    setup();
    set_targets(8.0f);

    g_tick_ms += COMM_WATCHDOG_MS + 50;
    robot_ctrl_loop();
    assert(robot_get_state()->emergency_stop_active == true);
    assert(g_motor_stopped == true);

    set_targets(8.0f);
    assert(robot_get_state()->emergency_stop_active == false);
    assert(g_motor_stopped == false);

    printf("PASS test_remote_command_recovers_watchdog_stop\n");
}

static void test_remote_command_does_not_clear_explicit_estop(void) {
    setup();
    robot_emergency_stop();
    assert(robot_get_state()->emergency_stop_active == true);

    set_targets(8.0f);
    assert(robot_get_state()->emergency_stop_active == true);
    assert(g_motor_stopped == true);

    printf("PASS test_remote_command_does_not_clear_explicit_estop\n");
}

static void test_default_pid_breaks_measured_start_deadzone(void) {
    g_tick_ms = 1000;
    g_motor_stopped = false;
    memset(g_duty, 0, sizeof(g_duty));
    memset(g_fake_speed, 0, sizeof(g_fake_speed));
    robot_init();

    /* 2 rad/s is roughly 10% of the remote's full-forward wheel target.
     * The real drivetrain starts at 10% duty but not at 5%, so the
     * production defaults must produce at least 100/1000 immediately. */
    set_targets(2.0f);
    /* The accepted controller slews 1 rad/s per tick, so the first tick
     * deliberately stays soft and the second crosses the 10% duty start. */
    g_tick_ms += TICK_MS;
    robot_ctrl_loop();
    g_tick_ms += TICK_MS;
    robot_ctrl_loop();
    for (int i = 0; i < MOTOR_COUNT; i++) {
        assert(g_duty[i] >= 100);
    }

    printf("PASS test_default_pid_breaks_measured_start_deadzone\n");
}

static void test_centered_remote_does_not_dither_stopped_wheel(void) {
    setup();
    set_targets(0.0f);

    /* One encoder quantum can alternate sign around rest. A controller
     * that keeps chasing zero turns that into alternating motor clicks. */
    for (int n = 0; n < 8; n++) {
        float quantum = (n & 1) ? -1.4f : 1.4f;
        for (int i = 0; i < MOTOR_COUNT; i++) g_fake_speed[i] = quantum;
        run_ms_linked(TICK_MS);
        for (int i = 0; i < MOTOR_COUNT; i++) assert(g_duty[i] == 0);
    }

    printf("PASS test_centered_remote_does_not_dither_stopped_wheel\n");
}

static void test_remote_cadence_has_watchdog_margin(void) {
    setup();
    set_targets(2.0f);

    /* The handset transmits every 100 ms and the receiver polls every
     * 50 ms. A legal scheduling phase can therefore approach 150 ms. */
    g_tick_ms += 150;
    robot_ctrl_loop();
    assert(robot_get_state()->emergency_stop_active == false);
    assert(g_motor_stopped == false);

    printf("PASS test_remote_cadence_has_watchdog_margin\n");
}

/* ===== A jammed wheel trips, and only that wheel ===== */

static void test_stall_trips_only_the_stalled_wheel(void) {
    setup();

    /* All four commanded; three turn normally, RR does not move at all —
     * this is exactly the failed-motor case measured on 2026-08-29. */
    g_fake_speed[MOTOR_FL] = 7.5f;
    g_fake_speed[MOTOR_FR] = 7.5f;
    g_fake_speed[MOTOR_RL] = 7.5f;
    g_fake_speed[MOTOR_RR] = 0.0f;
    set_targets(8.0f);

    /* Just under the trip window: nothing latched yet. */
    run_ms_linked(STALL_TRIP_MS - 2 * TICK_MS);
    assert(robot_get_state()->error_flags != ERR_MOTOR_FAULT);
    assert(g_duty[MOTOR_RR] != 0);

    /* Past the window: RR is cut, the other three keep driving. */
    run_ms_linked(4 * TICK_MS);
    assert(robot_get_state()->error_flags == ERR_MOTOR_FAULT);
    assert(g_duty[MOTOR_RR] == 0);
    assert(g_duty[MOTOR_FL] != 0);
    assert(g_duty[MOTOR_FR] != 0);
    assert(g_duty[MOTOR_RL] != 0);

    printf("PASS test_stall_trips_only_the_stalled_wheel\n");
}

/* ===== The latch holds while the command persists ===== */

static void test_latched_wheel_stays_dead(void) {
    setup();
    g_fake_speed[MOTOR_RR] = 0.0f;
    set_targets(8.0f);
    run_ms_linked(STALL_TRIP_MS + 4 * TICK_MS);
    assert(g_duty[MOTOR_RR] == 0);

    /* Keep commanding it for a long time: it must stay at zero duty
     * rather than being re-driven. This is the property that protects the
     * motor. */
    run_ms_linked(3000);
    assert(g_duty[MOTOR_RR] == 0);
    assert(robot_get_state()->error_flags == ERR_MOTOR_FAULT);

    printf("PASS test_latched_wheel_stays_dead\n");
}

/* ===== A comm blip must not release the latch ===== */

static void test_deadman_recovery_does_not_release_stall(void) {
    setup();
    g_fake_speed[MOTOR_RR] = 0.0f;
    set_targets(8.0f);
    run_ms_linked(STALL_TRIP_MS + 4 * TICK_MS);
    assert(robot_get_state()->error_flags == ERR_MOTOR_FAULT);

    /* Let the comm watchdog trip, then send a fresh motion command — the
     * deadman path calls robot_emergency_clear() here. Before this was
     * separated out, that re-drove the dead motor a few hundred ms after
     * every comm blip. */
    g_tick_ms += COMM_WATCHDOG_MS + 50;
    robot_ctrl_loop();
    cmd_vel_ctrl_t vel = {8.0f, 8.0f, 8.0f, 8.0f};
    robot_handle_command(CMD_VEL_CTRL, (const uint8_t *)&vel, sizeof(vel));

    run_ms_linked(200);
    assert(g_duty[MOTOR_RR] == 0);
    assert(robot_get_state()->error_flags == ERR_MOTOR_FAULT);

    printf("PASS test_deadman_recovery_does_not_release_stall\n");
}

/* ===== Explicit fault clear releases it ===== */

static void test_explicit_clear_releases_stall(void) {
    setup();
    g_fake_speed[MOTOR_RR] = 0.0f;
    set_targets(8.0f);
    run_ms_linked(STALL_TRIP_MS + 4 * TICK_MS);
    assert(robot_get_state()->error_flags == ERR_MOTOR_FAULT);

    /* Cause dealt with: the wheel turns again once released. */
    robot_clear_motor_fault();
    assert(robot_get_state()->error_flags == ERR_NONE);
    g_fake_speed[MOTOR_RR] = 7.5f;
    run_ms_linked(100);
    assert(g_duty[MOTOR_RR] != 0);

    printf("PASS test_explicit_clear_releases_stall\n");
}

/* ===== Normal startup must not trip ===== */

static void test_startup_transient_does_not_trip(void) {
    setup();

    /* A real wheel accelerating from rest: stationary for the first few
     * ticks while the PI ramps, then moving. If the trip window were too
     * short this would false-positive on every single start. */
    set_targets(8.0f);
    for (int i = 0; i < MOTOR_COUNT; i++) g_fake_speed[i] = 0.0f;
    run_ms_linked(80);
    for (int i = 0; i < MOTOR_COUNT; i++) g_fake_speed[i] = 2.0f;
    run_ms_linked(80);
    for (int i = 0; i < MOTOR_COUNT; i++) g_fake_speed[i] = 7.5f;
    run_ms_linked(500);

    assert(robot_get_state()->error_flags != ERR_MOTOR_FAULT);
    for (int i = 0; i < MOTOR_COUNT; i++) assert(g_duty[i] != 0);

    printf("PASS test_startup_transient_does_not_trip\n");
}

/* ===== A slow-but-turning wheel is not a stall ===== */

static void test_slow_wheel_is_not_a_stall(void) {
    setup();

    /* Above STALL_SPEED_MAX but well below target: a weak-but-alive motor,
     * or a heavily loaded one. Cutting it would be wrong — that is a
     * tuning/mechanical matter, not a stall. */
    for (int i = 0; i < MOTOR_COUNT; i++) g_fake_speed[i] = 1.5f;
    set_targets(8.0f);
    run_ms_linked(2000);

    assert(robot_get_state()->error_flags != ERR_MOTOR_FAULT);
    for (int i = 0; i < MOTOR_COUNT; i++) assert(g_duty[i] != 0);

    printf("PASS test_slow_wheel_is_not_a_stall\n");
}

/* ===== Idle wheels are not stalls ===== */

static void test_zero_command_is_not_a_stall(void) {
    setup();

    /* Not turning because nothing is asking it to. The duty floor is what
     * keeps this from tripping, including inside the measured 5-10% start
     * deadzone where a motor legitimately does not turn. */
    for (int i = 0; i < MOTOR_COUNT; i++) g_fake_speed[i] = 0.0f;
    set_targets(0.0f);
    run_ms_linked(2000);

    assert(robot_get_state()->error_flags != ERR_MOTOR_FAULT);

    printf("PASS test_zero_command_is_not_a_stall\n");
}

/* ===== Reverse direction is detected too ===== */

static void test_stall_detected_in_reverse(void) {
    setup();

    /* Both the duty and speed tests use magnitude, so a wheel jammed
     * while being driven backwards must trip identically. */
    g_fake_speed[MOTOR_RR] = 0.0f;
    set_targets(-8.0f);
    run_ms_linked(STALL_TRIP_MS + 4 * TICK_MS);

    assert(robot_get_state()->error_flags == ERR_MOTOR_FAULT);
    assert(g_duty[MOTOR_RR] == 0);

    printf("PASS test_stall_detected_in_reverse\n");
}

int main(void) {
    test_remote_command_recovers_boot_stop();
    test_remote_command_recovers_watchdog_stop();
    test_remote_command_does_not_clear_explicit_estop();
    test_default_pid_breaks_measured_start_deadzone();
    test_centered_remote_does_not_dither_stopped_wheel();
    test_remote_cadence_has_watchdog_margin();
    test_stall_trips_only_the_stalled_wheel();
    test_latched_wheel_stays_dead();
    test_deadman_recovery_does_not_release_stall();
    test_explicit_clear_releases_stall();
    test_startup_transient_does_not_trip();
    test_slow_wheel_is_not_a_stall();
    test_zero_command_is_not_a_stall();
    test_stall_detected_in_reverse();
    printf("\nAll stall protection tests passed.\n");
    return 0;
}
