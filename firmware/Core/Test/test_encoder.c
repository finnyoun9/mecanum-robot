/**
 * @file test_encoder.c
 * @brief Unit tests for encoder.c (software quadrature decode).
 *
 * These cover the decode logic that used to live inside an EXTI ISR and
 * so could only be checked on hardware. Routing every edge through
 * encoder_on_edge() is what makes it reachable here.
 *
 * Build: gcc -std=c11 -I firmware/Core/Inc firmware/Core/Test/test_encoder.c \
 *            firmware/Core/Src/encoder.c -lm -o test_encoder
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "encoder.h"

/* Drive n edges in one direction.
 *
 * Decoding keys off a_level == b_level, and A alternates every edge, so a
 * run of same-direction edges must alternate both levels together rather
 * than hold them fixed — that is what a real quadrature signal does. */
static void feed_edges(motor_id_t id, int n, bool forward) {
    static bool a = false;
    for (int i = 0; i < n; i++) {
        a = !a;
        encoder_on_edge(id, a, forward ? a : !a);
    }
}

/* ===== Counting direction ===== */

static void test_counts_up_and_down(void) {
    encoder_init();

    feed_edges(MOTOR_FL, 100, true);
    assert(encoder_get_count(MOTOR_FL) == 100);

    feed_edges(MOTOR_FL, 40, false);
    assert(encoder_get_count(MOTOR_FL) == 60);

    /* Reversing past zero must go negative, not clamp or wrap. */
    feed_edges(MOTOR_FL, 100, false);
    assert(encoder_get_count(MOTOR_FL) == -40);

    printf("PASS test_counts_up_and_down\n");
}

/* ===== Per-motor isolation ===== */

static void test_motors_are_independent(void) {
    encoder_init();

    feed_edges(MOTOR_FL, 10, true);
    feed_edges(MOTOR_FR, 20, true);
    feed_edges(MOTOR_RL, 30, false);
    /* MOTOR_RR deliberately left untouched. */

    assert(encoder_get_count(MOTOR_FL) == 10);
    assert(encoder_get_count(MOTOR_FR) == 20);
    assert(encoder_get_count(MOTOR_RL) == -30);
    assert(encoder_get_count(MOTOR_RR) == 0);

    printf("PASS test_motors_are_independent\n");
}

/* ===== Out-of-range ids are ignored, not written past the array ===== */

static void test_invalid_id_is_ignored(void) {
    encoder_init();

    encoder_on_edge((motor_id_t)MOTOR_COUNT, true, true);
    encoder_on_edge((motor_id_t)(MOTOR_COUNT + 7), true, true);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        assert(encoder_get_count((motor_id_t)i) == 0);
    }
    assert(encoder_get_count((motor_id_t)MOTOR_COUNT) == 0);

    printf("PASS test_invalid_id_is_ignored\n");
}

/* ===== Reset ===== */

static void test_reset_clears_all(void) {
    encoder_init();

    feed_edges(MOTOR_FL, 55, true);
    feed_edges(MOTOR_RR, 55, false);
    encoder_reset_all();

    for (int i = 0; i < MOTOR_COUNT; i++) {
        assert(encoder_get_count((motor_id_t)i) == 0);
    }

    /* Speed must also restart from the cleared count rather than
     * reporting a huge jump relative to a stale previous sample. */
    float speed = encoder_get_speed_rads(MOTOR_FL, 10);
    assert(fabsf(speed) < 1e-6f);

    printf("PASS test_reset_clears_all\n");
}

/* ===== Speed conversion ===== */

static void test_speed_matches_measured_scaling(void) {
    encoder_init();

    /* Baseline the speed sampler so the first real sample measures only
     * the edges fed after it. */
    (void)encoder_get_speed_rads(MOTOR_FR, 10);

    /* One full wheel revolution in 1000 ms == 2*pi rad/s, using the
     * measured EDGES_PER_WHEEL_REV rather than any derived constant. */
    feed_edges(MOTOR_FR, EDGES_PER_WHEEL_REV, true);
    float speed = encoder_get_speed_rads(MOTOR_FR, 1000);
    assert(fabsf(speed - 6.283185307f) < 1e-3f);

    /* Same edges in a tenth of the time is ten times the speed. */
    feed_edges(MOTOR_FR, EDGES_PER_WHEEL_REV, true);
    speed = encoder_get_speed_rads(MOTOR_FR, 100);
    assert(fabsf(speed - 62.83185307f) < 1e-2f);

    /* Reverse rotation yields negative speed. */
    feed_edges(MOTOR_FR, EDGES_PER_WHEEL_REV, false);
    speed = encoder_get_speed_rads(MOTOR_FR, 1000);
    assert(fabsf(speed + 6.283185307f) < 1e-3f);

    /* No motion means no speed, and the sampler stays put. */
    speed = encoder_get_speed_rads(MOTOR_FR, 1000);
    assert(fabsf(speed) < 1e-6f);

    printf("PASS test_speed_matches_measured_scaling\n");
}

/* A zero interval would divide by zero; it must return 0 and, critically,
 * not consume the pending edges — they belong to the next real sample. */
static void test_zero_interval_is_safe(void) {
    encoder_init();
    (void)encoder_get_speed_rads(MOTOR_RL, 10);

    feed_edges(MOTOR_RL, EDGES_PER_WHEEL_REV, true);

    float speed = encoder_get_speed_rads(MOTOR_RL, 0);
    assert(fabsf(speed) < 1e-6f);

    speed = encoder_get_speed_rads(MOTOR_RL, 1000);
    assert(fabsf(speed - 6.283185307f) < 1e-3f);

    printf("PASS test_zero_interval_is_safe\n");
}

/* Speed is a delta, so a large cumulative count must not inflate it. */
static void test_speed_is_incremental(void) {
    encoder_init();

    feed_edges(MOTOR_RR, 50 * EDGES_PER_WHEEL_REV, true);
    (void)encoder_get_speed_rads(MOTOR_RR, 1000);   /* absorb the backlog */

    feed_edges(MOTOR_RR, EDGES_PER_WHEEL_REV, true);
    float speed = encoder_get_speed_rads(MOTOR_RR, 1000);
    assert(fabsf(speed - 6.283185307f) < 1e-3f);

    printf("PASS test_speed_is_incremental\n");
}

int main(void) {
    printf("=== encoder.c unit tests ===\n");
    test_counts_up_and_down();
    test_motors_are_independent();
    test_invalid_id_is_ignored();
    test_reset_clears_all();
    test_speed_matches_measured_scaling();
    test_zero_interval_is_safe();
    test_speed_is_incremental();
    printf("All encoder tests passed.\n");
    return 0;
}
