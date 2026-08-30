/**
 * @file test_mecanum_ik.c
 * @brief Host-buildable unit tests for mecanum_ik.c.
 *
 * No test framework — plain main() with assert(). Pure C math, builds and
 * runs on a host machine without any STM32/FreeRTOS deps.
 *
 * Build & run (from this directory):
 *   gcc -Wall -Wextra -std=c11 -I../Inc -lm test_mecanum_ik.c ../Src/mecanum_ik.c -o test_mecanum_ik && ./test_mecanum_ik
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "mecanum_ik.h"

/* Matches mcr.urdf.xacro and the ROS2 hardware interface */
#define R  0.0325f
#define LX 0.0625f
#define LY 0.09f

static const mecanum_ik_config_t cfg = { .wheel_radius = R, .lx = LX, .ly = LY };

static int close(float a, float b, float tol)
{
    return fabsf(a - b) <= tol;
}

static void check_all_close(const float *w, float exp[MECANUM_WHEEL_COUNT], float tol)
{
    for (int i = 0; i < MECANUM_WHEEL_COUNT; i++) {
        assert(close(w[i], exp[i], tol));
    }
}

int main(void)
{
    float w[MECANUM_WHEEL_COUNT];
    float l = LX + LY;

    /* (1) Pure forward: all four wheels turn the same direction/speed. */
    {
        float vx = 0.3f, expected = vx / R;
        mecanum_ik(&cfg, vx, 0.0f, 0.0f, w);
        float exp[4] = { expected, expected, expected, expected };
        check_all_close(w, exp, 1e-4f);
        printf("pure forward: OK (%.3f rad/s each)\n", w[0]);
    }

    /* (2) Pure strafe right (vy positive = left per convention, so use -vy):
     *     FL/RR turn one way, FR/RL the other. */
    {
        float vy = 0.3f, expected = vy / R;
        mecanum_ik(&cfg, 0.0f, vy, 0.0f, w);
        /* w1 = -vy/R, w2 = +vy/R, w3 = +vy/R, w4 = -vy/R */
        assert(close(w[0], -expected, 1e-4f));
        assert(close(w[1],  expected, 1e-4f));
        assert(close(w[2],  expected, 1e-4f));
        assert(close(w[3], -expected, 1e-4f));
        printf("pure strafe: OK\n");
    }

    /* (3) Pure rotation (CCW, omega>0): left wheels (FL/RL) run backwards,
     *     right wheels (FR/RR) forwards, magnitude omega*(lx+ly)/R. */
    {
        float om = 1.0f, expected = om * l / R;
        mecanum_ik(&cfg, 0.0f, 0.0f, om, w);
        assert(close(w[0], -expected, 1e-4f));  /* FL backward */
        assert(close(w[1],  expected, 1e-4f));  /* FR forward */
        assert(close(w[2], -expected, 1e-4f));  /* RL backward */
        assert(close(w[3],  expected, 1e-4f));  /* RR forward */
        printf("pure rotation: OK\n");
    }

    /* (4) Identity: zero twist → zero wheel speeds. */
    {
        mecanum_ik(&cfg, 0.0f, 0.0f, 0.0f, w);
        float exp[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
        check_all_close(w, exp, 1e-6f);
        printf("zero twist: OK\n");
    }

    /* (5) scale_to_limit: below limit unchanged, above limit scaled down
     *     proportionally and worst wheel exactly at limit. */
    {
        float vx = 0.3f, om = 1.5f;  /* corner wheel likely exceeds 10 rad/s */
        mecanum_ik(&cfg, vx, 0.0f, om, w);
        float orig[4];
        for (int i = 0; i < 4; i++) orig[i] = w[i];

        float limit = 10.0f;
        mecanum_ik_scale_to_limit(w, limit);
        float max_w = 0.0f;
        for (int i = 0; i < 4; i++) {
            if (fabsf(w[i]) > max_w) max_w = fabsf(w[i]);
            assert(fabsf(w[i]) <= limit + 1e-4f);
        }
        assert(close(max_w, limit, 1e-4f));
        /* Scaling is uniform: every wheel scaled by the same factor. */
        float ratio = w[0] / orig[0];
        for (int i = 1; i < 4; i++) {
            assert(close(w[i] / orig[i], ratio, 1e-4f));
        }
        printf("scale_to_limit: OK\n");
    }

    printf("ALL MECANUM IK TESTS PASSED\n");
    return 0;
}
