/**
 * @file test_pid.c
 * @brief Self-contained unit tests for pid.c (PID controller).
 * Compiles without dependencies: gcc -std=c11 -I firmware/Core/Inc firmware/Core/Test/test_pid.c firmware/Core/Src/pid.c -lm -o test_pid
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include "pid.h"

/* ===== Step Response Test ===== */

void test_step_response(void) {
    pid_ctrl_t pid;
    float dt = 0.01f;  /* 100 Hz */
    float setpoint = 10.0f;

    /* Initialize PID with moderate gains */
    pid_init(&pid, 2.0f, 0.5f, 0.01f, 100.0f, 20.0f, dt);
    pid_setpoint(&pid, setpoint);

    float measurement = 0.0f;
    float tolerance = setpoint * 0.10f;  /* 10% tolerance */
    int max_iterations = 1000;
    int converged_count = 0;

    for (int i = 0; i < max_iterations; i++) {
        /* Compute PID output */
        float output = pid_update(&pid, measurement);

        /* Simulate simple first-order plant: exponential approach */
        measurement += (output - measurement) * 0.1f;

        /* Check convergence */
        if (fabsf(measurement - setpoint) < tolerance) {
            converged_count++;
        } else {
            converged_count = 0;  /* Reset if we drift out */
        }

        /* If we've been within tolerance for 20 consecutive samples, converged */
        if (converged_count >= 20) {
            printf("PASS: Step response converged to %f (setpoint %f, error %f) in %d iterations\n",
                   measurement, setpoint, setpoint - measurement, i);
            return;
        }
    }

    /* Verify steady-state error is reasonable (within 15%) */
    float final_error = fabsf(measurement - setpoint);
    if (final_error < setpoint * 0.15f) {
        printf("PASS: Step response reached steady-state (measurement: %f, setpoint: %f, error: %f)\n",
               measurement, setpoint, final_error);
        return;
    }

    printf("FAIL: Step response did not converge (final measurement: %f, setpoint: %f)\n",
           measurement, setpoint);
    assert(false);
}

/* ===== Anti-Windup Integral Clamping Test ===== */

void test_anti_windup(void) {
    pid_ctrl_t pid;
    float dt = 0.01f;  /* 100 Hz */
    float setpoint = 10000.0f;  /* Unreachable target */

    /* Initialize with integral clamping */
    pid_init(&pid, 0.1f, 0.5f, 0.01f, 10.0f, 5.0f, dt);
    pid_setpoint(&pid, setpoint);

    float measurement = 0.0f;
    int iterations = 1000;

    for (int i = 0; i < iterations; i++) {
        float output = pid_update(&pid, measurement);

        /* Plant barely moves (we're stuck trying to reach 10000) */
        measurement += 0.001f;

        /* Check integral never exceeds limit */
        assert(pid.integral <= pid.integral_max);
        assert(pid.integral >= -pid.integral_max);
    }

    /* Integral should be clamped to max */
    assert(fabs(pid.integral) <= pid.integral_max + 0.01f);  /* Small epsilon for float comparison */
    printf("PASS: Anti-windup maintains integral within bounds (integral: %f, max: %f)\n",
           pid.integral, pid.integral_max);
}

/* ===== Output Clamping Test ===== */

void test_output_clamping(void) {
    pid_ctrl_t pid;
    float dt = 0.01f;

    /* High gains to force large outputs */
    pid_init(&pid, 50.0f, 10.0f, 5.0f, 25.0f, 20.0f, dt);

    float setpoint = 100.0f;
    pid_setpoint(&pid, setpoint);

    float measurement = 0.0f;
    int iterations = 200;

    for (int i = 0; i < iterations; i++) {
        float output = pid_update(&pid, measurement);

        /* Verify output is within limits */
        assert(output <= pid.out_max);
        assert(output >= -pid.out_max);
    }

    printf("PASS: Output clamping keeps output within bounds (max: %f)\n", pid.out_max);
}

/* ===== PID Reset Test ===== */

void test_pid_reset(void) {
    pid_ctrl_t pid;
    pid_init(&pid, 1.0f, 0.1f, 0.05f, 10.0f, 5.0f, 0.01f);
    pid_setpoint(&pid, 10.0f);

    /* Run a few iterations to accumulate state */
    float measurement = 0.0f;
    for (int i = 0; i < 10; i++) {
        pid_update(&pid, measurement);
        measurement += 0.5f;
    }

    /* Verify state is non-zero */
    assert(pid.integral != 0.0f || pid.prev_error != 0.0f);

    /* Reset */
    pid_reset(&pid);

    /* Verify state is zero */
    assert(pid.integral == 0.0f);
    assert(pid.prev_error == 0.0f);
    assert(pid.output == 0.0f);

    printf("PASS: PID reset clears integral, prev_error, and output\n");
}

/* ===== Setpoint Change Test ===== */

void test_setpoint_change(void) {
    pid_ctrl_t pid;
    pid_init(&pid, 1.0f, 0.1f, 0.05f, 100.0f, 10.0f, 0.01f);

    float measurement = 5.0f;

    /* Set initial setpoint */
    pid_setpoint(&pid, 10.0f);
    float output1 = pid_update(&pid, measurement);

    /* Change setpoint */
    pid_setpoint(&pid, 15.0f);
    float output2 = pid_update(&pid, measurement);

    /* Outputs should differ because error changed */
    assert(output1 != output2);

    printf("PASS: Setpoint change produces different control output\n");
}

/* ===== Gain Configuration Test ===== */

void test_pid_set_gains(void) {
    pid_ctrl_t pid;
    pid_init(&pid, 1.0f, 0.1f, 0.05f, 100.0f, 10.0f, 0.01f);
    pid_setpoint(&pid, 10.0f);

    float measurement = 0.0f;
    float output1 = pid_update(&pid, measurement);

    /* Change gains */
    pid_set_gains(&pid, 2.0f, 0.2f, 0.1f);
    assert(pid.kp == 2.0f);
    assert(pid.ki == 0.2f);
    assert(pid.kd == 0.1f);

    /* Reset and run again with new gains */
    pid_reset(&pid);
    measurement = 0.0f;
    float output2 = pid_update(&pid, measurement);

    /* P term should be different due to higher Kp */
    assert(output1 != output2);

    printf("PASS: PID gain configuration works correctly\n");
}

/* ===== Disabled PID Test ===== */

void test_pid_disabled(void) {
    pid_ctrl_t pid;
    pid_init(&pid, 1.0f, 0.1f, 0.05f, 100.0f, 10.0f, 0.01f);
    pid_setpoint(&pid, 100.0f);

    /* Disable the PID */
    pid.enabled = false;

    float measurement = 0.0f;
    float output = pid_update(&pid, measurement);

    /* Should return 0 when disabled */
    assert(output == 0.0f);
    assert(pid.output == 0.0f);

    printf("PASS: Disabled PID returns 0 output\n");
}

/* ===== Error Reduction Over Time ===== */

void test_error_reduction(void) {
    pid_ctrl_t pid;
    float dt = 0.01f;  /* 100 Hz */

    pid_init(&pid, 2.0f, 0.5f, 0.01f, 100.0f, 20.0f, dt);
    pid_setpoint(&pid, 10.0f);

    float measurement = 0.0f;
    float setpoint = 10.0f;
    float initial_error = fabsf(measurement - setpoint);
    float prev_error = initial_error;

    for (int i = 0; i < 200; i++) {
        float output = pid_update(&pid, measurement);
        measurement += (output - measurement) * 0.1f;
        float current_error = fabsf(measurement - setpoint);

        /* Allow for some oscillation but overall error should decrease on average */
        if (current_error > prev_error * 1.5f) {
            /* Too much divergence, fail */
            printf("FAIL: Error increased significantly (prev: %f, current: %f)\n", prev_error, current_error);
            assert(false);
        }
        prev_error = current_error;
    }

    float final_error = fabsf(measurement - setpoint);
    /* Should reduce error by at least 50% from initial */
    assert(final_error < initial_error * 0.5f);
    printf("PASS: Error reduction over time (initial: %f, final: %f)\n", initial_error, final_error);
}

/* ===== Main ===== */

int main(void) {
    printf("Starting pid.c unit tests...\n\n");

    /* Core functionality tests */
    test_step_response();
    test_anti_windup();
    test_output_clamping();
    test_pid_reset();

    /* Configuration tests */
    test_setpoint_change();
    test_pid_set_gains();
    test_pid_disabled();

    /* Error reduction test */
    test_error_reduction();

    printf("\nAll PID tests passed!\n");
    return 0;
}
