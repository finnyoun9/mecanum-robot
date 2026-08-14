/**
 * @file pid.h
 * @brief PID controller for wheel speed regulation.
 *
 * Uses incremental (velocity) form to avoid integral windup on setpoint changes.
 * Anti-windup via integral clamping.
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    /* Gains */
    float kp;
    float ki;
    float kd;

    /* Limits */
    float out_max;       /* Max absolute output */
    float integral_max;  /* Max absolute integral term */

    /* State */
    float setpoint;          /* Target value */
    float integral;          /* Accumulated integral */
    float prev_error;        /* Previous error (kept for diagnostics) */
    float prev_measurement;  /* Previous measurement (for D term) */
    float output;            /* Last computed output */
    bool  prev_meas_valid;   /* prev_measurement holds a real sample */

    /* Config */
    float dt;            /* Sample time in seconds */
    bool  enabled;
} pid_ctrl_t;

/**
 * @brief Initialise PID controller.
 * @param dt Sample time in seconds (e.g. 0.01 for 100Hz)
 */
void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd,
              float out_max, float integral_max, float dt);

/** Set new target */
void pid_setpoint(pid_ctrl_t *pid, float sp);

/** Reset integral and previous error */
void pid_reset(pid_ctrl_t *pid);

/** Set gains at runtime */
void pid_set_gains(pid_ctrl_t *pid, float kp, float ki, float kd);

/** Compute one iteration. Returns output. Call at fixed dt interval. */
float pid_update(pid_ctrl_t *pid, float measurement);

#endif /* PID_H */
