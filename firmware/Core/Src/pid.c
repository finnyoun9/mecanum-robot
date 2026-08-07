/**
 * @file pid.c
 * @brief PID controller implementation (velocity form).
 */

#include "pid.h"

void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd,
              float out_max, float integral_max, float dt) {
    pid->kp           = kp;
    pid->ki           = ki;
    pid->kd           = kd;
    pid->out_max      = out_max;
    pid->integral_max = integral_max;
    pid->dt           = dt;
    pid->setpoint     = 0.0f;
    pid->integral     = 0.0f;
    pid->prev_error   = 0.0f;
    pid->output       = 0.0f;
    pid->enabled      = true;
}

void pid_setpoint(pid_ctrl_t *pid, float sp) {
    pid->setpoint = sp;
}

void pid_reset(pid_ctrl_t *pid) {
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}

void pid_set_gains(pid_ctrl_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float pid_update(pid_ctrl_t *pid, float measurement) {
    if (!pid->enabled) {
        pid->output = 0.0f;
        return 0.0f;
    }

    float error = pid->setpoint - measurement;

    /* Proportional */
    float p_term = pid->kp * error;

    /* Integral with anti-windup clamping */
    pid->integral += error * pid->dt;
    if (pid->integral > pid->integral_max)  pid->integral = pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float i_term = pid->ki * pid->integral;

    /* Derivative (on measurement to avoid derivative kick) */
    float d_term = pid->kd * (error - pid->prev_error) / pid->dt;
    pid->prev_error = error;

    /* Sum and clamp */
    float out = p_term + i_term + d_term;
    if (out > pid->out_max)  out = pid->out_max;
    if (out < -pid->out_max) out = -pid->out_max;

    pid->output = out;
    return out;
}
