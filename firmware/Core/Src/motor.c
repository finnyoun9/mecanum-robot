/**
 * @file motor.c
 * @brief Motor driver implementation for TB6612FNG via STM32 HAL.
 *
 * Hardware mapping (example — adjust to actual wiring):
 *
 *   Board 1 (FL + FR):
 *     FL: DIR=PA4,  PWM=TIM3_CH1 (PA6)
 *     FR: DIR=PA5,  PWM=TIM3_CH2 (PA7)
 *     STBY: PB0 (shared)
 *
 *   Board 2 (RL + RR):
 *     RL: DIR=PB12, PWM=TIM4_CH1 (PB6)
 *     RR: DIR=PB13, PWM=TIM4_CH2 (PB7)
 *     STBY: PB1 (shared)
 *
 * PWM frequency: 20 kHz (above audible range)
 * PWM resolution: 1000 steps (ARR = 999)
 */

#include "motor.h"
#include <stddef.h>

/* --- Replace with actual HAL includes in real project --- */
/* #include "main.h" */
/* #include "tim.h" */
/* #include "gpio.h" */

/* --- Per-motor pin/binding table --- */
typedef struct {
    GPIO_TypeDef *dir_port;
    uint16_t      dir_pin;
    TIM_HandleTypeDef *htim;
    uint32_t      tim_channel;  /* TIM_CHANNEL_1..4 */
} motor_pin_t;

/*
 * Example pin mapping — pin assignments depend on actual STM32 model.
 * Fill these in from your CubeMX .ioc file.
 */
static motor_pin_t motor_pins[MOTOR_COUNT] = {
    /* FL */ {NULL, 0, NULL, 0},  /* TODO: Wire to TIMx_CHy */
    /* FR */ {NULL, 0, NULL, 0},
    /* RL */ {NULL, 0, NULL, 0},
    /* RR */ {NULL, 0, NULL, 0},
};

static bool emergency_stopped = true;

/* --- Private helpers --- */
static void motor_set_pwm(motor_id_t id, uint16_t duty) {
    motor_pin_t *m = &motor_pins[id];
    if (!m->htim) return;
    /* __HAL_TIM_SET_COMPARE(m->htim, m->tim_channel, duty); */
    (void)duty; /* Suppress unused warning until HAL is wired */
}

static void motor_set_dir(motor_id_t id, bool forward) {
    motor_pin_t *m = &motor_pins[id];
    if (!m->dir_port) return;
    /* HAL_GPIO_WritePin(m->dir_port, m->dir_pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET); */
    (void)forward;
}

void motor_init(void) {
    /* Start all PWMs */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_pin_t *m = &motor_pins[i];
        if (m->htim) {
            /* HAL_TIM_PWM_Start(m->htim, m->tim_channel); */
        }
    }
    emergency_stopped = true;
}

void motor_set_duty(motor_id_t id, int16_t duty) {
    if (id >= MOTOR_COUNT || emergency_stopped) return;

    bool forward = (duty >= 0);
    uint16_t abs_duty = (duty >= 0) ? (uint16_t)duty : (uint16_t)(-duty);
    if (abs_duty > 1000) abs_duty = 1000;

    motor_set_dir(id, forward);
    motor_set_pwm(id, abs_duty);
}

void motor_emergency_stop(void) {
    emergency_stopped = true;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_set_pwm((motor_id_t)i, 0);
    }
}

void motor_resume(void) {
    emergency_stopped = false;
}

bool motor_is_stopped(void) {
    return emergency_stopped;
}
