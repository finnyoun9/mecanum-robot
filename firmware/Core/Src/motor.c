/**
 * @file motor.c
 * @brief Motor driver implementation for TB6612FNG via STM32 HAL.
 *
 * TB6612 needs AIN1/AIN2 (or BIN1/BIN2) driven complementarily to select
 * forward/reverse — a single DIR pin can only reach brake/coast plus one
 * direction, never both. See docs/wiring.md for the confirmed
 * STM32F103C8T6 pin table (Board 1 = FL+FR, Board 2 = RL+RR).
 *
 * PWM frequency: 20 kHz (above audible range)
 * PWM resolution: 1000 steps (ARR = 999)
 */

#include "motor.h"
#ifdef STM32F103xB
#include "stm32f1xx_hal.h"
#else
#include "tim.h"
#include "gpio.h"
#endif
#include <stddef.h>

/* --- Replace with actual HAL includes in real project --- */
/* #include "main.h" */
/* #include "tim.h" */
/* #include "gpio.h" */

/* --- Per-motor pin/binding table --- */
typedef struct {
    GPIO_TypeDef *dir_port_a;   /* AIN1/BIN1 */
    uint16_t      dir_pin_a;
    GPIO_TypeDef *dir_port_b;   /* AIN2/BIN2 — driven as complement of A */
    uint16_t      dir_pin_b;
    TIM_HandleTypeDef *htim;
    uint32_t      tim_channel;  /* TIM_CHANNEL_1..4 */
} motor_pin_t;

/*
 * Filled in at runtime via motor_set_tim() — see docs/wiring.md for the
 * confirmed STM32F103C8T6 pin assignment.
 */
static motor_pin_t motor_pins[MOTOR_COUNT] = {
    /* FL */ {NULL, 0, NULL, 0, NULL, 0},
    /* FR */ {NULL, 0, NULL, 0, NULL, 0},
    /* RL */ {NULL, 0, NULL, 0, NULL, 0},
    /* RR */ {NULL, 0, NULL, 0, NULL, 0},
};

static bool emergency_stopped = true;

/* --- Private helpers --- */
static void motor_set_pwm(motor_id_t id, uint16_t duty) {
    motor_pin_t *m = &motor_pins[id];
    if (!m->htim) return;
    __HAL_TIM_SET_COMPARE(m->htim, m->tim_channel, duty);
}

static void motor_set_dir(motor_id_t id, bool forward) {
    motor_pin_t *m = &motor_pins[id];
    if (!m->dir_port_a || !m->dir_port_b) return;
    /* AIN1/AIN2 (or BIN1/BIN2) driven complementarily: forward = (H,L),
     * reverse = (L,H). Never (H,H)/(L,L) — that's brake/coast, not a
     * direction. */
    HAL_GPIO_WritePin(m->dir_port_a, m->dir_pin_a, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->dir_port_b, m->dir_pin_b, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void motor_init(void) {
    /* Start all PWMs */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_pin_t *m = &motor_pins[i];
        if (m->htim) {
            HAL_TIM_PWM_Start(m->htim, m->tim_channel);
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

void motor_set_tim(motor_id_t id, void *htim,
                   void *dir_port_a, uint16_t dir_pin_a,
                   void *dir_port_b, uint16_t dir_pin_b,
                   uint32_t tim_ch) {
    if (id >= MOTOR_COUNT) return;
    motor_pins[id].htim        = (TIM_HandleTypeDef *)htim;
    motor_pins[id].dir_port_a  = (GPIO_TypeDef *)dir_port_a;
    motor_pins[id].dir_pin_a   = dir_pin_a;
    motor_pins[id].dir_port_b  = (GPIO_TypeDef *)dir_port_b;
    motor_pins[id].dir_pin_b   = dir_pin_b;
    motor_pins[id].tim_channel = tim_ch;
}
