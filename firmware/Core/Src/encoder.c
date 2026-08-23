/**
 * @file encoder.c
 * @brief Quadrature encoder counting via software decode of GPIO edges.
 *
 * This file previously used the STM32 timer peripheral's hardware encoder
 * mode. That is not usable on this board: TIM2/3/4 all generate motor PWM,
 * and a timer's counter cannot simultaneously free-run for PWM and be
 * clocked by external quadrature edges. TIM1's encoder inputs are on
 * PA8/PA9, taken by NRF24L01 CE and USART1. With no timer left, all four
 * encoders decode in software — see docs/wiring.md for the EXTI line
 * allocation and firmware/Core/HW/encoder_debug_main.c for the bare-metal
 * verification this was ported from.
 *
 * Counting is driven entirely by encoder_on_edge(), called from the EXTI
 * ISR on hardware (and directly by host tests and SIL). Keeping the decode
 * in one plain function, rather than inline in an ISR, is what makes it
 * testable off-target.
 *
 * Resolution: both edges of channel A (2x). Not 4x — that needs EXTI on
 * channel B too, which the pin assignment cannot support (see encoder.h).
 * At 100 Hz sampling, 1x quantised speed to 2.8 rad/s steps, coarse enough
 * that a 10 rad/s setpoint had under four usable levels and PID tuning was
 * meaningless; 2x halves that. EDGES_PER_WHEEL_REV in encoder.h reflects
 * the 2x scheme.
 */

#include "encoder.h"
#include <string.h>

/* Written by encoder_on_edge() (ISR context on hardware), read by the
 * control loop. int32_t is naturally aligned on Cortex-M3, so a read
 * cannot tear against a concurrent write and no critical section is
 * needed around plain reads of `count`. */
typedef struct {
    volatile int32_t count;       /* cumulative edges, signed by direction */
    int32_t          prev_count;  /* snapshot for speed calc; task context only */
} encoder_ctx_t;

static encoder_ctx_t encoders[MOTOR_COUNT];

void encoder_init(void) {
    memset(encoders, 0, sizeof(encoders));
}

void encoder_on_edge(motor_id_t id, bool a_level, bool b_level) {
    if (id >= MOTOR_COUNT) return;

    /* Called on both of channel A's edges (2x). Comparing A against B is
     * what makes that work: B's level relative to A inverts between A's
     * rising and falling edge, so sampling B alone would count one edge up
     * and the next down, cancelling to zero however far the wheel turned.
     *
     * Which physical direction maps to positive is set by the harness:
     * all four wheels were wired/adjusted so forward travel counts up.
     * See docs/wiring.md, "编码器方向校准结果". */
    if (a_level == b_level) {
        encoders[id].count++;
    } else {
        encoders[id].count--;
    }
}

int32_t encoder_get_count(motor_id_t id) {
    if (id >= MOTOR_COUNT) return 0;
    return encoders[id].count;
}

float encoder_get_speed_rads(motor_id_t id, uint32_t delta_ms) {
    if (id >= MOTOR_COUNT || delta_ms == 0) return 0.0f;

    encoder_ctx_t *e = &encoders[id];
    int32_t count = e->count;                 /* single read; see struct note */
    int32_t delta_edges = count - e->prev_count;
    e->prev_count = count;

    /* edges → wheel radians:
     *   edges / EDGES_PER_WHEEL_REV = wheel revolutions
     *   revolutions * 2π            = radians
     *   / (delta_ms / 1000)         = rad/s
     */
    float revs  = (float)delta_edges / (float)EDGES_PER_WHEEL_REV;
    float rads  = revs * 6.283185307f;
    float speed = rads / ((float)delta_ms * 0.001f);

    return speed;
}

void encoder_reset_all(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        encoders[i].count      = 0;
        encoders[i].prev_count = 0;
    }
}
