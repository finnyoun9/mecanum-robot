/**
 * @file mock_hal.c — SIL mock HAL implementations.
 */
#include "mock_hal.h"
#include <string.h>

/* --- Global instances --- */
mock_gpio_t mock_gpioa = {0};
mock_gpio_t mock_gpiob = {0};
mock_gpio_t mock_gpioc = {0};

mock_tim_t  mock_tim2  = {.arr = 0xFFFF};
mock_tim_t  mock_tim3  = {.arr = 0xFFFF};
mock_tim_t  mock_tim4  = {.arr = 0xFFFF};
mock_tim_t  mock_tim5  = {.arr = 0xFFFF};

mock_uart_t mock_uart1 = {0};
mock_i2c_t  mock_i2c1  = {0};

volatile uint32_t mock_tick_ms = 0;

/* --- UART helpers --- */

void mock_uart_rx_byte(uint8_t b) {
    uint16_t next = (mock_uart1.rx_head + 1) % MOCK_UART_RX_SIZE;
    if (next == mock_uart1.rx_tail) return; /* ring full */
    mock_uart1.rx_buf[mock_uart1.rx_head] = b;
    mock_uart1.rx_head = next;
}

bool mock_uart_tx_byte(uint8_t *b) {
    if (mock_uart1.tx_head == mock_uart1.tx_tail) return false;
    *b = mock_uart1.tx_buf[mock_uart1.tx_tail];
    mock_uart1.tx_tail = (mock_uart1.tx_tail + 1) % MOCK_UART_TX_SIZE;
    return true;
}

/* --- Encoder model --- */
uint16_t mock_encoder_integrate(mock_tim_t *tim, int16_t duty,
                                 uint32_t dt_ms, uint16_t edges_per_rev) {
    if (!tim || !tim->pwm_started || edges_per_rev == 0) return tim ? tim->cnt : 0;

    /* Use a static fractional accumulator so small PWM duty cycles
     * still produce edges over multiple ticks. */
    static float edge_accum[4] = {0.0f};  /* one per TIM2..TIM5 */
    int idx;
    extern mock_tim_t mock_tim2, mock_tim3, mock_tim4, mock_tim5;
    if      (tim == &mock_tim2) idx = 0;
    else if (tim == &mock_tim3) idx = 1;
    else if (tim == &mock_tim4) idx = 2;
    else if (tim == &mock_tim5) idx = 3;
    else return tim->cnt;

    float duty_frac = (float)duty / 1000.0f;
    /* Extrapolated from the measured duty sweep (2026-08-23, chassis
     * lifted, firmware/Core/HW/duty_sweep_main.c): the duty -> speed
     * response is closely linear above the deadband at ~0.986 edges per
     * duty unit per second, which projects to ~4.27 wheel rev/s at full
     * duty. Replaces an earlier 5.0 guess, and before that a 0.18 that was
     * not physical at all (it existed to make counts visible back when
     * EDGES_PER_WHEEL_REV was wrongly 1496).
     *
     * Still an approximation of the real plant: it is linear through the
     * origin, so it models neither the ~5-10% startup deadband nor supply
     * voltage sag. SIL asserts control-path logic — that a duty produces
     * edges and frames get published — never speed accuracy, so that is
     * an acceptable simplification here. */
    float max_wheel_rps = 4.27f;       /* wheel rev/s at full duty */
    float dt_s = dt_ms * 0.001f;
    float edges = duty_frac * max_wheel_rps * (float)edges_per_rev * dt_s;

    edge_accum[idx] += edges;
    int16_t delta = (int16_t)edge_accum[idx];
    if (delta != 0) {
        edge_accum[idx] -= (float)delta;
        tim->cnt = (uint16_t)((uint32_t)tim->cnt + (int32_t)delta);
    }
    return tim->cnt;
}
