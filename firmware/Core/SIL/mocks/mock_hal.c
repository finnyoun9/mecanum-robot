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

/* --- Encoder model ---
 * Converts PWM duty into a number of encoder edges for one tick. Returns
 * the signed edge count rather than writing a mock TIM->CNT: the firmware
 * decodes encoders in software now, so SIL feeds these through
 * encoder_on_edge() — the same entry point the EXTI ISR calls on hardware.
 */
int16_t mock_encoder_edges(int motor_idx, int16_t duty,
                           uint32_t dt_ms, uint16_t edges_per_rev) {
    if (motor_idx < 0 || motor_idx >= 4 || edges_per_rev == 0) return 0;

    /* Fractional accumulator so small duty cycles still yield edges
     * across several ticks instead of truncating to zero every time. */
    static float edge_accum[4] = {0.0f};

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
    float dt_s = (float)dt_ms * 0.001f;

    edge_accum[motor_idx] += duty_frac * max_wheel_rps *
                             (float)edges_per_rev * dt_s;

    int16_t delta = (int16_t)edge_accum[motor_idx];
    edge_accum[motor_idx] -= (float)delta;
    return delta;
}
