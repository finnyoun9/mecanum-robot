/**
 * @file mock_hal.h — Shared state for the SIL mock HAL layer.
 *
 * Every hardware register that the firmware touches gets a mock variable here.
 * The SIL main loop reads/writes these to close the control loop in software.
 */
#ifndef SIL_MOCK_HAL_H
#define SIL_MOCK_HAL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- GPIO mock (per-port, 16 pins each) --- */
#define MOCK_GPIO_PIN_COUNT 16

typedef struct {
    uint16_t odr;   /* output data register */
    uint16_t idr;   /* input data register  */
} mock_gpio_t;

extern mock_gpio_t mock_gpioa;
extern mock_gpio_t mock_gpiob;
extern mock_gpio_t mock_gpioc;

/* --- TIM mock (encoder + PWM in one struct) --- */
typedef struct {
    uint16_t cnt;        /* counter register (encoder counts) */
    uint16_t ccr[4];     /* capture/compare (PWM duty for CH1-4) */
    uint16_t psc;        /* prescaler */
    uint16_t arr;        /* auto-reload */
    bool     encoder_mode;
    bool     pwm_started;
    bool     encoder_started;
} mock_tim_t;

extern mock_tim_t mock_tim2;
extern mock_tim_t mock_tim3;
extern mock_tim_t mock_tim4;
extern mock_tim_t mock_tim5;

/* --- UART mock (ring buffers) --- */
#define MOCK_UART_RX_SIZE 1024
#define MOCK_UART_TX_SIZE 1024

typedef struct {
    uint8_t  rx_buf[MOCK_UART_RX_SIZE];
    uint16_t rx_head;
    uint16_t rx_tail;
    uint8_t  tx_buf[MOCK_UART_TX_SIZE];
    uint16_t tx_head;
    uint16_t tx_tail;
    bool     dma_tx_active;
    bool     dma_rx_active;
} mock_uart_t;

extern mock_uart_t mock_uart1;

/* --- I2C mock --- */
typedef struct {
    uint8_t mem[256];  /* pretend device register space */
    bool    busy;
} mock_i2c_t;

extern mock_i2c_t mock_i2c1;

/* --- Global tick --- */
extern volatile uint32_t mock_tick_ms;

/* --- Helpers for the SIL main loop --- */

/** Feed a byte into the mock UART RX ring (simulates DMA RX). */
void mock_uart_rx_byte(uint8_t b);

/** Read a byte from the mock UART TX ring. Returns true if data available. */
bool mock_uart_tx_byte(uint8_t *b);

/** Advance the mock encoder by converting a PWM duty cycle to edges.
 *  duty is -1000..+1000; a positive duty drives the counter up.
 *  Returns the raw CNT value after integration. */
uint16_t mock_encoder_integrate(mock_tim_t *tim, int16_t duty,
                                 uint32_t dt_ms, uint16_t edges_per_rev);

#ifdef __cplusplus
}
#endif

#endif /* SIL_MOCK_HAL_H */
