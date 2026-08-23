/**
 * @file encoder_count_main.c
 * @brief Passive encoder counter for measuring edges-per-wheel-revolution
 *        by turning each wheel by hand. Motors are never driven.
 *
 * Purpose: EDGES_PER_WHEEL_REV was originally derived from vendor-typical
 * constants (CPR 11 * 4 * GEAR_RATIO 34 = 1496) and came out 6.7x wrong —
 * the gearbox is ~1:20, and software decode gives 2x, not the 4x a
 * hardware quadrature peripheral would.
 *
 * Measuring the figure directly sidesteps all of that: it is the only
 * quantity encoder_get_speed_rads() actually needs, and a hand-turn
 * measurement inherently captures whatever CPR, gearing and edge-counting
 * scheme are really in play. This target decodes both edges of channel A,
 * matching encoder.c, so what it measures is what the firmware uses.
 *
 * Method: turn one wheel by hand through a whole number of revolutions
 * (10 is a good balance — enough to average out start/stop misalignment,
 * few enough to keep count), then read that wheel's counter and divide.
 * Rotation speed and smoothness are irrelevant; only the total matters.
 * Turn slowly enough that the 150us debounce window can't merge edges.
 *
 * Both TB6612 STBY pins are held low for the whole run, so the bridges
 * are disabled and the wheels stay free to turn by hand.
 *
 * Counter signs follow the calibration in encoder_debug: forward wheel
 * rotation counts positive on all four.
 */
#include "stm32f1xx_hal.h"
#include <stdbool.h>

extern void SystemClock_Config(void);

/* Matches encoder_debug — rejects contact bounce and pickup without
 * dropping genuine edges at hand-turning speeds. */
#define ENCODER_DEBOUNCE_CYCLES  9600U

/* Read these via GDB while the target free-runs:
 *   (gdb) print fl_count
 * Reset a counter between measurements with:
 *   (gdb) set var fl_count = 0
 */
volatile int32_t fl_count = 0;
volatile int32_t fr_count = 0;
volatile int32_t rl_count = 0;
volatile int32_t rr_count = 0;

static volatile uint32_t fl_last_cycle = 0;
static volatile uint32_t fr_last_cycle = 0;
static volatile uint32_t rl_last_cycle = 0;
static volatile uint32_t rr_last_cycle = 0;

static void dwt_cycle_counter_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static bool debounce_ok(volatile uint32_t *last_cycle) {
    uint32_t now = DWT->CYCCNT;
    if ((now - *last_cycle) < ENCODER_DEBOUNCE_CYCLES) {
        return false;
    }
    *last_cycle = now;
    return true;
}


/* Hold both TB6612 STBY pins low so no bridge can drive a motor. */
static void bridges_off(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOC, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
}

/* Same pin/EXTI allocation as encoder_debug — see that file's header for
 * why RL's interrupt pin is PB7 rather than PB6. */
static void encoder_gpio_init(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    switch (GPIO_Pin) {
    case GPIO_PIN_0: /* FL: A = PA0, B = PA1 */
        if (!debounce_ok(&fl_last_cycle)) break;
        if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) ==
            (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET)) fl_count++; else fl_count--;
        break;
    case GPIO_PIN_6: /* FR: A = PA6, B = PA7 */
        if (!debounce_ok(&fr_last_cycle)) break;
        if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) ==
            (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET)) fr_count++; else fr_count--;
        break;
    case GPIO_PIN_7: /* RL: A = PB7, B = PB6 */
        if (!debounce_ok(&rl_last_cycle)) break;
        if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) ==
            (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)) rl_count++; else rl_count--;
        break;
    case GPIO_PIN_12: /* RR: A = PB12, B = PB13 */
        if (!debounce_ok(&rr_last_cycle)) break;
        if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) ==
            (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET)) rr_count++; else rr_count--;
        break;
    default:
        break;
    }
}

void EXTI0_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void EXTI9_5_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
}

void EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    dwt_cycle_counter_init();
    bridges_off();
    encoder_gpio_init();

    for (;;) {
        /* Counting happens entirely in the EXTI handlers. */
    }
}
