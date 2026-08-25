/**
 * @file rr_encoder_probe_main.c
 * @brief Passive right-rear encoder A/B continuity probe.
 *
 * No motor PWM or TB6612 bridge is enabled.  Turn the RR wheel by hand ten
 * full revolutions, then inspect rr_a_edges and rr_b_edges through GDB.
 * With the measured 448 edges/rev 2x decoder, both channels should report
 * about 4480 edges.  This deliberately does not decode direction: it tells
 * whether PB12 and PB13 each receive a complete physical signal.
 */
#include "stm32f1xx_hal.h"

extern void SystemClock_Config(void);

volatile uint32_t rr_a_edges;  /* PB12 */
volatile uint32_t rr_b_edges;  /* PB13 */

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
}

static void encoder_gpio_init(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_12) rr_a_edges++;
    if (GPIO_Pin == GPIO_PIN_13) rr_b_edges++;
}

void EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    bridges_off();
    encoder_gpio_init();
    for (;;) {
    }
}
