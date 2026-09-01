/**
 * @file rr_encoder_uart_main.c
 * @brief Passive RR encoder A/B counter that prints edges over USART1.
 *
 * No motor PWM / TB6612 bridge is enabled. Turn the RR wheel by hand and the
 * A/B edge counts stream over USART1 (PA9 TX, PA10 RX, 921600 baud) as:
 *   RA:<rr_a_edges> RB:<rr_b_edges>
 * This avoids needing to halt the MCU (OpenOCD mdw) — just cat the Pi's
 * serial port. PB12 = rr_a_edges, PB13 = rr_b_edges.
 */
#include "stm32f1xx_hal.h"
#include <stdio.h>

extern void SystemClock_Config(void);

#define UART_BAUD 921600U

volatile uint32_t rr_a_edges;
volatile uint32_t rr_b_edges;

static UART_HandleTypeDef huart1;

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

static void uart_init(void) {
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = UART_BAUD;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
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
    char line[48];
    HAL_Init();
    SystemClock_Config();
    bridges_off();
    uart_init();
    encoder_gpio_init();
    for (;;) {
        int n = snprintf(line, sizeof(line), "RA:%lu RB:%lu\n",
                         rr_a_edges, rr_b_edges);
        HAL_UART_Transmit(&huart1, (uint8_t *)line, n, 100U);
        HAL_Delay(500);
    }
}
