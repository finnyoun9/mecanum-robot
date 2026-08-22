/**
 * @file bringup_main.c
 * @brief First real-hardware smoke test for the STM32F103C8T6 chassis MCU.
 *
 * Deliberately minimal and separate from the full FreeRTOS robot firmware:
 * proves the toolchain (compiler/linker/startup/HAL/clock) works on actual
 * silicon before layering the FreeRTOS app + TIM/UART/I2C peripherals on
 * top. Toggles the Blue Pill's onboard LED (PC13) — no external wiring
 * needed to verify this runs.
 *
 * PC13 is reused later as the RR motor's BIN1 direction pin (see
 * docs/wiring.md) — no conflict, this binary is throwaway.
 */
#include "stm32f1xx_hal.h"

extern void SystemClock_Config(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();

    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_13;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);

    for (;;) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(500);
    }
}
