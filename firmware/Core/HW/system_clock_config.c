/**
 * @file system_clock_config.c
 * @brief SystemClock_Config() for the mecanum-robot chassis MCU.
 *
 * This specific Blue Pill has NO HSE crystal fitted — see
 * dev-environment/bench-win11/README.md ("Blue Pill 开发板：无 HSE 晶振，
 * 必须用 HSI 内部时钟"). The PLL is therefore sourced from HSI, not HSE.
 *
 * HSI = 8 MHz. On F1, the PLL input when sourced from HSI is fixed at
 * HSI/2 = 4 MHz (hardware divider, not configurable). PLLMUL max is x16,
 * giving 4 MHz * 16 = 64 MHz — the highest SYSCLK reachable without an
 * external crystal (vs. 72 MHz max when HSE-sourced).
 *
 * APB1 max is 36 MHz, so it must be divided by 2 from a 64 MHz AHB clock.
 * APB2 can run at the full 64 MHz.
 */
#include "stm32f1xx_hal.h"

void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState       = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState  = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2; /* fixed /2 when HSI drives the PLL */
    osc.PLL.PLLMUL    = RCC_PLL_MUL16;          /* 4 MHz * 16 = 64 MHz */
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        while (1) { /* Clock config failed — nothing to fall back to safely. */ }
    }

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;  /* HCLK = 64 MHz */
    clk.APB1CLKDivider = RCC_HCLK_DIV2;    /* PCLK1 = 32 MHz (max 36 MHz) */
    clk.APB2CLKDivider = RCC_HCLK_DIV1;    /* PCLK2 = 64 MHz */

    /* 64 MHz needs 2 flash wait states (0WS<=24MHz, 1WS<=48MHz, 2WS<=72MHz). */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        while (1) { /* Clock config failed. */ }
    }
}
