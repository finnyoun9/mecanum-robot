/**
 * @file stm32f1xx_hal_conf.h
 * @brief HAL configuration for the mecanum-robot chassis MCU (STM32F103C8T6).
 *
 * Trimmed from ST's stm32f1xx_hal_conf_template.h: only the modules this
 * firmware actually touches are enabled (GPIO, TIM, UART, DMA, I2C — plus
 * RCC/CORTEX/FLASH/PWR which HAL_Init()/SystemClock_Config() need).
 *
 * This board has NO HSE crystal (Blue Pill without the can) — SystemClock_Config()
 * drives the PLL from HSI, not HSE. HSE_VALUE below is unused at runtime but
 * some HAL headers reference the macro at compile time, so it's kept defined.
 */
#ifndef __STM32F1xx_HAL_CONF_H
#define __STM32F1xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ########################## Module Selection ############################## */
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* ########################## Oscillator Values ############################## */
#if !defined(HSE_VALUE)
#define HSE_VALUE 8000000U /* Unused: no HSE crystal on this board, kept for HAL headers */
#endif
#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT 100U
#endif
#if !defined(HSI_VALUE)
#define HSI_VALUE 8000000U /* Actual clock source: internal RC, no external crystal fitted */
#endif
#if !defined(LSI_VALUE)
#define LSI_VALUE 40000U
#endif
#if !defined(LSE_VALUE)
#define LSE_VALUE 32768U
#endif
#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT 5000U
#endif

/* ########################### System Configuration ######################### */
#define VDD_VALUE          3300U
#define TICK_INT_PRIORITY  0x0FU
#define USE_RTOS           0U   /* FreeRTOS is layered on top of HAL, not via CMSIS-RTOS */
#define PREFETCH_ENABLE    1U

#define USE_HAL_I2C_REGISTER_CALLBACKS   0U
#define USE_HAL_TIM_REGISTER_CALLBACKS   0U
#define USE_HAL_UART_REGISTER_CALLBACKS  0U

/* #define USE_FULL_ASSERT 1U */

/* Includes ------------------------------------------------------------------*/
#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32f1xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32f1xx_hal_gpio.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32f1xx_hal_dma.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32f1xx_hal_cortex.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32f1xx_hal_flash.h"
#endif
#ifdef HAL_I2C_MODULE_ENABLED
#include "stm32f1xx_hal_i2c.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32f1xx_hal_pwr.h"
#endif
#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32f1xx_hal_tim.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
#include "stm32f1xx_hal_uart.h"
#endif

/* Exported macro ------------------------------------------------------------*/
#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_HAL_CONF_H */
