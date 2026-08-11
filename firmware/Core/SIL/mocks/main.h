/**
 * @file main.h — SIL mock of CubeMX-generated main.h.
 *
 * In a real CubeMX project this pulls in all peripheral headers.
 * The SIL version provides the system-level HAL functions.
 */
#ifndef SIL_MAIN_H
#define SIL_MAIN_H

#include <stdint.h>
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- System functions --- */
void HAL_Init(void);
void SystemClock_Config(void);
uint32_t HAL_GetTick(void);

/* --- AFIO / SYSCFG for JTAG remap (used by nrf24l01.c) --- */
#define __HAL_AFIO_REMAP_SWJ_NOJTAG()  do {} while (0)
#define __HAL_RCC_AFIO_CLK_ENABLE()    do {} while (0)

/* --- RCC clock enable macros (no-op in SIL) --- */
#define __HAL_RCC_GPIOA_CLK_ENABLE()   do {} while (0)
#define __HAL_RCC_GPIOB_CLK_ENABLE()   do {} while (0)
#define __HAL_RCC_GPIOC_CLK_ENABLE()   do {} while (0)

#ifdef __cplusplus
}
#endif

#endif /* SIL_MAIN_H */
