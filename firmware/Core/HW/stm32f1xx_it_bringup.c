/**
 * @file stm32f1xx_it_bringup.c
 * @brief Minimal interrupt vector handlers for the bring-up smoke test.
 *
 * No FreeRTOS in this binary, so SysTick drives HAL_GetTick()/HAL_Delay()
 * directly — unlike the full RTOS build, where FreeRTOSConfig.h reroutes
 * SysTick_Handler/PendSV_Handler/SVC_Handler to the FreeRTOS port instead.
 */
#include "stm32f1xx_hal.h"

/* startup_stm32f103xb.s calls __libc_init_array() before main(), which
 * expects these from crti.o/crtn.o — excluded by -nostartfiles. No static
 * C++ constructors in this project, so empty stubs are correct. */
void _init(void) {}
void _fini(void) {}

void NMI_Handler(void) {}

void HardFault_Handler(void) {
    while (1) {}
}

void MemManage_Handler(void) {
    while (1) {}
}

void BusFault_Handler(void) {
    while (1) {}
}

void UsageFault_Handler(void) {
    while (1) {}
}

void SVC_Handler(void) {}

void PendSV_Handler(void) {}

void SysTick_Handler(void) {
    HAL_IncTick();
}
