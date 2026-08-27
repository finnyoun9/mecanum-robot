/**
 * @file stm32f1xx_it_rtos.c
 * @brief Interrupt handlers for the FreeRTOS drive target.
 *
 * SVC_Handler / PendSV_Handler / SysTick_Handler are provided by the
 * ARM_CM3 FreeRTOS port (FreeRTOSConfig.h #defines the port symbols onto
 * the vector-table names), so they must NOT be defined here. Encoder EXTI
 * handlers live in rtos_drive_main.c next to their callback.
 */
#include "stm32f1xx_hal.h"
#include "usart.h"

/* startup_stm32f103xb.s calls __libc_init_array() before main(); with
 * -nostartfiles crti.o/crtn.o are absent, and there are no C++ static
 * constructors — empty stubs are correct. */
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

/* --- Pi link: USART1 + its two DMA channels --- */

void DMA1_Channel4_IRQHandler(void) {  /* USART1_TX */
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

void DMA1_Channel5_IRQHandler(void) {  /* USART1_RX */
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

void USART1_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart1);
}
