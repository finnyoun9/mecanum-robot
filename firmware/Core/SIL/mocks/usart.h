/**
 * @file usart.h — SIL mock of STM32 HAL UART.
 */
#ifndef SIL_USART_H
#define SIL_USART_H

#include <stdint.h>
#include <stdbool.h>
#include "mock_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef mock_uart_t UART_HandleTypeDef;

/* Firmware code uses huart1; map it to the mock instance used by the SIL harness. */
#define huart1 mock_uart1

/* --- API --- */
void HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len);
void HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len);
void HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
void HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);

/* Firmware-defined callbacks (main.c) invoked by the mock HAL. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

/* Core/Src/main.c comm layer (same prototypes as the HW header). */
void comm_create_kernel_objects(void);
uint32_t comm_uart_error_count(void);

#ifdef __cplusplus
}
#endif

#endif /* SIL_USART_H */
