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

#ifdef __cplusplus
}
#endif

#endif /* SIL_USART_H */
