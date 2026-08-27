/**
 * @file usart.h — hardware USART1 handle for the RTOS drive target.
 *
 * The SIL build replaces this with SIL/mocks/usart.h; on real hardware
 * this is the single definition point for the UART the Pi link uses.
 */
#ifndef HW_USART_H
#define HW_USART_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern UART_HandleTypeDef huart1;      /* Pi 5 link, 921600 8N1, PA9/PA10 */
extern DMA_HandleTypeDef  hdma_usart1_tx;  /* DMA1 channel 4 */
extern DMA_HandleTypeDef  hdma_usart1_rx;  /* DMA1 channel 5 */

/* Core/Src/main.c comm layer. The kernel objects MUST be created before
 * the UART/DMA interrupts are enabled (Pi may already stream at reset);
 * the error counter is GDB/telemetry evidence of link recovery. */
void comm_create_kernel_objects(void);
uint32_t comm_uart_error_count(void);

#ifdef __cplusplus
}
#endif

#endif /* HW_USART_H */
