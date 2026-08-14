/**
 * @file hal_stubs.c — SIL implementations of STM32 HAL functions.
 */
#include "main.h"
#include "mock_hal.h"
#include <string.h>

/* --- Peripherals --- */
/* huart1 and hi2c1 are aliases for the mock instances declared in mock_hal.c.
 * The firmware references huart1/hi2c1; the SIL harness uses mock_uart1/mock_i2c1.
 * They must be the same object. */
I2C_HandleTypeDef  hi2c1  = {0};

/* ========================================================================
 *  GPIO
 * ======================================================================== */

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, uint8_t state) {
    if (!port) return;
    if (state) port->odr |=  pin;
    else       port->odr &= ~pin;
}

uint8_t HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
    if (!port) return 0;
    return (port->idr & pin) ? 1 : 0;
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init) {
    (void)port;
    (void)init;
    /* No-op in SIL — pin config is done via motor_set_tim / encoder_set_tim */
}

/* ========================================================================
 *  TIM
 * ======================================================================== */

void HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel) {
    if (!htim) return;
    htim->pwm_started = true;
    (void)channel;
}

void HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t channel) {
    if (!htim) return;
    htim->encoder_started = true;
    (void)channel;
}

/* ========================================================================
 *  UART
 * ======================================================================== */

void HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len) {
    if (!huart || !data) return;
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (huart->tx_head + 1) % MOCK_UART_TX_SIZE;
        if (next == huart->tx_tail) break; /* ring full */
        huart->tx_buf[huart->tx_head] = data[i];
        huart->tx_head = next;
    }
    huart->dma_tx_active = true;
    /* Simulate instant DMA completion: the firmware's TX-complete callback
     * is the only place the xTxComplete semaphore is given back. */
    HAL_UART_TxCpltCallback(huart);
}

void HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len) {
    if (!huart) return;
    /* In SIL, RX is fed byte-by-byte via mock_uart_rx_byte() from the SIL loop.
     * This call just arms the mock DMA flag. */
    huart->dma_rx_active = true;
    (void)data;
    (void)len;
}

void HAL_UART_AbortReceive(UART_HandleTypeDef *huart) {
    if (!huart) return;
    huart->dma_rx_active = false;
}

void HAL_UART_AbortTransmit(UART_HandleTypeDef *huart) {
    if (!huart) return;
    huart->dma_tx_active = false;
}

/* ========================================================================
 *  I2C
 * ======================================================================== */

int HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t mem_addr,
                     uint16_t mem_add_size, uint8_t *data, uint16_t size, uint32_t timeout) {
    (void)dev_addr;
    (void)timeout;
    if (!hi2c || !data) return -1;
    if ((size_t)mem_addr + size > sizeof(hi2c->mem)) return -1;
    if (mem_add_size != 1) return -1;
    memcpy(data, &hi2c->mem[mem_addr], size);
    return 0;
}

int HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t mem_addr,
                      uint16_t mem_add_size, uint8_t *data, uint16_t size, uint32_t timeout) {
    (void)dev_addr;
    (void)timeout;
    if (!hi2c || !data) return -1;
    if ((size_t)mem_addr + size > sizeof(hi2c->mem)) return -1;
    if (mem_add_size != 1) return -1;
    memcpy(&hi2c->mem[mem_addr], data, size);
    return 0;
}

/* ========================================================================
 *  System
 * ======================================================================== */

void HAL_Init(void) {
    /* No-op in SIL */
}

void SystemClock_Config(void) {
    /* No-op in SIL */
}

uint32_t HAL_GetTick(void) {
    return mock_tick_ms;
}
