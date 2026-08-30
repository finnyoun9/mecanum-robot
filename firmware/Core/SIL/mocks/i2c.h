/**
 * @file i2c.h — SIL mock of STM32 HAL I2C.
 */
#ifndef SIL_I2C_H
#define SIL_I2C_H

#include <stdint.h>
#include "mock_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef mock_i2c_t I2C_HandleTypeDef;

extern I2C_HandleTypeDef hi2c1;

#define I2C_MEMADD_SIZE_8BIT  1
#define HAL_MAX_DELAY         0xFFFFFFFFU

/* The real HAL returns HAL_StatusTypeDef; drivers compare against HAL_OK.
 * Mirror that here so driver code is identical in both builds — the stubs
 * return 0 on success, which is what HAL_OK is on the real HAL too. */
#define HAL_OK    0
#define HAL_ERROR 1

/* --- API --- */
int HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t mem_addr,
                     uint16_t mem_add_size, uint8_t *data, uint16_t size, uint32_t timeout);
int HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t mem_addr,
                      uint16_t mem_add_size, uint8_t *data, uint16_t size, uint32_t timeout);
int HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t dev_addr,
                            uint8_t *data, uint16_t size, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* SIL_I2C_H */
