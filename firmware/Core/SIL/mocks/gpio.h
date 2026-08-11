/**
 * @file gpio.h — SIL mock of STM32 HAL GPIO.
 */
#ifndef SIL_GPIO_H
#define SIL_GPIO_H

#include <stdint.h>
#include "mock_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- GPIO port base addresses (cast to the mock struct via the HAL shim) --- */
typedef mock_gpio_t GPIO_TypeDef;

#define GPIOA  (&mock_gpioa)
#define GPIOB  (&mock_gpiob)
#define GPIOC  (&mock_gpioc)

/* --- GPIO_InitTypeDef (needed by nrf24l01.c to compile) --- */
typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
} GPIO_InitTypeDef;

/* --- Pin masks (0..15) --- */
#define GPIO_PIN_0   ((uint16_t)0x0001)
#define GPIO_PIN_1   ((uint16_t)0x0002)
#define GPIO_PIN_2   ((uint16_t)0x0004)
#define GPIO_PIN_3   ((uint16_t)0x0008)
#define GPIO_PIN_4   ((uint16_t)0x0010)
#define GPIO_PIN_5   ((uint16_t)0x0020)
#define GPIO_PIN_6   ((uint16_t)0x0040)
#define GPIO_PIN_7   ((uint16_t)0x0080)
#define GPIO_PIN_8   ((uint16_t)0x0100)
#define GPIO_PIN_9   ((uint16_t)0x0200)
#define GPIO_PIN_10  ((uint16_t)0x0400)
#define GPIO_PIN_11  ((uint16_t)0x0800)
#define GPIO_PIN_12  ((uint16_t)0x1000)
#define GPIO_PIN_13  ((uint16_t)0x2000)
#define GPIO_PIN_14  ((uint16_t)0x4000)
#define GPIO_PIN_15  ((uint16_t)0x8000)
#define GPIO_PIN_ALL ((uint16_t)0xFFFF)

#define GPIO_PIN_SET   1
#define GPIO_PIN_RESET 0

/* --- GPIO mode / speed / pull constants --- */
#define GPIO_MODE_OUTPUT_PP    1
#define GPIO_MODE_INPUT        0
#define GPIO_SPEED_FREQ_HIGH   3
#define GPIO_PULLUP            1
#define GPIO_PULLDOWN          2
#define GPIO_NOPULL            0

/* --- API --- */
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, uint8_t state);
uint8_t HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);

#ifdef __cplusplus
}
#endif

#endif /* SIL_GPIO_H */
