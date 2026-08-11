/**
 * @file tim.h — SIL mock of STM32 HAL TIM.
 */
#ifndef SIL_TIM_H
#define SIL_TIM_H

#include <stdint.h>
#include <stdbool.h>
#include "mock_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef mock_tim_t TIM_HandleTypeDef;

/* Channel identifiers */
#define TIM_CHANNEL_1   0U
#define TIM_CHANNEL_2   1U
#define TIM_CHANNEL_3   2U
#define TIM_CHANNEL_4   3U
#define TIM_CHANNEL_ALL 0x0FU

/* --- API --- */
void HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel);
void HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t channel);

/* Macro-like accessors (used as __HAL_TIM_SET_COMPARE(htim, ch, val)) */
#define __HAL_TIM_SET_COMPARE(htim, channel, value) \
    do { if ((htim) && (channel) < 4) (htim)->ccr[(channel)] = (uint16_t)(value); } while (0)

#define __HAL_TIM_GET_COUNTER(htim) \
    ((htim) ? (htim)->cnt : 0)

#ifdef __cplusplus
}
#endif

#endif /* SIL_TIM_H */
