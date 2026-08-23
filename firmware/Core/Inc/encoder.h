/**
 * @file encoder.h
 * @brief Quadrature encoder reader using STM32 TIM hardware encoder mode.
 *
 * Each motor encoder uses one general-purpose timer in encoder mode (TI1+TI2).
 * Requires 4 independent TIM channels for 4 motors.
 *
 * Typical STM32F4 pin allocation (example — adjust per actual wiring):
 *   MOTOR_FL: TIM2  CH1/CH2  (PA0/PA1)
 *   MOTOR_FR: TIM3  CH1/CH2  (PA6/PA7)  or (PB4/PB5)
 *   MOTOR_RL: TIM4  CH1/CH2  (PB6/PB7)
 *   MOTOR_RR: TIM5  CH1/CH2  (PA0/PA1)  — check availability
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "motor.h"

/**
 * Encoder edges counted per full wheel revolution — MEASURED, not derived.
 *
 * Measured 2026-08-23 by hand-turning the FR wheel exactly 10 revolutions
 * with the bridges disabled (firmware/Core/HW/encoder_count_main.c) and
 * reading the counter: 2240 and 2244 across two runs, i.e. 224.0 and
 * 224.4 edges/rev — 0.18% apart, the spread expected from eyeballing the
 * start/stop mark by hand.
 *
 * This supersedes the previous derived value of CPR(11) * 4 * GEAR_RATIO(34)
 * = 1496, which was wrong by 6.7x on two counts: the gearbox is ~1:20, not
 * 1:34 (224 / 11 ≈ 20.4), and the software EXTI decode in use counts only
 * channel A's rising edge — 1x, not the 4x a hardware quadrature peripheral
 * would give. Deriving this figure from datasheet-typical constants is what
 * made it wrong; measuring the end-to-end number is what makes it right, so
 * keep it measured. Re-measure if the motors, gearboxes, or the decode
 * scheme (e.g. moving to both-edge counting) change.
 */
#define EDGES_PER_WHEEL_REV 224

/** Wheel outer diameter in metres (60 mm mecanum wheels). */
#define WHEEL_DIAMETER_M 0.060f

/**
 * @brief Initialise all 4 encoder TIM peripherals.
 * Configures TIM in encoder mode (TI1+TI2, 4x counting).
 * Call once after HAL_Init().
 */
void encoder_init(void);

/**
 * @brief Read raw timer counter value (cumulative edge count since init).
 * The counter is 16-bit (TIM->CNT); wraps are tracked internally
 * to provide a 32-bit cumulative value.
 *
 * @param id Motor index
 * @return int32_t Cumulative encoder edge count
 */
int32_t encoder_get_count(motor_id_t id);

/**
 * @brief Compute wheel angular velocity from encoder ticks.
 * Call at fixed interval.
 *
 * @param id        Motor index
 * @param delta_ms  Time since last call (milliseconds)
 * @return float    Wheel angular velocity in rad/s
 */
float encoder_get_speed_rads(motor_id_t id, uint32_t delta_ms);

/**
 * @brief Reset cumulative count for all encoders to zero.
 */
void encoder_reset_all(void);

/**
 * @brief Wire an encoder to a TIM handle.
 *
 * In production this is driven by a static pin-mapping table; for SIL
 * testing it lets the test harness connect mock TIM handles at runtime.
 */
void encoder_set_tim(motor_id_t id, void *htim);

#endif /* ENCODER_H */
