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

/** Encoder counts per revolution (motor shaft, before gearbox) */
#define ENCODER_CPR 11  /* JGA25-370 typical: 11 pulses/rev on Hall sensor */

/** Gear ratio (motor shaft : wheel shaft) */
#define GEAR_RATIO  34  /* JGA25-370: ~34:1  (may vary, measure) */

/** Total edges per wheel revolution: CPR * 4 (quadrature) * GEAR_RATIO */
#define EDGES_PER_WHEEL_REV (ENCODER_CPR * 4 * GEAR_RATIO)

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

#endif /* ENCODER_H */
