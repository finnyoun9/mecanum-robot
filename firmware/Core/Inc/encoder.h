/**
 * @file encoder.h
 * @brief Quadrature encoder counting via software decode of GPIO edges.
 *
 * Not STM32 hardware encoder mode: TIM2/3/4 are all committed to motor PWM
 * and TIM1's encoder inputs are taken by NRF24L01/USART1, so no timer is
 * available. Each encoder instead interrupts on channel A and reads channel
 * B for direction. See docs/wiring.md for the pin and EXTI line allocation.
 *
 * Counts both edges of channel A (2x). Channel B is sampled for direction
 * but never interrupts — a 4x scheme would need EXTI on B as well, and
 * that is impossible without rewiring: EXTI lines are muxed by pin number
 * across all ports, so FR's B (PA7) collides with RL's A (PB7), and PA6
 * with PB6 likewise.
 *
 * Wiring (channel A drives the interrupt, channel B is sampled):
 *   MOTOR_FL: PA0  / PA1   (EXTI0)
 *   MOTOR_FR: PA6  / PA7   (EXTI9_5)
 *   MOTOR_RL: PB7  / PB6   (EXTI9_5)   — PB7 not PB6, avoids clashing with PA6
 *   MOTOR_RR: PB12 / PB13  (EXTI15_10)
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "motor.h"

/**
 * Encoder edges counted per full wheel revolution — MEASURED, not derived.
 *
 * Measured 2026-08-23 by hand-turning the FR wheel exactly 10 revolutions
 * with the bridges disabled (firmware/Core/HW/encoder_count_main.c) and
 * reading the counter: 2240 and 2244 across two runs, i.e. 224.0 and
 * 224.4 edges/rev at 1x — 0.18% apart, the spread expected from eyeballing
 * the start/stop mark by hand. Doubled to 448 when decoding moved to both
 * edges of channel A: every pulse carries exactly one rising and one
 * falling edge, so the factor is exact, and it was confirmed on hardware
 * (counts at a fixed duty came out 2x, not more — which also rules out
 * spurious edges being counted as noise).
 *
 * This supersedes a derived value of CPR(11) * 4 * GEAR_RATIO(34) = 1496,
 * wrong by 6.7x because the gearbox is ~1:20 rather than 1:34 (224/11 at 1x
 * ≈ 20.4) and because software decode does not give the 4x a hardware
 * quadrature peripheral would. Deriving the figure from datasheet-typical
 * constants is what made it wrong; measuring end to end is what makes it
 * right, so keep it measured. Re-measure if the motors, gearboxes or the
 * decode scheme change.
 */
#define EDGES_PER_WHEEL_REV 448

/** Wheel outer diameter in metres (60 mm mecanum wheels). */
#define WHEEL_DIAMETER_M 0.060f

/**
 * @brief Zero all encoder counters.
 *
 * Does not touch any peripheral — GPIO and EXTI setup belongs to the
 * board init code, which is what routes edges here via encoder_on_edge().
 */
void encoder_init(void);

/**
 * @brief Record one encoder edge. Call from the channel-A EXTI handler on
 *        both edges.
 *
 * The single entry point for decoding; keeping it a plain function rather
 * than inlining the logic in an ISR is what lets host tests and SIL drive
 * it directly.
 *
 * Direction is a_level == b_level, which holds on both of A's edges: B's
 * level relative to A inverts between A's rising and falling edge, so
 * comparing the two works where sampling B alone would count one edge up
 * and the next down and cancel out.
 *
 * @param id       Motor whose channel A just changed
 * @param a_level  Channel A's level after the edge
 * @param b_level  Channel B's level at that instant
 */
void encoder_on_edge(motor_id_t id, bool a_level, bool b_level);

/**
 * @brief Cumulative signed edge count since init/reset.
 *
 * Counts up for forward wheel rotation on all four wheels (calibrated in
 * the harness, see docs/wiring.md).
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
