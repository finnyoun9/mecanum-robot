/**
 * @file motor.h
 * @brief Motor driver abstraction for TB6612FNG dual H-bridge.
 *
 * Each TB6612 controls 2 motors (2 channels). We use two TB6612 boards
 * to drive 4 motors (FL, FR, RL, RR).
 *
 * Interface per motor:
 *   - DIR pin: GPIO output (0=reverse, 1=forward)
 *   - PWM pin: TIM PWM channel (duty cycle 0-100%)
 *   - (optional) STBY pin: shared standby, pulled high for enable
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MOTOR_FL = 0,  /* Front-left  */
    MOTOR_FR = 1,  /* Front-right */
    MOTOR_RL = 2,  /* Rear-left   */
    MOTOR_RR = 3,  /* Rear-right  */
    MOTOR_COUNT = 4
} motor_id_t;

/**
 * @brief Initialise all motor GPIOs and TIM PWM channels.
 * Call once at startup. Assumes CubeMX has configured the HAL peripherals.
 */
void motor_init(void);

/**
 * @brief Set motor duty cycle.
 * @param id    Motor index
 * @param duty  -1000 to +1000 (negative = reverse)
 *
 * Maps duty to DIR pin + PWM CCR value.
 */
void motor_set_duty(motor_id_t id, int16_t duty);

/** Emergency stop: disable all PWM outputs and set motors to coast/brake. */
void motor_emergency_stop(void);

/** Resume after emergency stop. */
void motor_resume(void);

/** Get whether motors are in emergency-stop state. */
bool motor_is_stopped(void);

/**
 * @brief Wire a motor to a TIM channel + DIR GPIO.
 *
 * In production this is a static pin-mapping table; for SIL testing it lets
 * the test harness connect mock TIM handles at runtime.
 *
 * @param id         Motor index
 * @param htim       TIM handle (mock_tim_t* in SIL, real TIM_HandleTypeDef* on HW)
 * @param dir_port   DIR GPIO port (NULL = mock)
 * @param dir_pin    DIR GPIO pin
 * @param tim_ch     TIM channel (TIM_CHANNEL_1..4)
 */
void motor_set_tim(motor_id_t id, void *htim, void *dir_port,
                   uint16_t dir_pin, uint32_t tim_ch);

#endif /* MOTOR_H */
