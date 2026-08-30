/**
 * @file remote_control.h
 * @brief NRF24L01 remote controller → robot motion (pure C, host-testable).
 *
 * Parses the 江协科技 hand-held controller's packet format and maps the
 * joysticks onto omnidirectional (mecanum) motion:
 *
 *   left stick  = translation  (LV → vx forward/back, LH → -vy; stick left strafes left)
 *   right stick = rotation     (RH → -omega; stick left turns nose left)
 *
 * Uses the same 6-byte protocol as the remote firmware:
 *   [0]=ID(0x00), [1]=LH, [2]=LV, [3]=RH, [4]=RV, [5]=KEY
 * Values are int8_t, -100..100. Send rate is 100 ms (10 Hz).
 *
 * Key mapping:
 *   KEY=1  → toggle remote drive enable/disable
 *   KEY=9  → emergency stop
 *   KEY=10 → clear a latched motor stall fault (see robot_clear_motor_fault())
 */

#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "mecanum_ik.h"
#include "nrf24l01.h"  /* NRF24L01_PACKET_WIDTH */

/* --- Motion limits (tune to the drive train) --- */
#define REMOTE_VX_MAX    0.6f   /* m/s   — max forward/backward speed */
#define REMOTE_VY_MAX    0.6f   /* m/s   — max lateral (strafe) speed */
#define REMOTE_OMEGA_MAX 2.5f   /* rad/s — max rotation rate */

/* --- Key codes in the controller's KEY byte --- */
#define REMOTE_KEY_TOGGLE_ENABLE 1   /* K1 — enable/disable remote drive */
#define REMOTE_KEY_ESTOP         9   /* K9 — emergency stop */
#define REMOTE_KEY_CLEAR_FAULT   10  /* K10 — clear a latched motor stall fault */

/* --- Remote state --- */
typedef struct {
    bool     enabled;    /* K1-toggled: remote drives the motors */
    uint8_t  last_key;   /* Last key code seen (event, cleared by controller) */
} remote_state_t;

/* --- Result of processing one packet --- */
typedef struct {
    float wheel_speed[MECANUM_WHEEL_COUNT];  /* IK output, rad/s */
    uint8_t key;                             /* Key event in this packet (0 = none) */
} remote_result_t;

/**
 * @brief Initialise remote state (drive disabled).
 */
void remote_init(remote_state_t *state);

/**
 * @brief Process one received 32-byte NRF packet.
 *
 * Only packets with ID == 0x00 are treated as remote data. Updates the
 * state (KEY=1 toggles enabled) and, when enabled, computes the 4 wheel
 * speeds from the joysticks.
 *
 * @param packet 32-byte NRF payload
 * @param state  Remote state (persists across calls)
 * @param out    Receives wheel speeds + key event when packet is valid
 * @return true if this was a valid remote packet
 */
bool remote_process(const uint8_t packet[NRF24L01_PACKET_WIDTH],
                    remote_state_t *state, remote_result_t *out);

#endif /* REMOTE_CONTROL_H */
