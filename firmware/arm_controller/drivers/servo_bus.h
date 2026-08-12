/**
 * @file servo_bus.h
 * @brief Half-duplex smart servo bus driver interface.
 *
 * The vendor frame protocol (LX-16A-class: half-duplex UART, per-servo
 * commands + checksum) is not yet confirmed — see
 * manipulator/docs/decision-log.md. This header pins the interface the
 * controller depends on; the real STM32 driver replaces the SIL mock
 * (SIL/mock_servo_bus.c) once the protocol is known.
 */

#ifndef SERVO_BUS_H
#define SERVO_BUS_H

#include <stdint.h>

int servo_bus_init(void);
int servo_bus_set_target(uint8_t joint, float pos_rad); /* 0 ok, -1 no ack/timeout */

#endif /* SERVO_BUS_H */
