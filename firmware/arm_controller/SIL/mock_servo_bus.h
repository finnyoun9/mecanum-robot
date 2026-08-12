/**
 * @file mock_servo_bus.h
 * @brief SIL test hooks for the mock servo bus (not part of the driver API).
 */

#ifndef MOCK_SERVO_BUS_H
#define MOCK_SERVO_BUS_H

#include <stdint.h>

int   mock_bus_write_count(void);
int   mock_bus_last_joint(void);
float mock_bus_last_pos(void);
void  mock_bus_reset(void);
void  mock_bus_fail_next(void); /* next set_target() returns -1 */

#endif /* MOCK_SERVO_BUS_H */
