/**
 * @file arm_control.h
 * @brief Joint command state: accepted targets, torque mask, bus writes.
 *
 * Holds the host-accepted joint targets and the energized-joint mask. Only
 * energized joints are pushed to the servo bus, and only when a target has
 * changed (dirty). Motion interpolation/smoothing is a later addition.
 */

#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include <stdint.h>
#include "protocol.h"   /* ARM_JOINT_COUNT */
#include "servo_bus.h"

typedef struct {
    float   target[ARM_JOINT_COUNT]; /* radians, last accepted from host */
    uint8_t torque_mask;             /* bit0..5 → joint energized */
    uint8_t dirty;                   /* targets changed, bus write pending */
} arm_control_t;

void arm_control_init(arm_control_t *ac);
void arm_control_set_targets(arm_control_t *ac, const float pos[ARM_JOINT_COUNT]);
void arm_control_set_torque(arm_control_t *ac, uint8_t mask);
int  arm_control_tick(arm_control_t *ac); /* push dirty targets to bus; returns failed joint count */
void arm_control_torque_off(arm_control_t *ac);

#endif /* ARM_CONTROL_H */
