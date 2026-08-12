#include "arm_control.h"
#include <string.h>

void arm_control_init(arm_control_t *ac) {
    memset(ac, 0, sizeof(*ac));
}

void arm_control_set_targets(arm_control_t *ac, const float pos[ARM_JOINT_COUNT]) {
    memcpy(ac->target, pos, sizeof(ac->target));
    ac->dirty = 1;
}

void arm_control_set_torque(arm_control_t *ac, uint8_t mask) {
    ac->torque_mask = mask & ((1u << ARM_JOINT_COUNT) - 1);
}

int arm_control_tick(arm_control_t *ac) {
    if (!ac->dirty || ac->torque_mask == 0) {
        return 0;
    }
    int failed = 0;
    for (uint8_t j = 0; j < ARM_JOINT_COUNT; j++) {
        if (ac->torque_mask & (1u << j)) {
            if (servo_bus_set_target(j, ac->target[j]) != 0) {
                failed++;
            }
        }
    }
    ac->dirty = 0;
    return failed;
}

void arm_control_torque_off(arm_control_t *ac) {
    ac->torque_mask = 0;
}
