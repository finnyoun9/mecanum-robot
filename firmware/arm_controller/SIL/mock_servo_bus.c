#include "servo_bus.h"
#include "mock_servo_bus.h"

#define MOCK_MAX_WRITES 64

typedef struct {
    uint8_t joint;
    float   pos_rad;
} mock_write_t;

static mock_write_t writes[MOCK_MAX_WRITES];
static int n_writes = 0;
static int fail_next = 0;

int servo_bus_init(void) {
    n_writes = 0;
    fail_next = 0;
    return 0;
}

int servo_bus_set_target(uint8_t joint, float pos_rad) {
    if (fail_next) {
        fail_next = 0;
        return -1;
    }
    if (n_writes < MOCK_MAX_WRITES) {
        writes[n_writes].joint   = joint;
        writes[n_writes].pos_rad = pos_rad;
        n_writes++;
    }
    return 0;
}

int mock_bus_write_count(void) { return n_writes; }

int mock_bus_last_joint(void) {
    return n_writes ? writes[n_writes - 1].joint : -1;
}

float mock_bus_last_pos(void) {
    return n_writes ? writes[n_writes - 1].pos_rad : 0.0f;
}

void mock_bus_reset(void) {
    n_writes  = 0;
    fail_next = 0;
}

void mock_bus_fail_next(void) {
    fail_next = 1;
}
