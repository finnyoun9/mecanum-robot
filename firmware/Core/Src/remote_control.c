/**
 * @file remote_control.c
 * @brief Map the hand-held controller's joysticks onto mecanum motion.
 *
 * Pure C so it can be unit-tested on the host (see firmware/Core/Test).
 * The NRF24L01 receive polling lives in the RemoteTask in main.c; this
 * module only parses and converts.
 */

#include "remote_control.h"
#include "nrf24l01.h"

/* Protocol field indices (match the controller firmware) */
#define PROTO_OFF_ID   0
#define PROTO_OFF_LH   1
#define PROTO_OFF_LV   2
#define PROTO_OFF_RH   3
#define PROTO_OFF_RV   4
#define PROTO_OFF_KEY  5
#define PROTO_ID_REMOTE 0x00

/* Wheel geometry — must match mcr.urdf.xacro and the ROS2 hardware
 * interface. Radius is MEASURED: 60 mm wheels, so 0.030 m. It read 0.0325
 * here until 2026-08-25, an 8% error left behind when the measurement
 * landed in encoder.h, the ROS 2 interface and the simulator but not in
 * this file.
 *
 * lx/ly remain ESTIMATES and have never been measured — see
 * mcr_hardware_interface.cpp and the roadmap's M4 section. */
static const mecanum_ik_config_t ik_cfg = {
    .wheel_radius = 0.030f,
    .lx = 0.10f,
    .ly = 0.12f,
};

/* Clamp to [-limit, +limit]. */
static float clamp(float v, float limit)
{
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

void remote_init(remote_state_t *state)
{
    state->enabled  = false;
    state->last_key = 0;
}

bool remote_process(const uint8_t packet[NRF24L01_PACKET_WIDTH],
                    remote_state_t *state, remote_result_t *out)
{
    if (packet[PROTO_OFF_ID] != PROTO_ID_REMOTE) {
        return false;  /* Not a remote-control packet */
    }

    int8_t lh = (int8_t)packet[PROTO_OFF_LH];
    int8_t lv = (int8_t)packet[PROTO_OFF_LV];
    int8_t rh = (int8_t)packet[PROTO_OFF_RH];
    /* RV (packet[4]) reserved — could map to e.g. altitude or deadman. */

    uint8_t key = packet[PROTO_OFF_KEY];
    out->key = key;

    if (key == REMOTE_KEY_TOGGLE_ENABLE) {
        state->enabled = !state->enabled;
    }
    state->last_key = key;

    /* Scale joystick (-100..100) to twist (m/s, rad/s). */
    float vx    = clamp((float)lv / 100.0f, 1.0f) * REMOTE_VX_MAX;
    float vy    = clamp((float)lh / 100.0f, 1.0f) * REMOTE_VY_MAX;
    /* Hand controller RH decreases when the stick moves left.  Negate it
     * so the conventional command holds: stick-left turns the robot CCW
     * (its nose left), while stick-right turns it CW. */
    float omega = -clamp((float)rh / 100.0f, 1.0f) * REMOTE_OMEGA_MAX;

    mecanum_ik(&ik_cfg, vx, vy, omega, out->wheel_speed);

    return true;
}
