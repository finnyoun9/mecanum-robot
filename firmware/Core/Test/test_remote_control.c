/**
 * @file test_remote_control.c
 * @brief Host-buildable unit tests for remote_control.c.
 *
 * No test framework — plain main() with assert(). Pure C, builds and runs
 * on a host machine without STM32/FreeRTOS deps (nrf24l01.h is HAL-free).
 *
 * Build & run (from this directory):
 *   gcc -Wall -Wextra -std=c11 -I../Inc -lm test_remote_control.c ../Src/remote_control.c ../Src/mecanum_ik.c -o test_remote_control && ./test_remote_control
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include "remote_control.h"
#include "nrf24l01.h"

static void make_packet(uint8_t p[NRF24L01_PACKET_WIDTH],
                        int8_t lh, int8_t lv, int8_t rh, int8_t rv, uint8_t key)
{
    memset(p, 0, NRF24L01_PACKET_WIDTH);
    p[0] = 0x00;              /* ID: remote-control packet */
    p[1] = (uint8_t)lh;
    p[2] = (uint8_t)lv;
    p[3] = (uint8_t)rh;
    p[4] = (uint8_t)rv;
    p[5] = key;
}

static int close(float a, float b, float tol)
{
    return fabsf(a - b) <= tol;
}

int main(void)
{
    remote_state_t st;
    remote_result_t res;
    uint8_t p[NRF24L01_PACKET_WIDTH];

    /* (1) Start disabled; non-remote packets are ignored. */
    remote_init(&st);
    assert(st.enabled == false);
    memset(p, 0xFF, NRF24L01_PACKET_WIDTH);  /* ID != 0x00 */
    assert(remote_process(p, &st, &res) == false);
    assert(st.enabled == false);

    /* (2) Full forward (LV=+100): all four wheels same positive speed. */
    make_packet(p, 0, 100, 0, 0, 0);
    assert(remote_process(p, &st, &res) == true);
    assert(res.key == 0);
    for (int i = 0; i < 4; i++) {
        assert(res.wheel_speed[i] > 0.0f);
    }
    printf("forward: w=%.2f rad/s\n", res.wheel_speed[0]);

    /* (3) Full reverse (LV=-100): all wheels negative. */
    make_packet(p, 0, -100, 0, 0, 0);
    assert(remote_process(p, &st, &res) == true);
    for (int i = 0; i < 4; i++) {
        assert(res.wheel_speed[i] < 0.0f);
    }

    /* (4) Strafe (LH=+100) vs rotate (RH=+100) produce distinct patterns.
     *     Strafe: same-side wheels pair up (FL=RR, FR=RL).
     *     Rotate: same-axle wheels pair up (FL=RL, FR=RR).
     *     In both, left wheels oppose right wheels. */
    make_packet(p, 100, 0, 0, 0, 0);
    assert(remote_process(p, &st, &res) == true);
    assert(close(res.wheel_speed[0], res.wheel_speed[3], 1e-4f));  /* FL=RR */
    assert(close(res.wheel_speed[1], res.wheel_speed[2], 1e-4f));  /* FR=RL */
    assert(close(res.wheel_speed[0], -res.wheel_speed[1], 1e-4f));

    make_packet(p, 0, 0, 100, 0, 0);
    assert(remote_process(p, &st, &res) == true);
    assert(close(res.wheel_speed[0], res.wheel_speed[2], 1e-4f));  /* FL=RL */
    assert(close(res.wheel_speed[1], res.wheel_speed[3], 1e-4f));  /* FR=RR */
    assert(close(res.wheel_speed[0], -res.wheel_speed[1], 1e-4f));

    /* (5) K1 toggles enable; K9 reports emergency-stop event. */
    make_packet(p, 0, 0, 0, 0, 1);
    assert(remote_process(p, &st, &res) == true);
    assert(st.enabled == true);

    make_packet(p, 0, 0, 0, 0, 1);
    assert(remote_process(p, &st, &res) == true);
    assert(st.enabled == false);

    make_packet(p, 0, 0, 0, 0, 9);
    assert(remote_process(p, &st, &res) == true);
    assert(res.key == REMOTE_KEY_ESTOP);

    printf("ALL REMOTE CONTROL TESTS PASSED\n");
    return 0;
}
