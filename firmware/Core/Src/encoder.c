/**
 * @file encoder.c
 * @brief Quadrature encoder driver via STM32 TIM hardware encoder mode.
 *
 * Each motor encoder requires a general-purpose timer in encoder mode.
 * TIM->CNT auto-reloads at 0xFFFF; we track wrap events to extend to 32-bit.
 */

#include "encoder.h"
#include "tim.h"
#include <string.h>

/*
 * Example encoder TIM mapping:
 *   MOTOR_FL: TIM2  (16-bit, 4x counting on CH1+CH2)
 *   MOTOR_FR: TIM3
 *   MOTOR_RL: TIM4
 *   MOTOR_RR: TIM5
 */
typedef struct {
    /* TIM_HandleTypeDef *htim; */   /* Uncomment when HAL headers available */
    void *htim;                      /* Placeholder */
    int32_t  accum;                  /* Accumulated wraps (extend 16→32 bit) */
    uint16_t last_cnt;               /* Previous raw CNT for wrap detection */
    int32_t  prev_count;             /* Previous cumulative for speed calc */
} encoder_ctx_t;

static encoder_ctx_t encoders[MOTOR_COUNT];

void encoder_init(void) {
    /* Save htim pointers before clearing — they may have been wired by
     * encoder_set_tim() before robot_init() (needed for SIL testing). */
    void *saved_htim[MOTOR_COUNT];
    for (int i = 0; i < MOTOR_COUNT; i++) {
        saved_htim[i] = encoders[i].htim;
    }

    memset(encoders, 0, sizeof(encoders));

    for (int i = 0; i < MOTOR_COUNT; i++) {
        encoders[i].htim = saved_htim[i];
        if (encoders[i].htim) {
            HAL_TIM_Encoder_Start((TIM_HandleTypeDef *)encoders[i].htim,
                                  TIM_CHANNEL_ALL);
        }
    }
}

int32_t encoder_get_count(motor_id_t id) {
    if (id >= MOTOR_COUNT) return 0;

    encoder_ctx_t *e = &encoders[id];
    if (!e->htim) return 0;

    uint16_t raw = __HAL_TIM_GET_COUNTER((TIM_HandleTypeDef *)e->htim);

    /* Detect 16-bit wrap (TIM counts UP in encoder mode by default) */
    int16_t delta = (int16_t)(raw - e->last_cnt);
    e->accum += delta;
    e->last_cnt = raw;

    return e->accum;
}

float encoder_get_speed_rads(motor_id_t id, uint32_t delta_ms) {
    if (id >= MOTOR_COUNT || delta_ms == 0) return 0.0f;

    encoder_ctx_t *e = &encoders[id];
    int32_t count = encoder_get_count(id);
    int32_t delta_edges = count - e->prev_count;
    e->prev_count = count;

    /* Convert edges → wheel radians:
     *   edges / EDGES_PER_WHEEL_REV = wheel revolutions
     *   wheel_rev * 2π = radians
     *   / (delta_ms / 1000.0) = rad/s
     */
    float revs  = (float)delta_edges / (float)EDGES_PER_WHEEL_REV;
    float rads  = revs * 6.283185307f;
    float speed = rads / (delta_ms * 0.001f);

    return speed;
}

void encoder_reset_all(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        encoders[i].accum      = 0;
        encoders[i].last_cnt   = 0;
        encoders[i].prev_count = 0;
    }
}

void encoder_set_tim(motor_id_t id, void *htim) {
    if (id < MOTOR_COUNT) {
        encoders[id].htim = htim;
    }
}
