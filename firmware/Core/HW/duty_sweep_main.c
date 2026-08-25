/**
 * @file duty_sweep_main.c
 * @brief Measures each wheel's steady-state speed across a range of PWM
 *        duty cycles, to find the deadband and the duty -> rad/s curve.
 *
 * Roadmap M1 step 3. Runs a fixed sequence of duty levels; at each level
 * the wheels spin for STEP_RUN_MS, then the per-wheel encoder delta over
 * that window is recorded. Duty steps are swept low-to-high and start well
 * below the expected deadband, so the first non-zero row shows where each
 * motor actually starts turning.
 *
 * Per-wheel deltas are kept separate rather than averaged: the four
 * motors are not assumed to share a deadband or gain, and knowing the
 * spread is the point (a shared PID tuning is only justified if they
 * turn out close).
 *
 * LIFT THE CHASSIS before flashing — wheels spin continuously for the
 * whole sweep. Both bridges are disabled before and after.
 *
 * Reading results (target must free-run to completion first, ~17s; note
 * st-util resets on GDB attach, so attach then continue, don't halt
 * immediately):
 *   (gdb) print step_duty
 *   (gdb) print step_fl
 *   (gdb) print step_fr
 *   (gdb) print step_rl
 *   (gdb) print step_rr
 *   (gdb) print sweep_done
 *
 * Convert a delta to wheel rev/s:
 *   rev_per_s = delta / EDGES_PER_WHEEL_REV / (STEP_RUN_MS / 1000)
 * with EDGES_PER_WHEEL_REV = 448 (measured at 1x then doubled for both-edge
 * decoding, see encoder.h).
 */
#include "stm32f1xx_hal.h"
#include "motor.h"

extern void SystemClock_Config(void);

#define ENCODER_DEBOUNCE_CYCLES  9600U

#define PWM_PRESCALER   2U
#define PWM_PERIOD      999U

#define STEP_RUN_MS     1000U   /* spin time per duty level */
#define STEP_SETTLE_MS   400U   /* let the wheel stop between levels */

/* Fine steps at the bottom to locate the deadband, coarser above it. */
static const uint16_t sweep_duties[] = {
    50U, 100U, 150U, 200U, 300U, 400U, 500U, 600U, 700U, 800U
};
#define SWEEP_STEPS (sizeof(sweep_duties) / sizeof(sweep_duties[0]))

/* Results, read via GDB after the sweep finishes. */
volatile uint16_t step_duty[SWEEP_STEPS];
volatile int32_t  step_fl[SWEEP_STEPS];
volatile int32_t  step_fr[SWEEP_STEPS];
volatile int32_t  step_rl[SWEEP_STEPS];
volatile int32_t  step_rr[SWEEP_STEPS];
volatile uint8_t  sweep_done = 0;

volatile int32_t fl_count = 0;
volatile int32_t fr_count = 0;
volatile int32_t rl_count = 0;
volatile int32_t rr_count = 0;

static volatile uint32_t fl_last_cycle = 0;
static volatile uint32_t fr_last_cycle = 0;
static volatile uint32_t rl_last_cycle = 0;
static volatile uint32_t rr_last_cycle = 0;

static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim4;

static void Error_Handler(void) {
    __disable_irq();
    for (;;) {
    }
}

static void dwt_cycle_counter_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static bool debounce_ok(volatile uint32_t *last_cycle) {
    uint32_t now = DWT->CYCCNT;
    if ((now - *last_cycle) < ENCODER_DEBOUNCE_CYCLES) {
        return false;
    }
    *last_cycle = now;
    return true;
}

/* ======================================================================== *
 *  Motor init — same mapping as drive_control/encoder_debug
 * ======================================================================== */

static void motor_gpio_init(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_1 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_8;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void timer_init(TIM_HandleTypeDef *timer, TIM_TypeDef *instance) {
    timer->Instance = instance;
    timer->Init.Prescaler = PWM_PRESCALER;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = PWM_PERIOD;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(timer) != HAL_OK) {
        Error_Handler();
    }
}

static void pwm_channel_config(TIM_HandleTypeDef *timer, uint32_t channel_number) {
    TIM_OC_InitTypeDef channel = {0};

    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(timer, &channel, channel_number) != HAL_OK) {
        Error_Handler();
    }
}

static void bridges_enable(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
}

static void bridges_disable(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
}

static void drive_hardware_init(void) {
    motor_gpio_init();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    timer_init(&htim2, TIM2);
    timer_init(&htim3, TIM3);
    timer_init(&htim4, TIM4);
    pwm_channel_config(&htim2, TIM_CHANNEL_3);
    pwm_channel_config(&htim2, TIM_CHANNEL_4);
    pwm_channel_config(&htim3, TIM_CHANNEL_3);
    pwm_channel_config(&htim4, TIM_CHANNEL_3);

    /* DIR pins passed (AIN2, AIN1) so positive duty == forward; see
     * drive_control_main.c for the hardware verification note. */
    motor_set_tim(MOTOR_FL, &htim2, GPIOA, GPIO_PIN_5, GPIOA, GPIO_PIN_4, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_FR, &htim3, GPIOA, GPIO_PIN_12, GPIOA, GPIO_PIN_11, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_RL, &htim4, GPIOB, GPIO_PIN_15, GPIOB, GPIO_PIN_1, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_RR, &htim2, GPIOC, GPIO_PIN_14, GPIOC, GPIO_PIN_13, TIM_CHANNEL_4);
    motor_init();
    motor_emergency_stop();
    bridges_disable();
}

/* ======================================================================== *
 *  Encoder EXTI — same allocation as encoder_debug
 * ======================================================================== */

static void encoder_gpio_init(void) {
    GPIO_InitTypeDef gpio = {0};

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    switch (GPIO_Pin) {
    case GPIO_PIN_0:
        if (!debounce_ok(&fl_last_cycle)) break;
        /* FL: A = PA0, B = PA1 -- direction is a_level == b_level, valid on both of A's edges. */
        if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) ==
            (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET)) fl_count++; else fl_count--;
        break;
    case GPIO_PIN_6:
        if (!debounce_ok(&fr_last_cycle)) break;
        /* FR: A = PA6, B = PA7 -- direction is a_level == b_level, valid on both of A's edges. */
        if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) ==
            (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET)) fr_count++; else fr_count--;
        break;
    case GPIO_PIN_7:
        if (!debounce_ok(&rl_last_cycle)) break;
        /* RL: A = PB7, B = PB6 -- direction is a_level == b_level, valid on both of A's edges. */
        if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) ==
            (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)) rl_count++; else rl_count--;
        break;
    case GPIO_PIN_12:
        if (!debounce_ok(&rr_last_cycle)) break;
        /* RR: A = PB12, B = PB13 -- direction is a_level == b_level, valid on both of A's edges. */
        if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) ==
            (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET)) rr_count++; else rr_count--;
        break;
    default:
        break;
    }
}

void EXTI0_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void EXTI9_5_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
}

void EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
}

/* ======================================================================== *
 *  Sweep
 * ======================================================================== */

int main(void) {
    HAL_Init();
    SystemClock_Config();
    dwt_cycle_counter_init();
    drive_hardware_init();
    encoder_gpio_init();

    HAL_Delay(2000U);

    for (uint32_t i = 0U; i < SWEEP_STEPS; i++) {
        const uint16_t duty = sweep_duties[i];

        motor_resume();
        for (uint32_t wheel = 0U; wheel < MOTOR_COUNT; wheel++) {
            motor_set_duty((motor_id_t)wheel, (int16_t)duty);
        }
        bridges_enable();

        /* Sample after the wheel has had a moment to reach steady state,
         * so the recorded window excludes spin-up. */
        HAL_Delay(300U);
        const int32_t fl0 = fl_count;
        const int32_t fr0 = fr_count;
        const int32_t rl0 = rl_count;
        const int32_t rr0 = rr_count;

        HAL_Delay(STEP_RUN_MS);

        step_duty[i] = duty;
        step_fl[i] = fl_count - fl0;
        step_fr[i] = fr_count - fr0;
        step_rl[i] = rl_count - rl0;
        step_rr[i] = rr_count - rr0;

        motor_emergency_stop();
        bridges_disable();
        HAL_Delay(STEP_SETTLE_MS);
    }

    sweep_done = 1U;

    for (;;) {
    }
}
