/**
 * @file remote_pid_drive_main.c
 * @brief M3 bench target: NRF24 remote -> four independent wheel-speed PI loops.
 *
 * This target deliberately leaves remote_drive_main.c untouched: the existing
 * open-loop remote controller remains a known-good fallback.  Start disabled;
 * K1 enables control, K9 or a 250 ms radio timeout stops all motors and drops
 * both TB6612 standby lines.  First run only with the chassis lifted.
 */
#include "stm32f1xx_hal.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "nrf24l01.h"
#include "remote_control.h"
#include <math.h>
#include <stdlib.h>

extern void SystemClock_Config(void);

#define PWM_PRESCALER             2U
#define PWM_PERIOD                 999U
#define CTRL_PERIOD_MS             10U
#define REMOTE_TIMEOUT_MS          250U
#define ENCODER_DEBOUNCE_CYCLES    9600U  /* 150 us at the 64 MHz HW clock */

/* M2's Kp=100/Ki=300 were valid for one isolated FR step test, but overdrive
 * the four-wheel plant.  The measured open-loop inverse supplies most of the
 * effort; PI only corrects wheel-to-wheel variation. */
#define PID_KP                     15.0f
#define PID_KI                     35.0f
#define PID_KD                     0.0f
#define PID_CORRECTION_MAX         350.0f

/* The battery sweep reached 22.21 rad/s at 80% duty with the chassis lifted.
 * Limit incoming IK commands to a slightly conservative, measured range. */
#define MAX_TARGET_RAD_S           20.0f
#define TARGET_SLEW_RAD_S_PER_TICK 1.0f
#define STOP_TARGET_RAD_S          0.25f
#define OVERSPEED_RAD_S            28.0f
#define OVERSPEED_TICKS            3U      /* reject one 10 ms quantisation spike */
#define STALL_TARGET_RAD_S         3.0f
#define STALL_SPEED_RAD_S          0.35f
#define STALL_DUTY                 900
#define STALL_TICKS                30U     /* 300 ms at 100 Hz */

static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim4;
static pid_ctrl_t wheel_pid[MOTOR_COUNT];
static volatile uint32_t last_cycle[MOTOR_COUNT];

/* GDB telemetry.  All motion starts disabled; observing these never enables a
 * bridge or changes a target. */
volatile uint32_t rx_packets;
volatile uint32_t control_ticks;
volatile uint8_t  drive_active;
volatile uint8_t  failsafe_hits;
volatile uint8_t  encoder_fault;
volatile uint8_t  encoder_fault_wheel;
volatile uint8_t  encoder_fault_reason; /* 1=overspeed, 2=stall */
volatile float    requested_speed[MOTOR_COUNT];
volatile float    target_speed[MOTOR_COUNT];
volatile float    measured_speed[MOTOR_COUNT];
volatile int16_t  last_duty[MOTOR_COUNT];
volatile float    fault_target_speed;
volatile float    fault_measured_speed;
volatile int16_t  fault_duty;

static uint8_t stall_ticks[MOTOR_COUNT];
static uint8_t overspeed_ticks[MOTOR_COUNT];

static void Error_Handler(void) {
    __disable_irq();
    for (;;) {
    }
}

static float clamp_speed(float speed) {
    if (speed > MAX_TARGET_RAD_S) return MAX_TARGET_RAD_S;
    if (speed < -MAX_TARGET_RAD_S) return -MAX_TARGET_RAD_S;
    return speed;
}

static float speed_feedforward(float speed) {
    /* Inverse of the lifted-chassis sweep: rad/s = 0.2875*duty% - 0.79.
     * Keep this continuous around zero; the PI term supplies any extra
     * startup effort without turning a small command into a full-PWM step. */
    float magnitude = fabsf(speed);
    float duty;

    if (magnitude < STOP_TARGET_RAD_S) return 0.0f;
    duty = (magnitude + 0.79f) * 34.78f;
    if (duty > 1000.0f) duty = 1000.0f;
    return (speed < 0.0f) ? -duty : duty;
}

static float slew_towards(float current, float requested) {
    if (requested > current + TARGET_SLEW_RAD_S_PER_TICK) {
        return current + TARGET_SLEW_RAD_S_PER_TICK;
    }
    if (requested < current - TARGET_SLEW_RAD_S_PER_TICK) {
        return current - TARGET_SLEW_RAD_S_PER_TICK;
    }
    return requested;
}

static void dwt_cycle_counter_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static bool debounce_ok(motor_id_t id) {
    uint32_t now = DWT->CYCCNT;
    if ((now - last_cycle[id]) < ENCODER_DEBOUNCE_CYCLES) return false;
    last_cycle[id] = now;
    return true;
}

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
    if (HAL_TIM_PWM_Init(timer) != HAL_OK) Error_Handler();
}

static void pwm_channel_config(TIM_HandleTypeDef *timer, uint32_t channel_number) {
    TIM_OC_InitTypeDef channel = {0};
    channel.OCMode = TIM_OCMODE_PWM1;
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

    /* Direction order is (AIN2, AIN1): positive duty means physical forward. */
    motor_set_tim(MOTOR_FL, &htim2, GPIOA, GPIO_PIN_5, GPIOA, GPIO_PIN_4, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_FR, &htim3, GPIOA, GPIO_PIN_12, GPIOA, GPIO_PIN_11, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_RL, &htim4, GPIOB, GPIO_PIN_15, GPIOB, GPIO_PIN_1, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_RR, &htim2, GPIOC, GPIO_PIN_14, GPIOC, GPIO_PIN_13, TIM_CHANNEL_4);
    motor_init();
    motor_emergency_stop();
    bridges_disable();
}

static void encoder_gpio_init(void) {
    GPIO_InitTypeDef gpio = {0};

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = GPIO_PIN_1 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_7 | GPIO_PIN_12;
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
        if (debounce_ok(MOTOR_FL)) {
            encoder_on_edge(MOTOR_FL, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET,
                            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET);
        }
        break;
    case GPIO_PIN_6:
        if (debounce_ok(MOTOR_FR)) {
            encoder_on_edge(MOTOR_FR, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET,
                            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET);
        }
        break;
    case GPIO_PIN_7:
        if (debounce_ok(MOTOR_RL)) {
            encoder_on_edge(MOTOR_RL, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET,
                            HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET);
        }
        break;
    case GPIO_PIN_12:
        if (debounce_ok(MOTOR_RR)) {
            encoder_on_edge(MOTOR_RR,
                            HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET,
                            HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET);
        }
        break;
    default:
        break;
    }
}

void EXTI0_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); }
void EXTI9_5_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
}
void EXTI15_10_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12); }

static void reset_control(void) {
    for (uint32_t i = 0; i < MOTOR_COUNT; i++) {
        pid_reset(&wheel_pid[i]);
        requested_speed[i] = 0.0f;
        target_speed[i] = 0.0f;
        measured_speed[i] = 0.0f;
        last_duty[i] = 0;
        stall_ticks[i] = 0U;
        overspeed_ticks[i] = 0U;
    }
}

static void all_stop(void) {
    motor_emergency_stop();
    bridges_disable();
    reset_control();
}

static void control_init(void) {
    const float integral_max = PID_CORRECTION_MAX / PID_KI;
    for (uint32_t i = 0; i < MOTOR_COUNT; i++) {
        pid_init(&wheel_pid[i], PID_KP, PID_KI, PID_KD, PID_CORRECTION_MAX,
                 integral_max, (float)CTRL_PERIOD_MS * 0.001f);
    }
    encoder_fault_wheel = UINT8_MAX;
    reset_control();
}

static uint8_t wheel_fault(motor_id_t id, float target, float measured, int16_t duty) {
    if (fabsf(measured) > OVERSPEED_RAD_S) {
        if (overspeed_ticks[id] < OVERSPEED_TICKS) overspeed_ticks[id]++;
    } else {
        overspeed_ticks[id] = 0U;
    }
    if (overspeed_ticks[id] >= OVERSPEED_TICKS) return 1U;

    if (fabsf(target) > STALL_TARGET_RAD_S && fabsf(measured) < STALL_SPEED_RAD_S &&
        abs(duty) >= STALL_DUTY) {
        if (stall_ticks[id] < STALL_TICKS) stall_ticks[id]++;
    } else {
        stall_ticks[id] = 0U;
    }
    return (stall_ticks[id] >= STALL_TICKS) ? 2U : 0U;
}

static void control_tick(void) {
    for (uint32_t i = 0; i < MOTOR_COUNT; i++) {
        motor_id_t id = (motor_id_t)i;
        float requested = requested_speed[i];
        float target = slew_towards(target_speed[i], requested);
        float measured = encoder_get_speed_rads(id, CTRL_PERIOD_MS);
        float output;
        int16_t duty;

        if (fabsf(requested) < STOP_TARGET_RAD_S &&
            fabsf(target_speed[i]) < STOP_TARGET_RAD_S) {
            /* Do not let an integral residue dither a stopped wheel in its
             * static-friction band.  PWM=0 coasts cleanly; the next motion
             * command begins from a fresh PI state. */
            target_speed[i] = 0.0f;
            measured_speed[i] = measured;
            pid_reset(&wheel_pid[i]);
            last_duty[i] = 0;
            motor_set_duty(id, 0);
            continue;
        }

        target_speed[i] = target;
        measured_speed[i] = measured;
        pid_setpoint(&wheel_pid[i], target);
        output = speed_feedforward(target) + pid_update(&wheel_pid[i], measured);
        if (output > 1000.0f) output = 1000.0f;
        if (output < -1000.0f) output = -1000.0f;
        duty = (int16_t)output;
        last_duty[i] = duty;

        uint8_t fault_reason = wheel_fault(id, target, measured, duty);
        if (fault_reason != 0U) {
            encoder_fault = 1U;
            encoder_fault_wheel = (uint8_t)i;
            encoder_fault_reason = fault_reason;
            fault_target_speed = target;
            fault_measured_speed = measured;
            fault_duty = duty;
            return;
        }
        motor_set_duty(id, duty);
    }
    control_ticks++;
}

int main(void) {
    remote_state_t state;
    remote_result_t result;
    uint32_t last_rx_ms;
    uint32_t next_control_ms;
    bool bridges_on = false;

    HAL_Init();
    SystemClock_Config();
    dwt_cycle_counter_init();
    drive_hardware_init();
    encoder_gpio_init();
    encoder_init();
    control_init();
    nrf24l01_init();
    remote_init(&state);

    last_rx_ms = HAL_GetTick();
    next_control_ms = last_rx_ms + CTRL_PERIOD_MS;

    for (;;) {
        uint32_t now = HAL_GetTick();

        if (nrf24l01_receive() &&
            remote_process(nrf24l01_rx_packet(), &state, &result)) {
            last_rx_ms = now;
            rx_packets++;

            if (result.key == REMOTE_KEY_ESTOP) state.enabled = false;

            if (state.enabled && !encoder_fault) {
                if (!bridges_on) {
                    encoder_reset_all();
                    reset_control();
                    motor_resume();
                    bridges_enable();
                    bridges_on = true;
                }
                for (uint32_t i = 0; i < MOTOR_COUNT; i++) {
                    requested_speed[i] = clamp_speed(result.wheel_speed[i]);
                }
            } else if (bridges_on) {
                all_stop();
                bridges_on = false;
            }
            drive_active = (state.enabled && !encoder_fault) ? 1U : 0U;
        }

        if (bridges_on && (int32_t)(now - next_control_ms) >= 0) {
            next_control_ms = now + CTRL_PERIOD_MS;
            control_tick();
            if (encoder_fault) {
                state.enabled = false;
                all_stop();
                bridges_on = false;
                drive_active = 0U;
            }
        }

        if ((now - last_rx_ms) > REMOTE_TIMEOUT_MS) {
            if (bridges_on) {
                all_stop();
                bridges_on = false;
                if (failsafe_hits < UINT8_MAX) failsafe_hits++;
            }
            state.enabled = false;
            drive_active = 0U;
        }
    }
}
