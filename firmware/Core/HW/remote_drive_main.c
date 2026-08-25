/**
 * @file remote_drive_main.c
 * @brief NRF24L01 hand-held remote -> four-wheel open-loop drive.
 *
 * Glues together three modules that already existed but had never been
 * wired to each other on real hardware: the NRF24L01 receiver driver
 * (nrf24l01.c), the joystick -> mecanum mapping (remote_control.c, host
 * tested) and the motor driver (motor.c). Until now the robot side of the
 * remote had only been verified in unit tests.
 *
 * This is OPEN LOOP: the wheel speeds that come out of the inverse
 * kinematics are converted straight to PWM duty through the measured
 * duty->speed line. Encoders are not read and no PID runs. Its purpose is
 * to prove the radio link, the joystick mapping and the drive polarity on
 * the real chassis before M3 closes the loop.
 *
 * Controls (江协科技 controller):
 *   left stick   translation (up/down = forward/back, left/right = strafe)
 *   right stick  rotation (left/right)
 *   K1           toggle drive enable — motors stay dead until pressed
 *   K9           emergency stop
 *
 * Safety:
 *   - Drive starts DISABLED. K1 must be pressed before anything moves.
 *   - Losing the radio for REMOTE_TIMEOUT_MS stops the motors and drops
 *     both bridges. The controller sends nominally every 100 ms; the
 *     250 ms watchdog tolerates normal scheduling jitter and one missing
 *     frame, while still stopping a lost controller promptly.
 *
 * LIFT THE CHASSIS for the first run. Check that each stick direction
 * moves the robot the way it should and that K9 stops it, before putting
 * it on the ground.
 */
#include "stm32f1xx_hal.h"
#include "motor.h"
#include "nrf24l01.h"
#include "remote_control.h"

extern void SystemClock_Config(void);

#define PWM_PRESCALER   2U
#define PWM_PERIOD      999U

/* 10 Hz is a 100 ms nominal period.  A 100 ms watchdog races the next
 * legitimate packet (the measured cadence is 9.96 Hz), immediately drops
 * the K1 latch, and looks like a single twitch. */
#define REMOTE_TIMEOUT_MS  250U

static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim4;

static void Error_Handler(void) {
    __disable_irq();
    for (;;) {
    }
}

/* ======================================================================== *
 *  Wheel speed -> PWM duty
 * ======================================================================== */

/**
 * Convert a wheel speed to motor duty using the measured plant.
 *
 * The 2026-08-25 battery sweep (11.47 V, chassis lifted) fits closely to
 *   rad/s = 0.2875 * duty% - 0.79      over 20%..80%
 * so the inverse is used here. Open loop means this line IS the
 * calibration: there is no feedback to absorb an error in it.
 *
 * Anything below the startup deadband is pushed up to it. The sweep found
 * 5% duty moves nothing and 10% turns, so a command that rounds to less
 * than 10% would otherwise leave the wheel silently stalled while the
 * stick is pushed. Static friction, not the electrical response, sets that
 * floor.
 */
static int16_t speed_to_duty(float w_rad_s) {
    const float min_command = 0.5f;   /* rad/s below this means "stop" */
    const float duty_floor  = 100.0f; /* 10% — measured startup deadband */

    float mag = (w_rad_s < 0.0f) ? -w_rad_s : w_rad_s;
    if (mag < min_command) {
        return 0;
    }

    float duty = (mag + 0.79f) * 34.78f;   /* inverse of the measured line */
    if (duty < duty_floor) duty = duty_floor;
    if (duty > 1000.0f)    duty = 1000.0f;

    return (int16_t)((w_rad_s < 0.0f) ? -duty : duty);
}

/* ======================================================================== *
 *  Motor hardware — same mapping as drive_control / duty_sweep
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

    /* DIR pins passed (AIN2, AIN1) so positive duty == forward. */
    motor_set_tim(MOTOR_FL, &htim2, GPIOA, GPIO_PIN_5, GPIOA, GPIO_PIN_4, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_FR, &htim3, GPIOA, GPIO_PIN_12, GPIOA, GPIO_PIN_11, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_RL, &htim4, GPIOB, GPIO_PIN_15, GPIOB, GPIO_PIN_1, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_RR, &htim2, GPIOC, GPIO_PIN_14, GPIOC, GPIO_PIN_13, TIM_CHANNEL_4);
    motor_init();
    motor_emergency_stop();
    bridges_disable();
}

static void all_stop(void) {
    motor_emergency_stop();
    bridges_disable();
}

/* ======================================================================== *
 *  Telemetry — read over GDB when the radio misbehaves
 * ======================================================================== */

volatile uint32_t rx_packets   = 0;  /* valid remote packets received */
volatile uint8_t  drive_active = 0;  /* mirrors remote_state.enabled */
volatile uint8_t  failsafe_hits = 0; /* times the radio timeout fired */
volatile int16_t  last_duty[MOTOR_COUNT];

int main(void) {
    remote_state_t  state;
    remote_result_t result;

    HAL_Init();
    SystemClock_Config();
    drive_hardware_init();

    /* Frees PA15/PB3/PB4 from JTAG (SWD on PA13/PA14 survives) and brings
     * the radio up in RX mode. */
    nrf24l01_init();
    remote_init(&state);

    uint32_t last_rx_ms = HAL_GetTick();
    bool bridges_on = false;

    for (;;) {
        if (nrf24l01_receive()) {
            if (remote_process(nrf24l01_rx_packet(), &state, &result)) {
                last_rx_ms = HAL_GetTick();
                rx_packets++;

                if (result.key == REMOTE_KEY_ESTOP) {
                    state.enabled = false;
                }

                if (state.enabled) {
                    if (!bridges_on) {
                        motor_resume();
                        bridges_enable();
                        bridges_on = true;
                    }
                    /* wheel_speed is indexed w1..w4 = FL, FR, RL, RR, the
                     * same order as motor_id_t. */
                    for (uint32_t i = 0; i < MOTOR_COUNT; i++) {
                        int16_t duty = speed_to_duty(result.wheel_speed[i]);
                        last_duty[i] = duty;
                        motor_set_duty((motor_id_t)i, duty);
                    }
                } else if (bridges_on) {
                    all_stop();
                    bridges_on = false;
                    for (uint32_t i = 0; i < MOTOR_COUNT; i++) last_duty[i] = 0;
                }

                drive_active = state.enabled ? 1U : 0U;
            }
        }

        /* Radio failsafe. Also covers the controller being switched off
         * mid-drive, which otherwise leaves the last duty latched. */
        if ((HAL_GetTick() - last_rx_ms) > REMOTE_TIMEOUT_MS) {
            if (bridges_on) {
                all_stop();
                bridges_on = false;
                for (uint32_t i = 0; i < MOTOR_COUNT; i++) last_duty[i] = 0;
                if (failsafe_hits < 255U) failsafe_hits++;
            }
            state.enabled = false;
            drive_active = 0;
        }
    }
}
