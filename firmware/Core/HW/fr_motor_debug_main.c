/**
 * @file fr_motor_debug_main.c
 * @brief One-shot open-loop test for the front-right TB6612 channel.
 *
 * Board 1 / front-right motor wiring:
 *   PB14 = TB6612 STBY, PB0 = PWMB (TIM3_CH3), PA11 = BIN1, PA12 = BIN2.
 *
 * The driver starts disabled. After a two-second observation window it runs
 * +70% PWM for five seconds, stops for one second, then runs -70% for five
 * second. It then disables STBY permanently until reset. Other motor pins
 * are never configured by this binary.
 */
#include "stm32f1xx_hal.h"

extern void SystemClock_Config(void);

#define FR_PWM_PERIOD       3199U /* 64 MHz TIM3 clock / 20 kHz */
#define FR_PWM_DUTY_70_PCT  2240U

typedef enum {
    FR_DEBUG_SAFE = 0,
    FR_DEBUG_FORWARD,
    FR_DEBUG_STOPPED,
    FR_DEBUG_REVERSE,
    FR_DEBUG_DONE,
} fr_debug_phase_t;

static TIM_HandleTypeDef htim3;

/* Read these through GDB while the target is halted. */
volatile fr_debug_phase_t fr_debug_phase = FR_DEBUG_SAFE;
volatile uint32_t fr_debug_compare = 0U;

static void Error_Handler(void) {
    __disable_irq();
    for (;;) {
    }
}

static void heartbeat_init(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_13;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
}

static void fr_motor_gpio_init(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* Hold the bridge disabled and both direction inputs low during setup. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);

    gpio.Pin = GPIO_PIN_14;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void fr_motor_pwm_init(void) {
    TIM_OC_InitTypeDef channel = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0U;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = FR_PWM_PERIOD;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &channel, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
}

static void fr_motor_stop(void) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
    fr_debug_compare = 0U;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);
}

static void fr_motor_drive(GPIO_PinState bin1) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, bin1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,
                      bin1 == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, FR_PWM_DUTY_70_PCT);
    fr_debug_compare = FR_PWM_DUTY_70_PCT;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    heartbeat_init();
    fr_motor_gpio_init();
    fr_motor_pwm_init();
    fr_motor_stop();

    HAL_Delay(2000U);

    fr_debug_phase = FR_DEBUG_FORWARD;
    fr_motor_drive(GPIO_PIN_SET);
    HAL_Delay(5000U);

    fr_debug_phase = FR_DEBUG_STOPPED;
    fr_motor_stop();
    HAL_Delay(5000U);

    fr_debug_phase = FR_DEBUG_REVERSE;
    fr_motor_drive(GPIO_PIN_RESET);
    HAL_Delay(1000U);

    fr_debug_phase = FR_DEBUG_DONE;
    fr_motor_stop();

    for (;;) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(500U);
    }
}
