/**
 * @file fl_motor_debug_main.c
 * @brief One-shot open-loop test for the front-left TB6612 channel.
 *
 * Board 1 / front-left motor wiring:
 *   PB14 = TB6612 STBY, PA2 = PWMA (TIM2_CH3), PA4 = AIN1, PA5 = AIN2.
 *
 * The driver starts disabled. After a two-second observation window it runs
 * +20% PWM for one second, stops for one second, then runs -20% for one
 * second. It then disables STBY permanently until reset. Other motor pins
 * are never configured by this binary.
 */
#include "stm32f1xx_hal.h"

extern void SystemClock_Config(void);

#define FL_PWM_PERIOD       3199U /* 64 MHz TIM2 clock / 20 kHz */
#define FL_PWM_DUTY_20_PCT   640U

typedef enum {
    FL_DEBUG_SAFE = 0,
    FL_DEBUG_FORWARD,
    FL_DEBUG_STOPPED,
    FL_DEBUG_REVERSE,
    FL_DEBUG_DONE,
} fl_debug_phase_t;

static TIM_HandleTypeDef htim2;

/* Read these through GDB while the target is halted. */
volatile fl_debug_phase_t fl_debug_phase = FL_DEBUG_SAFE;
volatile uint32_t fl_debug_compare = 0U;

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

static void fl_motor_gpio_init(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* Hold the bridge disabled and both direction inputs low during setup. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

    gpio.Pin = GPIO_PIN_14;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void fl_motor_pwm_init(void) {
    TIM_OC_InitTypeDef channel = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0U;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = FL_PWM_PERIOD;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &channel, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
}

static void fl_motor_stop(void) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
    fl_debug_compare = 0U;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
}

static void fl_motor_drive(GPIO_PinState ain1) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, ain1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,
                      ain1 == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, FL_PWM_DUTY_20_PCT);
    fl_debug_compare = FL_PWM_DUTY_20_PCT;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    heartbeat_init();
    fl_motor_gpio_init();
    fl_motor_pwm_init();
    fl_motor_stop();

    HAL_Delay(2000U);

    fl_debug_phase = FL_DEBUG_FORWARD;
    fl_motor_drive(GPIO_PIN_SET);
    HAL_Delay(1000U);

    fl_debug_phase = FL_DEBUG_STOPPED;
    fl_motor_stop();
    HAL_Delay(1000U);

    fl_debug_phase = FL_DEBUG_REVERSE;
    fl_motor_drive(GPIO_PIN_RESET);
    HAL_Delay(1000U);

    fl_debug_phase = FL_DEBUG_DONE;
    fl_motor_stop();

    for (;;) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(500U);
    }
}
