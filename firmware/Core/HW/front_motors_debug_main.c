/**
 * @file front_motors_debug_main.c
 * @brief Simultaneous open-loop test for the two front TB6612 channels.
 *
 * Front-left:  PA2=PWMA/TIM2_CH3, PA4=AIN1, PA5=AIN2.
 * Front-right: PB0=PWMB/TIM3_CH3, PA11=BIN1, PA12=BIN2.
 * Shared: PB14=STBY.
 */
#include "stm32f1xx_hal.h"

extern void SystemClock_Config(void);

#define PWM_PERIOD       3199U /* 64 MHz timer clock / 20 kHz */
#define PWM_DUTY_70_PCT  2240U

static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim3;

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

static void motor_gpio_init(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12,
                      GPIO_PIN_RESET);

    gpio.Pin = GPIO_PIN_14;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void pwm_init(TIM_HandleTypeDef *timer, TIM_TypeDef *instance) {
    TIM_OC_InitTypeDef channel = {0};

    timer->Instance = instance;
    timer->Init.Prescaler = 0U;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = PWM_PERIOD;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(timer) != HAL_OK) {
        Error_Handler();
    }

    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(timer, &channel, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(timer, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
}

static void motors_stop(void) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12,
                      GPIO_PIN_RESET);
}

static void motors_drive(GPIO_PinState direction) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_11, direction);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_12,
                      direction == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_DUTY_70_PCT);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_DUTY_70_PCT);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    heartbeat_init();
    motor_gpio_init();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    pwm_init(&htim2, TIM2);
    pwm_init(&htim3, TIM3);
    motors_stop();

    HAL_Delay(2000U);
    motors_drive(GPIO_PIN_SET);
    HAL_Delay(5000U);
    motors_stop();
    HAL_Delay(1000U);
    motors_drive(GPIO_PIN_RESET);
    HAL_Delay(5000U);
    motors_stop();

    for (;;) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(500U);
    }
}
