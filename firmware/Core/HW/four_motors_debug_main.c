/**
 * @file four_motors_debug_main.c
 * @brief Simultaneous open-loop test for all four TB6612 channels.
 *
 * FL: PA2=TIM2_CH3, PA4/PA5, STBY=PB14.
 * FR: PB0=TIM3_CH3, PA11/PA12, STBY=PB14.
 * RL: PB8=TIM4_CH3, PB1/PB15, STBY=PC15.
 * RR: PA3=TIM2_CH4, PC13/PC14, STBY=PC15.
 */
#include "stm32f1xx_hal.h"

extern void SystemClock_Config(void);

#define PWM_PERIOD       3199U /* 64 MHz timer clock / 20 kHz */
#define PWM_DUTY_70_PCT  2240U

static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim4;

static void Error_Handler(void) {
    __disable_irq();
    for (;;) {
    }
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
    timer->Init.Prescaler = 0U;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = PWM_PERIOD;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(timer) != HAL_OK) {
        Error_Handler();
    }
}

static void pwm_channel_start(TIM_HandleTypeDef *timer, uint32_t channel_number) {
    TIM_OC_InitTypeDef channel = {0};

    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(timer, &channel, channel_number) != HAL_OK ||
        HAL_TIM_PWM_Start(timer, channel_number) != HAL_OK) {
        Error_Handler();
    }
}

static void motors_stop(void) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
}

static void motors_drive(GPIO_PinState direction) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_11, direction);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_12,
                      direction == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, direction);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,
                      direction == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, direction);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,
                      direction == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_DUTY_70_PCT);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_DUTY_70_PCT);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_DUTY_70_PCT);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_DUTY_70_PCT);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    motor_gpio_init();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    timer_init(&htim2, TIM2);
    timer_init(&htim3, TIM3);
    timer_init(&htim4, TIM4);
    pwm_channel_start(&htim2, TIM_CHANNEL_3);
    pwm_channel_start(&htim2, TIM_CHANNEL_4);
    pwm_channel_start(&htim3, TIM_CHANNEL_3);
    pwm_channel_start(&htim4, TIM_CHANNEL_3);
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
    }
}
