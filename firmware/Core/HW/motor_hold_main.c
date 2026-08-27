/**
 * @file motor_hold_main.c
 * @brief Wiring diagnostic: hold all four TB6612 channels at a fixed
 *        duty forever, so a multimeter can probe each signal at leisure.
 *
 * Same pin/timer map and init as four_motors_debug (proven on the old
 * board), but instead of a timed sequence it enables both STBY lines and
 * drives all four wheels forward at HOLD_DUTY continuously until reset.
 *
 * Expected DC-average readings at the Blue Pill pins (20 kHz PWM):
 *   STBY  PB14 / PC15  -> steady ~3.3 V
 *   DIR   (forward)    -> one pin ~3.3 V, its partner ~0 V per channel
 *   PWM   PA2/PB0/PB8/PA3 -> ~1.65 V average at 50% duty
 * Chassis MUST stay lifted; motors spin continuously.
 */
#include "stm32f1xx_hal.h"

extern void SystemClock_Config(void);

#define PWM_PERIOD   3199U
#define HOLD_DUTY    1600U  /* 50% */

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
    if (HAL_TIM_PWM_Init(timer) != HAL_OK) Error_Handler();
}

static void pwm_channel_start(TIM_HandleTypeDef *timer, uint32_t channel) {
    TIM_OC_InitTypeDef c = {0};
    c.OCMode = TIM_OCMODE_PWM1;
    c.Pulse = 0U;
    c.OCPolarity = TIM_OCPOLARITY_HIGH;
    c.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(timer, &c, channel) != HAL_OK ||
        HAL_TIM_PWM_Start(timer, channel) != HAL_OK) {
        Error_Handler();
    }
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
    pwm_channel_start(&htim2, TIM_CHANNEL_3);  /* FL  PA2 */
    pwm_channel_start(&htim2, TIM_CHANNEL_4);  /* RR  PA3 */
    pwm_channel_start(&htim3, TIM_CHANNEL_3);  /* FR  PB0 */
    pwm_channel_start(&htim4, TIM_CHANNEL_3);  /* RL  PB8 */

    /* Forward direction on all four: (AIN2,AIN1) = (H,L) etc. */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, HOLD_DUTY);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, HOLD_DUTY);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, HOLD_DUTY);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, HOLD_DUTY);

    /* Enable both bridges. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

    for (;;) {
    }
}
