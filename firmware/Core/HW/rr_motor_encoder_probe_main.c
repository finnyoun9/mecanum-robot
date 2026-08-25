/**
 * @file rr_motor_encoder_probe_main.c
 * @brief Drive only RR at 100% and compare raw A/B edges with decoded edges.
 *
 * After a two-second disabled window this drives RR forward for 1.5 seconds,
 * records PB12/PB13 independently and then drops TB6612 STBY.  It isolates
 * motor/gearbox behaviour from encoder continuity (which the passive probe
 * already checked) without involving PID or NRF24.
 */
#include "stm32f1xx_hal.h"
#include "motor.h"

extern void SystemClock_Config(void);

#define PWM_PRESCALER 2U
#define PWM_PERIOD    999U
#define DRIVE_DUTY    1000
#define DRIVE_TIME_MS 1500U

volatile uint32_t rr_a_edges;
volatile uint32_t rr_b_edges;
volatile int32_t  rr_decoded_edges;
volatile uint8_t  run_done;

static TIM_HandleTypeDef htim2;

static void Error_Handler(void) {
    __disable_irq();
    for (;;) {
    }
}

static void bridge_disable(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
}

static void drive_hardware_init(void) {
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef channel = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &gpio);

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = PWM_PRESCALER;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = PWM_PERIOD;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();

    channel.OCMode = TIM_OCMODE_PWM1;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &channel, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }

    /* Direction pins are (BIN2, BIN1), so positive duty is chassis-forward. */
    motor_set_tim(MOTOR_RR, &htim2, GPIOC, GPIO_PIN_14, GPIOC, GPIO_PIN_13,
                  TIM_CHANNEL_4);
    motor_init();
    motor_emergency_stop();
    bridge_disable();
}

static void encoder_gpio_init(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_12) {
        bool a_level = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET;
        bool b_level = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET;
        rr_a_edges++;
        if (a_level == b_level) rr_decoded_edges++; else rr_decoded_edges--;
    }
    if (GPIO_Pin == GPIO_PIN_13) rr_b_edges++;
}

void EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    drive_hardware_init();
    encoder_gpio_init();

    HAL_Delay(2000U);
    motor_resume();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    motor_set_duty(MOTOR_RR, DRIVE_DUTY);
    HAL_Delay(DRIVE_TIME_MS);
    motor_emergency_stop();
    bridge_disable();
    run_done = 1U;

    for (;;) {
    }
}
