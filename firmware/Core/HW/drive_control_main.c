/**
 * @file drive_control_main.c
 * @brief Hardware entry point for open-loop four-wheel mecanum control.
 *
 * All commands use the verified motor order: FL, FR, RL, RR. The bridge is
 * disabled by default and after the short boot demonstration.
 */
#include "stm32f1xx_hal.h"
#include "motor.h"

extern void SystemClock_Config(void);

#define PWM_PRESCALER      2U
#define PWM_PERIOD         999U  /* 64 MHz / 3 / 1000 = 21.3 kHz */
#define DEMO_DUTY          700U  /* 70%; matches the verified four-wheel test */
#define DEMO_RUN_MS        1500U
#define DEMO_STOP_MS        600U

typedef enum {
    DRIVE_STOP = 0,
    DRIVE_FORWARD,
    DRIVE_REVERSE,
    DRIVE_LEFT,
    DRIVE_RIGHT,
    DRIVE_CCW,
    DRIVE_CW,
} drive_command_t;

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

static void bridge_disable(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
}

static void bridge_enable(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
}

static void drive_stop(void) {
    motor_emergency_stop();
    bridge_disable();
}

static void drive_apply(const int16_t wheel_duty[MOTOR_COUNT]) {
    bridge_disable();
    motor_resume();
    for (uint32_t wheel = 0U; wheel < MOTOR_COUNT; wheel++) {
        motor_set_duty((motor_id_t)wheel, wheel_duty[wheel]);
    }
    bridge_enable();
}

static void drive_command(drive_command_t command, uint16_t duty) {
    const int16_t speed = (int16_t)(duty > 1000U ? 1000U : duty);
    int16_t wheel_duty[MOTOR_COUNT] = {0};

    switch (command) {
    case DRIVE_FORWARD:
        wheel_duty[MOTOR_FL] = speed;  wheel_duty[MOTOR_FR] = speed;
        wheel_duty[MOTOR_RL] = speed;  wheel_duty[MOTOR_RR] = speed;
        break;
    case DRIVE_REVERSE:
        wheel_duty[MOTOR_FL] = -speed; wheel_duty[MOTOR_FR] = -speed;
        wheel_duty[MOTOR_RL] = -speed; wheel_duty[MOTOR_RR] = -speed;
        break;
    case DRIVE_LEFT:
        wheel_duty[MOTOR_FL] = -speed; wheel_duty[MOTOR_FR] = speed;
        wheel_duty[MOTOR_RL] = speed;  wheel_duty[MOTOR_RR] = -speed;
        break;
    case DRIVE_RIGHT:
        wheel_duty[MOTOR_FL] = speed;  wheel_duty[MOTOR_FR] = -speed;
        wheel_duty[MOTOR_RL] = -speed; wheel_duty[MOTOR_RR] = speed;
        break;
    case DRIVE_CCW:
        wheel_duty[MOTOR_FL] = -speed; wheel_duty[MOTOR_FR] = speed;
        wheel_duty[MOTOR_RL] = -speed; wheel_duty[MOTOR_RR] = speed;
        break;
    case DRIVE_CW:
        wheel_duty[MOTOR_FL] = speed;  wheel_duty[MOTOR_FR] = -speed;
        wheel_duty[MOTOR_RL] = speed;  wheel_duty[MOTOR_RR] = -speed;
        break;
    case DRIVE_STOP:
    default:
        drive_stop();
        return;
    }
    drive_apply(wheel_duty);
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

    motor_set_tim(MOTOR_FL, &htim2, GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_5, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_FR, &htim3, GPIOA, GPIO_PIN_11, GPIOA, GPIO_PIN_12, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_RL, &htim4, GPIOB, GPIO_PIN_1, GPIOB, GPIO_PIN_15, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_RR, &htim2, GPIOC, GPIO_PIN_13, GPIOC, GPIO_PIN_14, TIM_CHANNEL_4);
    motor_init();
    drive_stop();
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    drive_hardware_init();

    HAL_Delay(2000U);
    drive_command(DRIVE_FORWARD, DEMO_DUTY);
    HAL_Delay(DEMO_RUN_MS);
    drive_command(DRIVE_STOP, 0U);
    HAL_Delay(DEMO_STOP_MS);
    drive_command(DRIVE_LEFT, DEMO_DUTY);
    HAL_Delay(DEMO_RUN_MS);
    drive_command(DRIVE_STOP, 0U);
    HAL_Delay(DEMO_STOP_MS);
    drive_command(DRIVE_CCW, DEMO_DUTY);
    HAL_Delay(DEMO_RUN_MS);
    drive_command(DRIVE_STOP, 0U);

    for (;;) {
    }
}
