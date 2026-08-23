/**
 * @file encoder_port_check_main.c
 * @brief Validates the production encoder.c on real hardware after its
 *        port from hardware-timer mode to software EXTI decode.
 *
 * encoder_debug proved the decode *approach* works, but it carried its own
 * private counters. This target instead links the real firmware/Core/Src/
 * encoder.c — the same object the FreeRTOS application uses — and drives it
 * through encoder_on_edge() from the EXTI handlers, exactly as the
 * application will. Unit tests and SIL cover the logic; this covers the
 * thing they cannot: that the ported module behaves on the actual robot.
 *
 * Drives all four wheels at 40% for 1.5s, then records both APIs the
 * control loop depends on:
 *   - encoder_get_count()      cumulative edges
 *   - encoder_get_speed_rads() angular velocity
 *
 * Expected from the duty sweep at 40% (docs/hardware-closed-loop-roadmap.md):
 * ~366 edges/s, ~10.25 rad/s, so ~550 counts over the 1.5s drive. Counts
 * near encoder_debug's earlier 619-640 and speeds near 10 rad/s mean the
 * port preserved behaviour.
 *
 * LIFT THE CHASSIS before flashing.
 *
 * Read results (let it free-run ~8s first — st-util resets on attach):
 *   (gdb) print final_count
 *   (gdb) print final_speed
 *   (gdb) print check_done
 */
#include "stm32f1xx_hal.h"
#include "motor.h"
#include "encoder.h"

extern void SystemClock_Config(void);

#define ENCODER_DEBOUNCE_CYCLES  9600U

#define PWM_PRESCALER   2U
#define PWM_PERIOD      999U
#define DRIVE_DUTY      400U
#define DRIVE_RUN_MS   1500U
#define SPEED_WINDOW_MS 200U

/* Results, read via GDB. */
volatile int32_t final_count[MOTOR_COUNT];
volatile float   final_speed[MOTOR_COUNT];
volatile uint8_t check_done = 0;

static volatile uint32_t last_cycle[MOTOR_COUNT];

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

static bool debounce_ok(motor_id_t id) {
    uint32_t now = DWT->CYCCNT;
    if ((now - last_cycle[id]) < ENCODER_DEBOUNCE_CYCLES) {
        return false;
    }
    last_cycle[id] = now;
    return true;
}

/* ======================================================================== *
 *  Motor init — same mapping as drive_control
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

/* ======================================================================== *
 *  Encoder EXTI — routes edges into the production encoder.c
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
    case GPIO_PIN_0: /* FL: A = PA0, B = PA1 */
        if (!debounce_ok(MOTOR_FL)) break;
        encoder_on_edge(MOTOR_FL, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET,
                        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET);
        break;
    case GPIO_PIN_6: /* FR: A = PA6, B = PA7 */
        if (!debounce_ok(MOTOR_FR)) break;
        encoder_on_edge(MOTOR_FR, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET,
                        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET);
        break;
    case GPIO_PIN_7: /* RL: A = PB7, B = PB6 */
        if (!debounce_ok(MOTOR_RL)) break;
        encoder_on_edge(MOTOR_RL, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET,
                        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET);
        break;
    case GPIO_PIN_12: /* RR: A = PB12, B = PB13 */
        if (!debounce_ok(MOTOR_RR)) break;
        encoder_on_edge(MOTOR_RR, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET,
                        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET);
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

int main(void) {
    HAL_Init();
    SystemClock_Config();
    dwt_cycle_counter_init();
    drive_hardware_init();
    encoder_gpio_init();
    encoder_init();

    HAL_Delay(2000U);

    motor_resume();
    for (uint32_t wheel = 0U; wheel < MOTOR_COUNT; wheel++) {
        motor_set_duty((motor_id_t)wheel, (int16_t)DRIVE_DUTY);
    }
    bridges_enable();

    /* Baseline the speed sampler once the wheels are up to speed, so the
     * reading below covers a steady-state window rather than spin-up. */
    HAL_Delay(DRIVE_RUN_MS - SPEED_WINDOW_MS);
    for (uint32_t wheel = 0U; wheel < MOTOR_COUNT; wheel++) {
        (void)encoder_get_speed_rads((motor_id_t)wheel, SPEED_WINDOW_MS);
    }
    HAL_Delay(SPEED_WINDOW_MS);
    for (uint32_t wheel = 0U; wheel < MOTOR_COUNT; wheel++) {
        final_speed[wheel] = encoder_get_speed_rads((motor_id_t)wheel, SPEED_WINDOW_MS);
    }

    motor_emergency_stop();
    bridges_disable();

    for (uint32_t wheel = 0U; wheel < MOTOR_COUNT; wheel++) {
        final_count[wheel] = encoder_get_count((motor_id_t)wheel);
    }
    check_done = 1U;

    for (;;) {
    }
}
