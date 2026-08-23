/**
 * @file pid_step_main.c
 * @brief Single-wheel speed-PI step response rig (roadmap M2).
 *
 * Runs the production pid.c and encoder.c against the front-right wheel at
 * 100 Hz, captures a step response as a time series, and reports it through
 * GDB. Only FR is driven; the other three are left disabled.
 *
 * Gains are read from RAM at the start of each run rather than compiled in,
 * so a whole tuning sweep needs one flash: set the gains, trigger a run,
 * read the log, repeat. Reflashing per gain value would dominate the
 * iteration time and reset the counters each cycle.
 *
 *   (gdb) set var cfg_kp = 30
 *   (gdb) set var cfg_ki = 0
 *   (gdb) set var cfg_setpoint = 10
 *   (gdb) set var cmd_run = 1
 *   ... wait ~3s ...
 *   (gdb) print run_done
 *   (gdb) print log_speed
 *   (gdb) print log_pwm
 *   (gdb) print log_period_us
 *
 * The first PRE_SAMPLES samples hold setpoint 0 so the log contains the
 * step edge itself, not just what follows it.
 *
 * LIFT THE CHASSIS. Unloaded tuning only — gains will need revisiting
 * against the real load on the ground, and again on battery power (the
 * duty->speed curve is supply-voltage dependent).
 */
#include "stm32f1xx_hal.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"

extern void SystemClock_Config(void);

#define ENCODER_DEBOUNCE_CYCLES  9600U

#define PWM_PRESCALER   2U
#define PWM_PERIOD      999U

#define CTRL_HZ         100U
#define CTRL_DT_S       0.01f
#define CTRL_PERIOD_CYC 640000U   /* 10 ms at 64 MHz */

#define LOG_N           250U      /* 2.5 s at 100 Hz */
#define PRE_SAMPLES     20U       /* 200 ms of zero setpoint before the step */

#define TUNED_WHEEL     MOTOR_FR

/* --- Inputs: set these over GDB before triggering a run --- */
volatile float   cfg_kp       = 20.0f;
volatile float   cfg_ki       = 0.0f;
volatile float   cfg_kd       = 0.0f;
volatile float   cfg_setpoint = 10.0f;   /* rad/s */
volatile uint8_t cmd_run      = 0;

/* --- Outputs --- */
volatile uint8_t  run_done = 0;
volatile float    log_speed[LOG_N];      /* measured wheel speed, rad/s */
volatile int16_t  log_pwm[LOG_N];        /* commanded duty, -1000..1000 */
volatile uint16_t log_period_us[LOG_N];  /* actual loop period, for jitter */
volatile uint16_t log_count = 0;

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
 *  Motor + encoder hardware setup (same mapping as encoder_port_check)
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

/* FR sits on board 1, whose STBY is PB14. Board 2 (PC15) is never enabled. */
static void bridge_enable(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

static void bridge_disable(void) {
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
    bridge_disable();
}

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

    gpio.Mode = GPIO_MODE_IT_RISING;
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
    case GPIO_PIN_0:
        if (!debounce_ok(MOTOR_FL)) break;
        encoder_on_edge(MOTOR_FL, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET);
        break;
    case GPIO_PIN_6:
        if (!debounce_ok(MOTOR_FR)) break;
        encoder_on_edge(MOTOR_FR, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET);
        break;
    case GPIO_PIN_7:
        if (!debounce_ok(MOTOR_RL)) break;
        encoder_on_edge(MOTOR_RL, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET);
        break;
    case GPIO_PIN_12:
        if (!debounce_ok(MOTOR_RR)) break;
        encoder_on_edge(MOTOR_RR, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET);
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

/* ======================================================================== *
 *  Step response
 * ======================================================================== */

static void run_step_test(void) {
    pid_ctrl_t pid;

    /* out_max 1000 matches motor_set_duty()'s full-scale duty.
     * integral_max 1000 lets the integrator alone reach full output, which
     * it must be able to do: the startup deadband (5-10% duty) means a
     * small steady-state error has to accumulate into a large enough duty
     * to actually turn the wheel. */
    pid_init(&pid, cfg_kp, cfg_ki, cfg_kd, 1000.0f, 1000.0f, CTRL_DT_S);

    encoder_reset_all();
    (void)encoder_get_speed_rads(TUNED_WHEEL, 10);  /* baseline the sampler */

    log_count = 0;
    motor_resume();
    bridge_enable();

    uint32_t next = DWT->CYCCNT + CTRL_PERIOD_CYC;
    uint32_t prev_tick = DWT->CYCCNT;

    for (uint16_t i = 0; i < LOG_N; i++) {
        /* Busy-wait to the next control instant. Cheaper to reason about
         * than a timer ISR here, and this rig has nothing else to do. */
        while ((int32_t)(DWT->CYCCNT - next) < 0) {
        }
        uint32_t now = DWT->CYCCNT;
        next += CTRL_PERIOD_CYC;

        pid_setpoint(&pid, (i < PRE_SAMPLES) ? 0.0f : cfg_setpoint);

        float measured = encoder_get_speed_rads(TUNED_WHEEL, 1000U / CTRL_HZ);
        float out = pid_update(&pid, measured);

        int16_t duty = (int16_t)out;
        motor_set_duty(TUNED_WHEEL, duty);

        log_speed[i]     = measured;
        log_pwm[i]       = duty;
        log_period_us[i] = (uint16_t)((now - prev_tick) / 64U); /* 64 cyc/us */
        prev_tick = now;
        log_count = (uint16_t)(i + 1);
    }

    motor_emergency_stop();
    bridge_disable();
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    dwt_cycle_counter_init();
    drive_hardware_init();
    encoder_gpio_init();
    encoder_init();

    for (;;) {
        if (cmd_run) {
            run_done = 0;
            run_step_test();
            cmd_run  = 0;
            run_done = 1;
        }
    }
}
