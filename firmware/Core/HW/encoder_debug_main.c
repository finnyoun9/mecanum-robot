/**
 * @file encoder_debug_main.c
 * @brief Real-hardware smoke test: drive all 4 wheels briefly, decode all
 *        4 quadrature encoders in software, and verify every counter moved.
 *
 * IMPORTANT — corrects an earlier design error: docs/wiring.md originally
 * said 3 motors (FL/FR/RL) would use STM32 hardware TIM encoder mode
 * (TI1+TI2) and only the 4th (RR) would need software decode. That's
 * wrong: TIM2/TIM3/TIM4 are already fully committed to PWM generation
 * (see drive_control_main.c — TIM2 CH3/CH4, TIM3 CH3, TIM4 CH3), and a
 * timer's counter can't simultaneously free-run for PWM timing AND be
 * clocked by external quadrature edges. With TIM1's encoder channels
 * unusable (PA8/PA9 taken by NRF24L01 CE / USART1), there is no timer
 * left for hardware encoder mode on ANY motor. All 4 need software EXTI
 * decode. The wiring itself (PA0/PA1, PA6/PA7, PB6/PB7, PB12/PB13) is
 * unaffected — this is a firmware-only correction.
 *
 * EXTI is a single chip-wide mux per pin NUMBER (not per port): only one
 * port's PA6 *or* PB6 *or* PC6 can own EXTI line 6 at a time. FR's pins
 * (PA6/PA7) and RL's pins (PB6/PB7) collide on numbers 6 and 7. Fixed by
 * choosing PB7 (not PB6) as RL's interrupt-driven channel — the four
 * interrupt-driven pins end up on lines {0, 6, 7, 12}, all distinct.
 * Non-interrupt "read the level" companion pins don't touch EXTI at all,
 * so they can share numbers freely.
 *
 * | Motor | IRQ-driven (channel A) | Plain read (channel B) |
 * |-------|------------------------|--------------------------|
 * | FL    | PA0  (EXTI0)           | PA1                      |
 * | FR    | PA6  (EXTI9_5)         | PA7                      |
 * | RL    | PB7  (EXTI9_5)         | PB6                      |
 * | RR    | PB12 (EXTI15_10)       | PB13                     |
 *
 * Verified on hardware 2026-08-23, chassis lifted, 40% duty for 1.5s:
 * FL 631, FR 633, RL 640, RR 619 — all positive, spread ~3%, with the
 * robot physically moving forward. Two calibration fixes got it there:
 * the AIN1/AIN2 swap in drive_hardware_init() (positive duty now really
 * means forward), and swapping FL's two encoder wires in the harness
 * (its count used to run negative while the others ran positive).
 *
 * A note on measuring this: st-util resets the target both when it starts
 * and again when GDB attaches. Halting immediately after attach therefore
 * samples a chip that restarted milliseconds ago, still inside the 2s
 * startup delay — which reads as "encoders stuck near zero while the
 * wheels visibly spin". Let it free-run several seconds after attaching,
 * then interrupt, e.g.:
 *   gdb -ex "target extended-remote :4242" -ex continue ... &
 *   sleep 8; kill -INT %1
 */
#include "stm32f1xx_hal.h"
#include "motor.h"

extern void SystemClock_Config(void);

/* Debounce window in CPU cycles, measured with DWT->CYCCNT rather than
 * HAL_GetTick(): the cycle counter free-runs off the CPU clock and is
 * unaffected by interrupt load, so it stays trustworthy even if this ISR
 * is firing hard. 150us at 64MHz = 9600 cycles — rejects short glitches
 * while passing genuine encoder edges at any speed these motors reach.
 * Kept as cheap insurance against contact bounce and pickup on the
 * breadboard harness; the wheels count cleanly with it in place. */
#define ENCODER_DEBOUNCE_CYCLES  9600U

#define PWM_PRESCALER   2U
#define PWM_PERIOD      999U
#define DRIVE_DUTY      400U   /* 40% — modest speed, plenty for encoder counts */
#define DRIVE_RUN_MS    1500U

static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim4;

/* Last accepted edge's DWT->CYCCNT value, per motor — for debouncing. */
static volatile uint32_t fl_last_cycle = 0;
static volatile uint32_t fr_last_cycle = 0;
static volatile uint32_t rl_last_cycle = 0;
static volatile uint32_t rr_last_cycle = 0;

static void dwt_cycle_counter_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* True if enough cycles passed since *last_cycle to accept this edge as
 * real (not noise); updates *last_cycle when it does. */
static bool debounce_ok(volatile uint32_t *last_cycle) {
    uint32_t now = DWT->CYCCNT;
    if ((now - *last_cycle) < ENCODER_DEBOUNCE_CYCLES) {
        return false;
    }
    *last_cycle = now;
    return true;
}

/* Cumulative software-decoded counts. Read these via GDB after halting:
 *   (gdb) print fl_count
 *   (gdb) print fr_count / rl_count / rr_count
 */
volatile int32_t fl_count = 0;
volatile int32_t fr_count = 0;
volatile int32_t rl_count = 0;
volatile int32_t rr_count = 0;

static void Error_Handler(void) {
    __disable_irq();
    for (;;) {
    }
}

/* ======================================================================== *
 *  Motor GPIO/PWM init — identical to drive_control_main.c's verified setup
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

static void bridge_disable(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
}

/* Drive all four wheels forward at the given duty.
 *
 * No sign flip here any more: the AIN1/AIN2 swap in drive_hardware_init()
 * makes a positive duty produce real forward motion, so this reads the
 * way it should. Encoder signs recorded by this test are therefore the
 * signs that correspond to genuine forward travel. */
static void drive_all_forward(uint16_t duty) {
    motor_resume();
    for (uint32_t wheel = 0U; wheel < MOTOR_COUNT; wheel++) {
        motor_set_duty((motor_id_t)wheel, (int16_t)duty);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); /* board 1 STBY on */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); /* board 2 STBY on */
}

static void drive_stop(void) {
    motor_emergency_stop();
    bridge_disable();
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

    /* DIR pairs passed swapped (AIN2, AIN1) so positive duty == forward;
     * see drive_control_main.c for the hardware verification note. */
    motor_set_tim(MOTOR_FL, &htim2, GPIOA, GPIO_PIN_5, GPIOA, GPIO_PIN_4, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_FR, &htim3, GPIOA, GPIO_PIN_12, GPIOA, GPIO_PIN_11, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_RL, &htim4, GPIOB, GPIO_PIN_15, GPIOB, GPIO_PIN_1, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_RR, &htim2, GPIOC, GPIO_PIN_14, GPIOC, GPIO_PIN_13, TIM_CHANNEL_4);
    motor_init();
    drive_stop();
}

/* ======================================================================== *
 *  Encoder EXTI init
 * ======================================================================== */

static void encoder_gpio_init(void) {
    GPIO_InitTypeDef gpio = {0};

    /* Plain reads (channel B) — floating input is fine, we only sample level. */
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP; /* in case the encoder output is open-collector */

    gpio.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* IRQ-driven channel A pins — RISING edge only (halves the interrupt
     * rate vs. both-edges, at the cost of resolution; fine for this test). */
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

    /* Priority 15 = lowest tier, same as SysTick — deliberately not higher.
     * An encoder tick is never more urgent than the system timebase, so
     * even a misbehaving line can't starve HAL's millisecond counter. */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* Dispatched by pin NUMBER only (HAL doesn't tell us the port) — safe here
 * because the four IRQ-driven pins were deliberately chosen so no two of
 * them share a number (0, 6, 7, 12).
 *
 * Only channel A's rising edge fires this now, so channel B's level at
 * that instant is a consistent, unambiguous direction indicator — no need
 * to also read A (which an earlier both-edges version needed, since B's
 * level flips between A's rising and falling edges). */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    switch (GPIO_Pin) {
    case GPIO_PIN_0: /* FL channel A = PA0, channel B = PA1 */
        if (!debounce_ok(&fl_last_cycle)) break;
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) fl_count++; else fl_count--;
        break;
    case GPIO_PIN_6: /* FR channel A = PA6, channel B = PA7 */
        if (!debounce_ok(&fr_last_cycle)) break;
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET) fr_count++; else fr_count--;
        break;
    case GPIO_PIN_7: /* RL channel A = PB7, channel B = PB6 */
        if (!debounce_ok(&rl_last_cycle)) break;
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) rl_count++; else rl_count--;
        break;
    case GPIO_PIN_12: /* RR channel A = PB12, channel B = PB13 */
        if (!debounce_ok(&rr_last_cycle)) break;
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET) rr_count++; else rr_count--;
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
 *  Main
 * ======================================================================== */

int main(void) {
    HAL_Init();
    SystemClock_Config();
    dwt_cycle_counter_init();
    drive_hardware_init();
    encoder_gpio_init();

    HAL_Delay(2000U);
    drive_all_forward(DRIVE_DUTY);
    HAL_Delay(DRIVE_RUN_MS);
    drive_stop();

    /* Halt here — attach GDB and read fl_count/fr_count/rl_count/rr_count. */
    for (;;) {
    }
}
