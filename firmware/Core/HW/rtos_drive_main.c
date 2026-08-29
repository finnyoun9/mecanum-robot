/**
 * @file rtos_drive_main.c
 * @brief M4 hardware target: full FreeRTOS application on the real board.
 *
 * Brings the SIL-verified Core/Src application (CommTask UART DMA link +
 * CtrlTask 100 Hz four-wheel speed-PI + 50 Hz odometry) to silicon:
 *
 *   - TIM2/3/4 PWM at 20 kHz, TB6612 direction pins, two STBY lines
 *     (pin/timer bindings identical to the verified remote_pid_drive
 *     target — see docs/wiring.md)
 *   - Software quadrature decode on EXTI0 / EXTI9_5 / EXTI15_10
 *   - USART1 on PA9/PA10 at 921600 bit/s with DMA1 ch4 (TX) / ch5 (RX),
 *     IDLE-line reception into the Core/Src/main.c ring buffer
 *
 * Sensor (I2C) and NRF24 tasks are compiled out with HW_MINIMAL_TASKS.
 *
 * Safety state at boot: both TB6612 STBY lines are enabled, but motor.c
 * starts latched in emergency stop and the comm watchdog (100 ms deadman)
 * re-latches within the first control ticks — wheels stay coasted until
 * the Pi streams CMD_VEL_CTRL. First run: chassis lifted.
 */
#include "stm32f1xx_hal.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "motor.h"
#include "encoder.h"
#include "mpu6050.h"
#include "tof_sensor.h"
#include "ssd1306.h"

#include <stddef.h>

/* Provided by Core/Src/main.c: creates all tasks and starts the
 * scheduler (never returns). */
extern void firmware_arch_main(void);
extern void SystemClock_Config(void);

/* --- PWM/TIM configuration (identical to remote_pid_drive) --- */
#define PWM_PRESCALER          2U
#define PWM_PERIOD             999U
#define UART_BAUD              921600U
#define ENCODER_DEBOUNCE_CYCLES 9600U  /* 150 us at 64 MHz */

static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim4;

/* I2C2 on PB10/PB11 — MPU6050 (0x68) and, once fitted, VL53L0X (0x29)
 * share the bus. Not static: mpu6050.c references it by name, matching how
 * huart1 is shared with Core/Src/main.c. I2C1's pins (PB8/PB9) are
 * unavailable — PB8 is RL's PWMA. Named for the peripheral it actually
 * drives; the drivers' original placeholder comments said hi2c1. */
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef  hdma_usart1_tx;
DMA_HandleTypeDef  hdma_usart1_rx;

static volatile uint32_t last_edge_cycle[MOTOR_COUNT];

static void Error_Handler(void) {
    __disable_irq();
    for (;;) {
    }
}

/* ======================================================================== */
/*  Motor drive (verbatim pin/timer map from remote_pid_drive_main.c)        */
/* ======================================================================== */

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

    /* PWM channel pins: TIM2_CH3 PA2, TIM2_CH4 PA3, TIM3_CH3 PB0, TIM4_CH3 PB8 */
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
    if (HAL_TIM_PWM_Init(timer) != HAL_OK) Error_Handler();
}

static void pwm_channel_config(TIM_HandleTypeDef *timer, uint32_t channel) {
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(timer, &oc, channel) != HAL_OK) Error_Handler();
}

static void bridges_enable(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
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

    /* Direction order is (AIN2, AIN1): positive duty = physical forward. */
    motor_set_tim(MOTOR_FL, &htim2, GPIOA, GPIO_PIN_5, GPIOA, GPIO_PIN_4, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_FR, &htim3, GPIOA, GPIO_PIN_12, GPIOA, GPIO_PIN_11, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_RL, &htim4, GPIOB, GPIO_PIN_15, GPIOB, GPIO_PIN_1, TIM_CHANNEL_3);
    motor_set_tim(MOTOR_RR, &htim2, GPIOC, GPIO_PIN_14, GPIOC, GPIO_PIN_13, TIM_CHANNEL_4);
    motor_init();
    motor_emergency_stop();
}

/* ======================================================================== */
/*  Encoders: EXTI on channel A (both edges), B sampled for direction        */
/* ======================================================================== */

static void dwt_cycle_counter_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static bool debounce_ok(motor_id_t id) {
    uint32_t now = DWT->CYCCNT;
    if ((now - last_edge_cycle[id]) < ENCODER_DEBOUNCE_CYCLES) return false;
    last_edge_cycle[id] = now;
    return true;
}

static void encoder_gpio_init(void) {
    GPIO_InitTypeDef gpio = {0};

    /* Channel B inputs (sampled, no interrupt) */
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = GPIO_PIN_1 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Channel A interrupts on both edges:
     * FL PA0 (EXTI0), FR PA6 (EXTI9_5), RL PB7 (EXTI9_5), RR PB12 (EXTI15_10) */
    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_7 | GPIO_PIN_12;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Encoder ISRs call no FreeRTOS API — lowest urgency is fine. */
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
        if (debounce_ok(MOTOR_FL)) {
            encoder_on_edge(MOTOR_FL,
                            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET,
                            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET);
        }
        break;
    case GPIO_PIN_6:
        if (debounce_ok(MOTOR_FR)) {
            encoder_on_edge(MOTOR_FR,
                            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET,
                            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET);
        }
        break;
    case GPIO_PIN_7:
        if (debounce_ok(MOTOR_RL)) {
            encoder_on_edge(MOTOR_RL,
                            HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET,
                            HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET);
        }
        break;
    case GPIO_PIN_12:
        if (debounce_ok(MOTOR_RR)) {
            encoder_on_edge(MOTOR_RR,
                            HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET,
                            HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET);
        }
        break;
    default:
        break;
    }
}

void EXTI0_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); }
void EXTI9_5_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
}
void EXTI15_10_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12); }

/* ======================================================================== */
/*  USART1 + DMA (Pi link)                                                   */
/* ======================================================================== */

static void uart_dma_init(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* PA9 TX alternate-function push-pull; PA10 RX floating input. */
    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* STM32F103 fixed DMA map: USART1_TX = DMA1 channel 4,
     * USART1_RX = DMA1 channel 5. TX is NORMAL (one shot per frame, the
     * TxCplt callback returns the buffer semaphore); RX is CIRCULAR (the
     * RxEvent callback drains newly written bytes via CNDTR, no re-arm
     * race). The error callback re-arms RX: F1 HAL aborts the DMA channel
     * on any UART error even in circular mode. */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    /* CIRCULAR: HT/TC/IDLE events keep reception running; the RxEvent
     * callback drains only the newly written span (via CNDTR), so no
     * re-arm race can kill the receive path. */
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = UART_BAUD;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();

    /* These ISRs call xTaskNotifyFromISR / xSemaphoreGiveFromISR, so their
     * urgency must not be above configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
     * (numeric priority >= 5). */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* ======================================================================== */
/*  I2C2 — MPU6050 now, VL53L0X once fitted                                  */
/* ======================================================================== */

/*
 * Blocking master mode, no DMA and no interrupts: SensorTask runs at 20 Hz
 * and a 14-byte burst read at 400 kHz takes ~0.4 ms, so it can afford to
 * block. Keeping it off the NVIC also keeps it clear of the FreeRTOS
 * syscall priority ceiling that DMA/UART have to respect above.
 *
 * The timeouts passed by the drivers matter more than the speed here: a
 * missing or unpowered sensor makes every transaction wait out its timeout
 * inside SensorTask, so the drivers must not use HAL_MAX_DELAY.
 */
static void i2c_sensor_init(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();

    /* PB10 SCL, PB11 SDA — alternate-function OPEN DRAIN. Push-pull here
     * would fight the bus pull-ups and break clock stretching / arbitration. */
    gpio.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode  = GPIO_MODE_AF_OD;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pull  = GPIO_NOPULL;   /* module breakouts carry their own pull-ups */
    HAL_GPIO_Init(GPIOB, &gpio);

    hi2c2.Instance             = I2C2;
    /* 100 kHz is shared safely by the OLED, MPU6050 and VL53L0X on the
     * chassis' jumper-wire bus; the OLED from the OTA project is specified
     * and proven at Standard-mode only. */
    hi2c2.Init.ClockSpeed      = 100000U;
    hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1     = 0;
    hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2     = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();

    /* Hand the bus to the driver rather than having it reference a global
     * by name — see mpu6050_set_i2c()'s contract. */
    mpu6050_set_i2c(&hi2c2);
    tof_sensor_set_i2c(&hi2c2);
    ssd1306_set_i2c(&hi2c2);
}

/* ======================================================================== */
/*  FreeRTOS hooks                                                           */
/* ======================================================================== */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    /* Hard-latch both bridges off; never return. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
    __disable_irq();
    for (;;) {
    }
}

void vApplicationMallocFailedHook(void) {
    __disable_irq();
    for (;;) {
    }
}

/* ======================================================================== */
/*  Entry point                                                              */
/* ======================================================================== */

int main(void) {
    HAL_Init();
    SystemClock_Config();

    dwt_cycle_counter_init();
    drive_hardware_init();   /* PWM, direction pins, motor bindings */
    encoder_gpio_init();     /* EXTI software quadrature decode */
    /* Create the xTxComplete semaphore BEFORE the UART/DMA NVIC lines are
     * enabled: the Pi may already be streaming at reset, and an ORE before
     * the semaphore exists used to spin in configASSERT (GiveFromISR on a
     * NULL queue) before the scheduler ever started. */
    comm_create_kernel_objects();
    uart_dma_init();         /* USART1 PA9/PA10 + DMA1 ch4/ch5 */
    i2c_sensor_init();       /* I2C2 PB10/PB11 — MPU6050 */
    bridges_enable();        /* STBY high; motors still coast until commanded */

    firmware_arch_main();    /* robot_init(), tasks, scheduler — never returns */

    for (;;) {
    }
}
