/**
 * @file main.c
 * @brief FreeRTOS application entry point.
 *
 * Task summary:
 *   CtrlTask     — 100 Hz motor PID loop (highest priority)
 *   CommTask     — UART DMA RX/TX with protocol framing
 *   SensorTask   — ToF + IMU reads at respective rates
 *   MonitorTask  — Stack watermark + error reporting (lowest priority)
 *
 * Stack sizes are in words (4 bytes on ARM Cortex-M).
 * Tune with uxTaskGetStackHighWaterMark().
 */

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* App */
#include "robot_control.h"
#include "protocol.h"

/* HAL (uncomment in real project) */
/* #include "main.h" */
/* #include "usart.h" */

/* --- Task handles --- */
static TaskHandle_t hCtrlTask    = NULL;
static TaskHandle_t hCommTask    = NULL;
static TaskHandle_t hSensorTask  = NULL;
static TaskHandle_t hMonitorTask = NULL;

/* --- Queues --- */
static QueueHandle_t xCmdQueue;   /* Received commands → CtrlTask */

/* --- UART TX buffer (written by robot_control, sent by CommTask) --- */
static uint8_t  tx_buf[PROTO_MAX_FRAME];
static uint8_t  tx_len = 0;
static SemaphoreHandle_t xTxComplete;

/* --- UART RX ring buffer --- */
#define RX_RING_SIZE 256
static uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* ======================================================================== */
/*  UART HAL Callbacks (called from ISR)                                     */
/* ======================================================================== */

/*
 * DMA RX complete / IDLE line callback.
 * Frames received byte-by-byte via DMA into ring buffer.
 * CommTask parses frames from the ring.
 *
 * void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
 *     rx_tail = (rx_tail + size) % RX_RING_SIZE;
 *     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
 *     xTaskNotifyFromISR(hCommTask, 0, eNoAction, &xHigherPriorityTaskWoken);
 *     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
 * }
 */

/*
 * Byte-level fallback if DMA IDLE not used.
 * void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *     uint8_t byte;
 *     HAL_UART_Receive_IT(huart, &byte, 1);
 *     rx_ring[rx_head] = byte;
 *     rx_head = (rx_head + 1) % RX_RING_SIZE;
 * }
 */

/* ======================================================================== */
/*  Helper: non-blocking UART send                                           */
/* ======================================================================== */

void comm_send_frame(const uint8_t *frame, uint8_t len) {
    if (xSemaphoreTake(xTxComplete, 0) == pdTRUE) {
        memcpy(tx_buf, frame, len);
        tx_len = len;
        /* HAL_UART_Transmit_DMA(&huart1, tx_buf, tx_len); -- uncomment with HAL */
        xSemaphoreGive(xTxComplete); /* Released in TX complete callback */
    }
}

uint32_t hal_get_tick_ms(void) {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/* ======================================================================== */
/*  CommTask: Parse incoming frames, send outgoing frames                    */
/* ======================================================================== */

/* Simple frame synchroniser state machine */
typedef enum {
    SYNC_WAIT_SYNC0,
    SYNC_WAIT_SYNC1,
    SYNC_READ_HEADER,
    SYNC_READ_PAYLOAD,
    SYNC_READ_CRC,
} sync_state_t;

static uint8_t rx_byte(void) {
    if (rx_head == rx_tail) return 0; /* Ring empty */
    uint8_t b = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1) % RX_RING_SIZE;
    return b;
}

static bool rx_available(void) {
    return (rx_head != rx_tail);
}

static void CommTask(void *pvParameters) {
    (void)pvParameters;

    sync_state_t state = SYNC_WAIT_SYNC0;
    uint8_t frame_buf[PROTO_MAX_FRAME];
    uint8_t frame_idx = 0;
    uint8_t exp_len   = 0;

    /* Start UART DMA reception */
    /* HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_ring, RX_RING_SIZE); */

    for (;;) {
        /* Wait for data notification from ISR, with 10ms timeout */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

        while (rx_available()) {
            uint8_t b = rx_byte();

            switch (state) {
            case SYNC_WAIT_SYNC0:
                if (b == PROTO_SYNC0) {
                    frame_buf[0] = b;
                    frame_idx = 1;
                    state = SYNC_WAIT_SYNC1;
                }
                break;

            case SYNC_WAIT_SYNC1:
                if (b == PROTO_SYNC1) {
                    frame_buf[1] = b;
                    frame_idx = 2;
                    state = SYNC_READ_HEADER;
                } else {
                    state = SYNC_WAIT_SYNC0;
                }
                break;

            case SYNC_READ_HEADER:
                frame_buf[frame_idx++] = b;
                if (frame_idx == 5) { /* SYNC0+SYNC1+LEN+SEQ+CMD */
                    exp_len = frame_buf[2];
                    if (exp_len > PROTO_MAX_PAYLOAD) {
                        state = SYNC_WAIT_SYNC0;
                    } else {
                        state = SYNC_READ_PAYLOAD;
                    }
                }
                break;

            case SYNC_READ_PAYLOAD:
                frame_buf[frame_idx++] = b;
                if (frame_idx == 5 + exp_len) {
                    state = SYNC_READ_CRC;
                }
                break;

            case SYNC_READ_CRC:
                frame_buf[frame_idx++] = b;
                if (frame_idx == 5 + exp_len + 2) {
                    /* Full frame received — decode and queue */
                    uint8_t cmd, payload[PROTO_MAX_PAYLOAD], pay_len, seq;
                    int ret = proto_decode(frame_buf, frame_idx,
                                           &cmd, payload, &pay_len, &seq);
                    if (ret >= 0) {
                        /* Send to CtrlTask (simplified: handle inline) */
                        robot_handle_command(cmd, payload, pay_len);
                    }
                    state = SYNC_WAIT_SYNC0;
                }
                break;
            }
        }
    }
}

/* ======================================================================== */
/*  CtrlTask: 100 Hz PID control loop                                        */
/* ======================================================================== */

static void CtrlTask(void *pvParameters) {
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1000 / CTRL_LOOP_HZ);

    for (;;) {
        robot_ctrl_loop();
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

/* ======================================================================== */
/*  SensorTask: ToF + IMU at respective rates                                */
/* ======================================================================== */

static void SensorTask(void *pvParameters) {
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        /* --- ToF read (20 Hz = every 50ms) --- */
        /* tof_distance_mm = vl53l0x_read_mm(); -- hardware-specific */

        /* --- IMU read (100 Hz = every 10ms) --- */
        /* mpu9250_read_quat(imu_q); */
        /* mpu9250_read_gyro(imu_gyro); */
        /* MahonyAHRSupdate(imu_gyro, imu_accel, imu_mag, imu_q); */

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

/* ======================================================================== */
/*  MonitorTask: watchdog + debug output (1 Hz)                              */
/* ======================================================================== */

static void MonitorTask(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
        /* Print stack high watermarks for tuning */
        /*
        printf("Ctrl:    %lu\n", uxTaskGetStackHighWaterMark(hCtrlTask));
        printf("Comm:    %lu\n", uxTaskGetStackHighWaterMark(hCommTask));
        printf("Sensor:  %lu\n", uxTaskGetStackHighWaterMark(hSensorTask));
        printf("Monitor: %lu\n", uxTaskGetStackHighWaterMark(hMonitorTask));
        */

        /* Blink heartbeat LED */
        /* HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); */

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ======================================================================== */
/*  main()                                                                    */
/* ======================================================================== */

int main(void) {
    /* --- HAL init (CubeMX-generated) --- */
    /* HAL_Init(); */
    /* SystemClock_Config(); */
    /* MX_GPIO_Init(); */
    /* MX_DMA_Init(); */
    /* MX_USART1_UART_Init(); */
    /* MX_TIM2_Init(); -- encoders */
    /* MX_TIM3_Init(); -- encoders + PWM */
    /* MX_TIM4_Init(); -- encoders + PWM */
    /* MX_TIM5_Init(); -- encoders */

    /* --- App init --- */
    robot_init();

    /* --- Create queues & semaphores --- */
    xCmdQueue    = xQueueCreate(8, sizeof(uint8_t) * 2); /* [cmd, payload_len] */
    xTxComplete  = xSemaphoreCreateBinary();
    xSemaphoreGive(xTxComplete);

    /* --- Create tasks --- */
    xTaskCreate(CtrlTask,    "Ctrl",   512, NULL, 4, &hCtrlTask);
    xTaskCreate(CommTask,    "Comm",   512, NULL, 3, &hCommTask);
    xTaskCreate(SensorTask,  "Sensor", 256, NULL, 2, &hSensorTask);
    xTaskCreate(MonitorTask, "Monitor",128, NULL, 1, &hMonitorTask);

    /* --- Start scheduler (never returns) --- */
    vTaskStartScheduler();

    /* Should never reach here */
    for (;;) {}
}
