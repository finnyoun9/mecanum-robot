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

#include <string.h>

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* App */
#include "robot_control.h"
#include "protocol.h"
#include "ahrs.h"
#include "mpu6050.h"
#include "tof_sensor.h"
#include "ssd1306.h"
#include "oled_ui.h"
#include "nrf24l01.h"
#include "remote_control.h"

/* HAL */
#include "main.h"
#include "usart.h"

/* --- Task handles --- */
static TaskHandle_t hCtrlTask    = NULL;
static TaskHandle_t hCommTask    = NULL;
#ifndef HW_NO_SENSOR_TASK
static TaskHandle_t hSensorTask  = NULL;
#endif
#ifndef HW_MINIMAL_TASKS
static TaskHandle_t hRemoteTask  = NULL;
#endif
static TaskHandle_t hMonitorTask = NULL;

/* --- Queues --- */
static QueueHandle_t xCmdQueue;   /* Received commands → CtrlTask */

/* --- UART TX buffer (written by robot_control, sent by CommTask) --- */
static uint8_t  tx_buf[PROTO_MAX_FRAME];
static uint8_t  tx_len = 0;
/* Binary semaphore: available = no DMA TX in flight. Taken in
 * comm_send_frame(), given back ONLY in HAL_UART_TxCpltCallback(). */
SemaphoreHandle_t xTxComplete;

/* --- UART RX: DMA staging buffer + software ring (separate objects) ---
 * The DMA controller writes only into rx_stage; the RX-event callback
 * copies each completed chunk into rx_ring (the parser's buffer) and
 * updates the write pointer (rx_head). CommTask only ever reads the ring,
 * so DMA and the parser never race on the same memory. */
#define RX_RING_SIZE   256
#define RX_STAGE_SIZE  64
static uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;       /* ISR writes, task reads */
static volatile uint16_t rx_tail = 0;       /* task writes, ISR reads */
static volatile uint16_t rx_overflows = 0;  /* dropped bytes when ring full */

static uint8_t rx_stage[RX_STAGE_SIZE];     /* DMA staging buffer */

#ifndef SIL_BUILD
/* Circular-RX write position drained into rx_ring on the last RxEvent
 * callback (index into rx_stage). Zero at every (re)arm. */
static volatile uint16_t rx_dma_last_pos;
#endif

/** After an error-path abort/re-arm, circular RX restarts at rx_stage[0];
 * re-sync the drain position so the next callback does not replay bytes. */
void comm_rx_dma_resync(void) {
#ifndef SIL_BUILD
    rx_dma_last_pos = 0;
#endif
}

/**
 * @brief Feed bytes into the UART RX ring (for SIL / host testing).
 *
 * In production, HAL_UARTEx_RxEventCallback() (below) fills the ring from
 * the DMA staging buffer.  In SIL, the test harness calls this directly to
 * simulate received protocol frames.
 */
void sil_uart_rx_feed(const uint8_t *data, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((rx_head + 1) % RX_RING_SIZE);
        if (next == rx_tail) break; /* ring full */
        rx_ring[rx_head] = data[i];
        rx_head = next;
    }
}

/** Telemetry hook: count of RX bytes dropped due to a full ring. */
uint16_t comm_rx_overflows(void) {
    return rx_overflows;
}

/* ======================================================================== */
/*  UART HAL Callbacks (called from ISR)                                     */
/* ======================================================================== */

/**
 * DMA RX chunk complete (IDLE line or staging buffer full).
 *
 * Copies the freshly staged bytes into the software ring, updates the
 * write pointer, wakes CommTask, and re-arms DMA on the staging buffer.
 * Kept short: bounded copy of at most RX_STAGE_SIZE bytes.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    if (huart != &huart1) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

#ifdef SIL_BUILD
    /* The SIL mock delivers exact chunk sizes and models no DMA wrap. */
    if (size > 0) {
        for (uint16_t i = 0; i < size; i++) {
            uint16_t next = (uint16_t)((rx_head + 1) % RX_RING_SIZE);
            if (next == rx_tail) {
                rx_overflows++;           /* ring full: drop this byte */
            } else {
                rx_ring[rx_head] = rx_stage[i];
                rx_head = next;
            }
        }
        if (hCommTask != NULL) {
            xTaskNotifyFromISR(hCommTask, 0, eNoAction, &xHigherPriorityTaskWoken);
        }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_stage, RX_STAGE_SIZE);
#else
    /* Hardware: the RX DMA runs in CIRCULAR mode, so HT/TC/IDLE events do
     * NOT stop reception — there is no re-arm here (re-arming inside this
     * callback while RxState is still BUSY_RX returns HAL_BUSY and silently
     * kills RX until the next overrun; that race corrupted every frame
     * straddling the staging-buffer boundary). Instead, derive the span of
     * freshly written bytes from the DMA counter and drain just that span,
     * wrapping at the end of the staging buffer. */
    uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
    /* CNDTR reads 0 in the single-cycle reload window at TC in circular
     * mode; treat that as the wrapped position (drain through end of
     * stage) instead of computing pos == RX_STAGE_SIZE, which would make
     * the cursor loop never terminate. */
    uint16_t pos = (remaining == 0U) ? 0U
                                     : (uint16_t)(RX_STAGE_SIZE - remaining);
    uint16_t cursor = rx_dma_last_pos;
    bool drained = false;

    while (cursor != pos) {
        uint16_t next = (uint16_t)((rx_head + 1) % RX_RING_SIZE);
        if (next == rx_tail) {
            rx_overflows++;               /* ring full: drop this byte */
        } else {
            rx_ring[rx_head] = rx_stage[cursor];
            rx_head = next;
        }
        cursor = (uint16_t)((cursor + 1) % RX_STAGE_SIZE);
        drained = true;
    }
    rx_dma_last_pos = pos;

    /* hCommTask is NULL until firmware_arch_main() creates CommTask; an
     * HT/TC event before that (e.g. RX re-armed from the error callback
     * during boot) must not call xTaskNotifyFromISR(NULL). */
    if (drained && (hCommTask != NULL)) {
        xTaskNotifyFromISR(hCommTask, 0, eNoAction, &xHigherPriorityTaskWoken);
    }
#endif

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* UART error events observed in HAL_UART_ErrorCallback (ORE/FE/NE/PE/DMA).
 * Readable via GDB / future telemetry; proves link recovery in the flood
 * test rather than a silent lock-up. */
static volatile uint32_t comm_uart_errors;

uint32_t comm_uart_error_count(void) {
    return comm_uart_errors;
}

/* True while a TX DMA transfer owns the wire. On hardware the HAL gState
 * tracks this; the SIL mock exposes the same fact as dma_tx_active. */
static bool uart_tx_in_flight(UART_HandleTypeDef *huart) {
#ifdef SIL_BUILD
    return huart->dma_tx_active;
#else
    return huart->gState == HAL_UART_STATE_BUSY_TX;
#endif
}

/**
 * DMA TX complete: the tx_buf frame has been fully shifted out.
 * Return the buffer semaphore here — this is the ONLY routine place it is
 * given back, so comm_send_frame() can never overwrite a frame in flight.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart != &huart1) return;

    /* xTxComplete is NULL until comm_create_kernel_objects() runs; the
     * UART NVIC line is enabled before that on the hardware target. */
    if (xTxComplete == NULL) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xTxComplete, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * UART error (ORE overrun, FE framing, NE noise, PE parity, DMA error).
 *
 * STM32F1 HAL treats ANY error while DMA reception is enabled as blocking:
 * HAL_UART_IRQHandler() has ALREADY disabled DMAR, aborted the RX DMA
 * channel (UART_DMAAbortOnError invokes this callback synchronously) and
 * restored RxState to READY. Circular DMA does NOT restart itself, so RX
 * MUST be re-armed here — otherwise the link goes deaf until reboot.
 *
 * TX is deliberately NOT aborted: ORE/FE/NE are receive-side errors; an
 * in-flight TX completes normally and returns xTxComplete via
 * HAL_UART_TxCpltCallback(). The semaphore is released here only when no
 * TX is active (e.g. a DMA transfer-error already ended the TX path):
 * giving it while a transfer runs would let comm_send_frame() overwrite
 * tx_buf while the DMA engine is still reading it.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart != &huart1) return;

    comm_uart_errors++;

    /* Clear latched ORE/FE/NE/PE (on F1 they clear by reading SR then DR)
     * BEFORE EIE is re-enabled: the HAL error path never reads DR, so the
     * flags are still set here. Without this, the re-enabled error IRQ
     * re-enters this handler immediately in a tight interrupt storm.
     * The corrupting byte in DR is discarded. */
#ifndef SIL_BUILD
    __HAL_UART_CLEAR_OREFLAG(huart);
#endif

    /* Re-arm circular RX. The restarted channel writes rx_stage from
     * index 0, so the NDTR-derived drain cursor must re-sync too. */
    comm_rx_dma_resync();
    (void)HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_stage, RX_STAGE_SIZE);

    if ((xTxComplete != NULL) && !uart_tx_in_flight(huart)) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xTxComplete, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* ======================================================================== */
/*  Helper: non-blocking UART send                                           */
/* ======================================================================== */

void comm_send_frame(const uint8_t *frame, uint8_t len) {
    if (len > PROTO_MAX_FRAME) return;

    /* Non-blocking take: if the previous DMA transfer is still in flight,
     * drop this frame rather than overwrite tx_buf under the DMA engine.
     * The semaphore comes back only in HAL_UART_TxCpltCallback(). */
    if (xSemaphoreTake(xTxComplete, 0) != pdTRUE) return;

    memcpy(tx_buf, frame, len);
    tx_len = len;
    HAL_UART_Transmit_DMA(&huart1, tx_buf, tx_len);
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

void CommTask(void *pvParameters) {
    (void)pvParameters;

    static sync_state_t state = SYNC_WAIT_SYNC0;
    static uint8_t frame_buf[PROTO_MAX_FRAME];
    static uint8_t frame_idx = 0;
    static uint8_t exp_len   = 0;
    static bool initialized = false;

    if (!initialized) {
        /* Arm DMA reception on the staging buffer. Chunks are copied into
         * the software ring by HAL_UARTEx_RxEventCallback(). */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_stage, RX_STAGE_SIZE);
        initialized = true;
    }

#ifndef SIL_BUILD
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
#endif
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
                    } else if (exp_len == 0) {
                        /* Zero-length payload (heartbeat/ACK): the PAYLOAD
                         * state appends before comparing, so it can never
                         * reach frame_idx == 5 — skip straight to CRC. */
                        state = SYNC_READ_CRC;
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
#ifndef SIL_BUILD
    }
#endif
}

/* ======================================================================== */
/*  CtrlTask: 100 Hz PID control loop                                        */
/* ======================================================================== */

void CtrlTask(void *pvParameters) {
    (void)pvParameters;

#ifndef SIL_BUILD
    /* FreeRTOS periodic-scheduling state. Compiled out in SIL builds,
     * where the cooperative harness drives one iteration per tick. */
    static TickType_t xLastWakeTime;
    static bool initialized = false;
    const TickType_t xPeriod = pdMS_TO_TICKS(1000 / CTRL_LOOP_HZ);

    if (!initialized) {
        xLastWakeTime = xTaskGetTickCount();
        initialized = true;
    }
#endif

#ifndef SIL_BUILD
    for (;;) {
#endif
        robot_ctrl_loop();
#ifndef SIL_BUILD
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
#endif
}

/* ======================================================================== */
/*  SensorTask: ToF + IMU at respective rates                                */
/* ======================================================================== */

#ifdef HW_OLED
static uint8_t oled_frame[SSD1306_FRAME_BYTES];

/* SensorTask samples ToF every 50 ms. Five seconds is long enough to read a
 * page on the small panel without turning the OLED into a distracting ticker. */
#define OLED_PAGE_HOLD_TICKS 100U

static int16_t oled_scaled_i16(float value, float scale) {
    float scaled = value * scale;
    if (scaled > 32767.0f) return INT16_MAX;
    if (scaled < -32768.0f) return INT16_MIN;
    return (int16_t)scaled;
}

static void oled_render_state(uint8_t page) {
    const robot_state_t *state = robot_get_state();
    oled_ui_data_t data = {0};

    data.tof_mm = state->tof_distance_mm;
    data.error_flags = state->error_flags;
    data.comm_ok = !state->comm_timeout;
    data.emergency_stop = state->emergency_stop_active;
    data.qw_centi = oled_scaled_i16(state->imu_q[0], 100.0f);
    for (uint8_t i = 0; i < 4U; ++i) {
        data.target_deci_rads[i] = oled_scaled_i16(state->target_w[i], 10.0f);
        data.measured_deci_rads[i] = oled_scaled_i16(state->measured_w[i], 10.0f);
    }
    for (uint8_t i = 0; i < 3U; ++i) {
        data.gyro_milli_rads[i] = oled_scaled_i16(state->imu_gyro[i], 1000.0f);
    }
    oled_ui_render(oled_frame, page, &data);
}
#endif

void SensorTask(void *pvParameters) {
    (void)pvParameters;

    static float imu_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
#ifndef HW_IMU_ONLY
    static uint8_t tof_divider = 0;
#endif
#ifdef HW_OLED
    static uint8_t oled_page = 0;
    static uint8_t oled_frames_on_page = 0;
    static bool oled_ok = false;
#endif
    static bool initialized = false;
    const float xImuDt = 0.010f; /* matches this task's 10ms period */

#ifndef SIL_BUILD
    static TickType_t xLastWakeTime;
#endif

    if (!initialized) {
#ifndef SIL_BUILD
        xLastWakeTime = xTaskGetTickCount();
#endif
        mpu6050_init();
#ifndef HW_IMU_ONLY
        tof_init();
#endif
#ifdef HW_OLED
        oled_ok = ssd1306_init();
        if (oled_ok) {
            oled_render_state(oled_page);
            oled_ok = ssd1306_write_frame(oled_frame, sizeof(oled_frame));
        }
#endif
        initialized = true;
    }

#ifndef SIL_BUILD
    for (;;) {
#endif
        /* --- IMU read + AHRS update (100 Hz = every 10ms) --- */
        int16_t accel_raw[3], gyro_raw[3];
        float   accel_mps2[3], gyro_rads[3];

        mpu6050_read_raw(accel_raw, gyro_raw);
        mpu6050_convert_units(accel_raw, gyro_raw, accel_mps2, gyro_rads);
        MahonyAHRSupdateIMU(gyro_rads[0], gyro_rads[1], gyro_rads[2],
                             accel_mps2[0], accel_mps2[1], accel_mps2[2],
                             imu_q, xImuDt);
        robot_update_imu(imu_q, gyro_rads);

        /* --- ToF read (20 Hz = every 5th iteration / 50ms) ---
         * Compiled out under HW_IMU_ONLY: tof_sensor.c is still a stub whose
         * every read times out, which would pin ERR_TOF_TIMEOUT in the
         * odometry error_flags the Pi sees and make a real fault
         * indistinguishable from the placeholder. Drop the guard once the
         * VL53L0X is wired and its I2C calls are live. */
#ifndef HW_IMU_ONLY
        if (++tof_divider >= 5) {
            tof_divider = 0;
            tof_status_t tof_status;
            uint16_t tof_mm = tof_read_mm(&tof_status);
            robot_update_tof(tof_mm, tof_status == TOF_TIMEOUT);
#ifdef HW_OLED
            if (oled_ok) {
                /* One contiguous transfer avoids a visible 400 ms cascade
                 * of eight page writes. Refreshing only when changing page
                 * also prevents the status text from overlapping itself. */
                if (++oled_frames_on_page >= OLED_PAGE_HOLD_TICKS) {
                    oled_frames_on_page = 0;
                    oled_page = (uint8_t)((oled_page + 1U) % OLED_UI_PAGE_COUNT);
                    oled_render_state(oled_page);
                    oled_ok = ssd1306_write_frame(oled_frame, sizeof(oled_frame));
                }
            }
#endif
        }
#endif

#ifndef SIL_BUILD
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
#endif
}

/* ======================================================================== */
/*  RemoteTask: poll NRF24L01 (20 Hz), apply wireless remote control         */
/* ======================================================================== */

void RemoteTask(void *pvParameters) {
    (void)pvParameters;

    static remote_state_t rstate;
    static bool initialized = false;

    if (!initialized) {
        remote_init(&rstate);
        nrf24l01_init();
        initialized = true;
    }

#ifndef SIL_BUILD
    for (;;) {
#endif
        if (nrf24l01_receive()) {
            remote_result_t res;
            if (remote_process(nrf24l01_rx_packet(), &rstate, &res)) {
                if (res.key == REMOTE_KEY_ESTOP) {
                    robot_emergency_stop();
                }
                if (rstate.enabled) {
                    robot_set_target_wheels(res.wheel_speed);
                } else if (res.key == REMOTE_KEY_TOGGLE_ENABLE) {
                    /* Just disabled — zero the last remote speeds. */
                    float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                    robot_set_target_wheels(zero);
                }
            }
        }
#ifndef SIL_BUILD
        vTaskDelay(pdMS_TO_TICKS(50));
    }
#endif
}

/* ======================================================================== */
/*  MonitorTask: watchdog + debug output (1 Hz)                              */
/* ======================================================================== */

void MonitorTask(void *pvParameters) {
    (void)pvParameters;

#ifndef SIL_BUILD
    for (;;) {
#endif
        /* Print stack high watermarks for tuning */
        /*
        printf("Ctrl:    %lu\n", uxTaskGetStackHighWaterMark(hCtrlTask));
        printf("Comm:    %lu\n", uxTaskGetStackHighWaterMark(hCommTask));
        printf("Sensor:  %lu\n", uxTaskGetStackHighWaterMark(hSensorTask));
        printf("Monitor: %lu\n", uxTaskGetStackHighWaterMark(hMonitorTask));
        */

        /* Blink heartbeat LED */
        /* HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); */

#ifndef SIL_BUILD
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
}

/* ======================================================================== */
/*  Kernel object creation + main()                                          */
/* ======================================================================== */

/**
 * @brief Create the FreeRTOS objects the UART ISRs touch.
 *
 * The hardware target calls this from main() BEFORE uart_dma_init()
 * enables the USART1/DMA NVIC lines: the Pi may already be streaming at
 * reset, and an ORE/FE interrupt before the semaphore exists used to call
 * xSemaphoreGiveFromISR(NULL) and spin in configASSERT with the scheduler
 * never started. FreeRTOS permits object creation before the scheduler
 * starts. Idempotent — firmware_arch_main() calls it again.
 */
void comm_create_kernel_objects(void) {
    if (xTxComplete == NULL) {
        xTxComplete = xSemaphoreCreateBinary();
        if (xTxComplete != NULL) {
            /* Available = no DMA TX in flight. */
            xSemaphoreGive(xTxComplete);
        }
    }
}

int firmware_arch_main(void) {
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
    /* Idempotent: the hardware target already called this from main()
     * before uart_dma_init() enabled the UART/DMA interrupts. */
    comm_create_kernel_objects();

    /* --- Create tasks --- */
    xTaskCreate(CtrlTask,    "Ctrl",   512, NULL, 4, &hCtrlTask);
    xTaskCreate(CommTask,    "Comm",   512, NULL, 3, &hCommTask);
#ifndef HW_NO_SENSOR_TASK
    /* I2C sensors. Under HW_IMU_ONLY this reads the MPU6050 and runs the
     * Mahony filter; the ToF half stays compiled out until the VL53L0X is
     * wired (see the ToF block in SensorTask). */
    xTaskCreate(SensorTask,  "Sensor", 256, NULL, 2, &hSensorTask);
#endif
#ifndef HW_MINIMAL_TASKS
    /* The NRF24 link is not wired into the RTOS drive target yet. The task
     * function stays compiled so the SIL build and future hardware targets
     * are unaffected. */
    xTaskCreate(RemoteTask,  "Remote", 256, NULL, 2, &hRemoteTask);
#endif
    xTaskCreate(MonitorTask, "Monitor",128, NULL, 1, &hMonitorTask);

    /* --- Start scheduler (never returns) --- */
    vTaskStartScheduler();

    /* Should never reach here */
    for (;;) {}
}
