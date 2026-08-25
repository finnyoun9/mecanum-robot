/**
 * @file uart_link_probe_main.c
 * @brief M4 step 1: safe USART1 protocol probe for the Pi 5 link.
 *
 * PA9 (USART1_TX) and PA10 (USART1_RX) run at 921600 bit/s, matching the
 * ROS 2 hardware interface default.  This target never configures a motor,
 * TB6612 standby pin, or encoder interrupt: it is safe to flash while the
 * chassis is on the ground.  It validates the real wire using the shared
 * framed CRC protocol before M4 permits a UART command to reach the speed PI.
 */
#include "stm32f1xx_hal.h"
#include "protocol.h"
#include <string.h>

extern void SystemClock_Config(void);

#define UART_BAUD             921600U
#define ODOM_PERIOD_MS        50U

typedef enum {
    PARSER_SYNC0,
    PARSER_SYNC1,
    PARSER_HEADER,
    PARSER_BODY,
} parser_state_t;

static UART_HandleTypeDef huart1;
static parser_state_t parser_state;
static uint8_t parser_frame[PROTO_MAX_FRAME];
static uint8_t parser_index;
static uint8_t parser_expected;
static uint8_t tx_sequence;

/* GDB telemetry.  These variables are observational; reading them never
 * enables any actuator. */
volatile uint32_t uart_rx_bytes;
volatile uint32_t valid_frames;
volatile uint32_t invalid_frames;
volatile uint32_t ack_frames;
volatile uint32_t odom_frames;
volatile uint8_t last_command;
volatile uint8_t last_sequence;
volatile uint8_t last_payload_length;
volatile uint32_t last_valid_rx_ms;

static void Error_Handler(void) {
    __disable_irq();
    for (;;) {
    }
}

static void uart_init(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = UART_BAUD;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void send_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_length) {
    uint8_t frame[PROTO_MAX_FRAME];
    uint8_t frame_length;

    if (proto_encode(cmd, payload, payload_length, frame, &frame_length,
                     tx_sequence++) < 0) {
        return;
    }
    if (HAL_UART_Transmit(&huart1, frame, frame_length, 5U) == HAL_OK) {
        if (cmd == CMD_ACK) ack_frames++;
        if (cmd == CMD_ODOM_FEEDBACK) odom_frames++;
    }
}

static void send_odometry(void) {
    odom_feedback_t odom = {0};

    /* This link-only target has no sensors.  Identity orientation lets the
     * Pi distinguish a valid transport frame from uninitialised bytes. */
    odom.imu_q[0] = 1.0f;
    send_frame(CMD_ODOM_FEEDBACK, (const uint8_t *)&odom, sizeof(odom));
}

static void handle_frame(const uint8_t *frame, uint8_t frame_length) {
    uint8_t command;
    uint8_t payload[PROTO_MAX_PAYLOAD];
    uint8_t payload_length;
    uint8_t sequence;

    if (proto_decode(frame, frame_length, &command, payload, &payload_length,
                     &sequence) < 0) {
        invalid_frames++;
        return;
    }

    valid_frames++;
    last_command = command;
    last_sequence = sequence;
    last_payload_length = payload_length;
    last_valid_rx_ms = HAL_GetTick();

    /* A heartbeat should receive a deterministic response.  Velocity and
     * e-stop frames are counted but deliberately cannot actuate anything in
     * this probe target. */
    if (command == CMD_HEARTBEAT) send_frame(CMD_ACK, NULL, 0U);
}

static void parser_push(uint8_t byte) {
    uart_rx_bytes++;

    switch (parser_state) {
    case PARSER_SYNC0:
        if (byte == PROTO_SYNC0) {
            parser_frame[0] = byte;
            parser_index = 1U;
            parser_state = PARSER_SYNC1;
        }
        break;

    case PARSER_SYNC1:
        if (byte == PROTO_SYNC1) {
            parser_frame[1] = byte;
            parser_index = 2U;
            parser_state = PARSER_HEADER;
        } else if (byte == PROTO_SYNC0) {
            parser_frame[0] = byte;
            parser_index = 1U;
        } else {
            parser_state = PARSER_SYNC0;
        }
        break;

    case PARSER_HEADER:
        parser_frame[parser_index++] = byte;
        if (parser_index == 5U) { /* SYNC0, SYNC1, LEN, SEQ, CMD */
            if (parser_frame[2] > PROTO_MAX_PAYLOAD) {
                invalid_frames++;
                parser_state = PARSER_SYNC0;
            } else {
                parser_expected = (uint8_t)(PROTO_FRAME_OVERHEAD + parser_frame[2]);
                parser_state = PARSER_BODY;
            }
        }
        break;

    case PARSER_BODY:
        parser_frame[parser_index++] = byte;
        if (parser_index == parser_expected) {
            handle_frame(parser_frame, parser_index);
            parser_state = PARSER_SYNC0;
        }
        break;

    default:
        parser_state = PARSER_SYNC0;
        break;
    }
}

int main(void) {
    uint32_t next_odom_ms;

    HAL_Init();
    SystemClock_Config();
    uart_init();
    next_odom_ms = HAL_GetTick() + ODOM_PERIOD_MS;

    for (;;) {
        uint8_t byte;
        uint32_t now;

        if (HAL_UART_Receive(&huart1, &byte, 1U, 0U) == HAL_OK) {
            parser_push(byte);
        }

        now = HAL_GetTick();
        if ((int32_t)(now - next_odom_ms) >= 0) {
            next_odom_ms = now + ODOM_PERIOD_MS;
            send_odometry();
        }
    }
}
