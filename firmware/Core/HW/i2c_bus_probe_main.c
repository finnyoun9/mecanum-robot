/**
 * @file i2c_bus_probe_main.c
 * @brief Safe PB10/PB11 sensor-bus probe; never configures motor GPIO.
 *
 * A detected SSD1306 is initialized and forced to all-pixels-on. On a valid
 * ToF startup, ODOM carries raw millimetres and the raw range status. If ToF
 * startup fails, distance = 2000 + 10 * device mask + init stage, where mask
 * bit 0 is MPU6050, bit 1 VL53L0X, bit 2 SSD1306 and bit 3 OLED address 0x3d.
 */
#include "stm32f1xx_hal.h"
#include "protocol.h"
#include "ssd1306.h"
#include "tof_sensor.h"

extern void SystemClock_Config(void);

#define UART_BAUD 921600U
#define DEV_MPU   0x01U
#define DEV_TOF   0x02U
#define DEV_OLED  0x04U
#define DEV_OLED_3D 0x08U

static UART_HandleTypeDef huart1;
static I2C_HandleTypeDef hi2c2;
static uint8_t sequence;
static bool tof_started;

uint32_t hal_get_tick_ms(void) { return HAL_GetTick(); }

static void fail(void) {
    __disable_irq();
    for (;;) {}
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
    if (HAL_UART_Init(&huart1) != HAL_OK) fail();
}

static void i2c_init(void) {
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000U;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0U;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0U;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) fail();
}

static bool ready(uint8_t address) {
    return HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(address << 1), 2U, 5U) == HAL_OK;
}

static void report(uint8_t devices) {
    odom_feedback_t odom = {0};
    uint8_t frame[PROTO_MAX_FRAME];
    uint8_t length = 0;
    odom.imu_q[0] = 1.0f;
    if (tof_started) {
        uint8_t raw[12] = {0};
        (void)HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(0x29U << 1), 0x14U,
                               I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw), 5U);
        odom.tof_distance_mm = (uint16_t)(((uint16_t)raw[10] << 8) | raw[11]);
        odom.error_flags = (uint8_t)((raw[0] & 0x78U) >> 3);
        {
            uint8_t clear = 1U;
            (void)HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(0x29U << 1), 0x0BU,
                                    I2C_MEMADD_SIZE_8BIT, &clear, 1U, 5U);
        }
    } else {
        odom.tof_distance_mm = (uint16_t)(2000U + (uint16_t)devices * 10U +
                                          tof_init_stage());
    }
    if (proto_encode(CMD_ODOM_FEEDBACK, (const uint8_t *)&odom, sizeof(odom),
                     frame, &length, sequence++) >= 0) {
        (void)HAL_UART_Transmit(&huart1, frame, length, 10U);
    }
}

int main(void) {
    uint8_t devices = 0U;
    HAL_Init();
    SystemClock_Config();
    uart_init();
    i2c_init();
    if (ready(0x68U)) devices |= DEV_MPU;
    if (ready(0x29U)) devices |= DEV_TOF;
    if (ready(0x3CU)) {
        devices |= DEV_OLED;
        ssd1306_set_i2c(&hi2c2);
        if (!ssd1306_init() || !ssd1306_set_all_pixels(true)) devices &= (uint8_t)~DEV_OLED;
    }
    if (ready(0x3DU)) devices |= DEV_OLED_3D;
    tof_sensor_set_i2c(&hi2c2);
    tof_started = tof_init();
    for (;;) {
        report(devices);
        HAL_Delay(50U);
    }
}
