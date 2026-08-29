/**
 * @file tof_sensor.c
 * @brief VL53L0X continuous-ranging driver via STM32 HAL I2C.
 *
 * Compact implementation of the mandatory VL53L0X bring-up flow: read the
 * factory reference-SPAD data from NVM, load ST's default tuning settings, run
 * VHV/phase reference calibration, then start back-to-back ranging. It only
 * implements the obstacle-ranging profile the robot uses instead of pulling
 * the much larger generic ST PAL into a 20 KiB RAM STM32F103 target.
 *
 * All bus accesses and readiness polls are bounded. A missing sensor cannot
 * block SensorTask forever, and errors never fabricate a zero distance.
 */

#include "tof_sensor.h"

#include <stddef.h>

#ifdef STM32F103xB
#include "stm32f1xx_hal.h"
#else
#include "i2c.h"
#endif

#define TOF_HAL_ADDR                    ((uint16_t)(TOF_I2C_ADDR << 1))
#define TOF_I2C_TIMEOUT_MS              5U
#define TOF_READY_TIMEOUT_MS            50U
#define TOF_TIMEOUT_ERR_THRESHOLD       3U

#define REG_SYSRANGE_START              0x00U
#define REG_SYSTEM_SEQUENCE_CONFIG      0x01U
#define REG_SYSTEM_INTERRUPT_CONFIG     0x0AU
#define REG_SYSTEM_INTERRUPT_CLEAR      0x0BU
#define REG_RESULT_INTERRUPT_STATUS     0x13U
#define REG_RESULT_RANGE_STATUS         0x14U
#define REG_GPIO_HV_MUX_ACTIVE_HIGH     0x84U
#define REG_VHV_CONFIG_PAD_SCL_SDA      0x89U
#define REG_STOP_VARIABLE               0x91U
#define REG_IDENTIFICATION_MODEL_ID     0xC0U
#define REG_GLOBAL_CONFIG_SPAD_REF_0     0xB0U

#define MODEL_ID_EXPECTED               0xEEU
#define RANGE_STATUS_VALID              11U
#define MIN_VALID_MM                    15U
#define MAX_VALID_MM                    2000U

extern uint32_t hal_get_tick_ms(void);

static I2C_HandleTypeDef *bus = NULL;
static uint8_t stop_variable = 0;
static uint16_t last_valid_mm = 0;
static uint8_t consecutive_timeouts = 0;
static bool initialized = false;
static uint8_t init_stage = 0U;

uint8_t tof_init_stage(void) { return init_stage; }

/* STSW-IMG005 DefaultTuningSettings v36, represented as register/value pairs.
 * The original table encodes every entry as {write_count=1, register, value};
 * this equivalent form avoids carrying the generic table interpreter. */
static const uint8_t default_tuning[][2] = {
    {0xFF, 0x01}, {0x00, 0x00},
    {0xFF, 0x00}, {0x09, 0x00}, {0x10, 0x00}, {0x11, 0x00},
    {0x24, 0x01}, {0x25, 0xFF}, {0x75, 0x00},
    {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00}, {0x30, 0x20},
    {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00}, {0x31, 0x04},
    {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25}, {0x60, 0x00},
    {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00}, {0x52, 0x96},
    {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00}, {0x62, 0x00},
    {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0},
    {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF},
    {0x4A, 0x00},
    {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00}, {0x78, 0x21},
    {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00}, {0x44, 0xFF},
    {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40}, {0x0E, 0x06},
    {0x20, 0x1A}, {0x43, 0x40},
    {0xFF, 0x00}, {0x34, 0x03}, {0x35, 0x44},
    {0xFF, 0x01}, {0x31, 0x04}, {0x4B, 0x09}, {0x4C, 0x05},
    {0x4D, 0x04},
    {0xFF, 0x00}, {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08},
    {0x48, 0x28}, {0x67, 0x00}, {0x70, 0x04}, {0x71, 0x01},
    {0x72, 0xFE}, {0x76, 0x00}, {0x77, 0x00},
    {0xFF, 0x01}, {0x0D, 0x01},
    {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8},
    {0xFF, 0x01}, {0x8E, 0x01}, {0x00, 0x01},
    {0xFF, 0x00}, {0x80, 0x00},
};

void tof_sensor_set_i2c(void *hi2c) {
    bus = (I2C_HandleTypeDef *)hi2c;
    initialized = false;
    init_stage = 0U;
}

static bool reg_write(uint8_t reg, uint8_t value) {
    if (bus == NULL) return false;
    return HAL_I2C_Mem_Write(bus, TOF_HAL_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, TOF_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool reg_read(uint8_t reg, uint8_t *data, uint16_t len) {
    if (bus == NULL || data == NULL) return false;
    return HAL_I2C_Mem_Read(bus, TOF_HAL_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            data, len, TOF_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool device_read_strobe(void) {
    uint8_t strobe = 0;
    uint32_t start;

    if (!reg_write(0x83, 0x00)) return false;
    start = hal_get_tick_ms();
    do {
        if (!reg_read(0x83, &strobe, 1)) return false;
        if (strobe != 0) return reg_write(0x83, 0x01);
    } while ((hal_get_tick_ms() - start) < TOF_READY_TIMEOUT_MS);
    return false;
}

static bool read_reference_spads(uint8_t good_map[6], uint8_t *count,
                                 bool *aperture) {
    uint8_t temp = 0;
    bool ok;

    ok = reg_write(0x80, 0x01) && reg_write(0xFF, 0x01) &&
         reg_write(0x00, 0x00) && reg_write(0xFF, 0x06) &&
         reg_read(0x83, &temp, 1) && reg_write(0x83, (uint8_t)(temp | 0x04)) &&
         reg_write(0xFF, 0x07) && reg_write(0x81, 0x01) &&
         reg_write(0x80, 0x01) && reg_write(0x94, 0x6B) &&
         device_read_strobe() && reg_read(0x92, &temp, 1);

    if (ok) {
        *count = (uint8_t)(temp & 0x7FU);
        *aperture = (temp & 0x80U) != 0U;
    }

    /* Always restore the normal page, even after a failed NVM access. */
    ok = reg_write(0x81, 0x00) && ok;
    ok = reg_write(0xFF, 0x06) && ok;
    if (reg_read(0x83, &temp, 1)) {
        ok = reg_write(0x83, (uint8_t)(temp & 0xFBU)) && ok;
    } else {
        ok = false;
    }
    ok = reg_write(0xFF, 0x01) && ok;
    ok = reg_write(0x00, 0x01) && ok;
    ok = reg_write(0xFF, 0x00) && ok;
    ok = reg_write(0x80, 0x00) && ok;
    if (!ok) return false;

    return *count > 0 && *count <= 48 &&
           reg_read(REG_GLOBAL_CONFIG_SPAD_REF_0, good_map, 6U);
}

static bool configure_reference_spads(void) {
    uint8_t good_map[6];
    uint8_t enabled_map[6] = {0};
    uint8_t requested = 0;
    uint8_t enabled = 0;
    bool aperture = false;
    uint8_t first;

    if (!read_reference_spads(good_map, &requested, &aperture)) return false;
    first = aperture ? 12U : 0U;
    for (uint8_t i = 0; i < 48U && enabled < requested; ++i) {
        if (i < first) continue;
        if ((good_map[i / 8U] & (uint8_t)(1U << (i % 8U))) != 0U) {
            enabled_map[i / 8U] |= (uint8_t)(1U << (i % 8U));
            ++enabled;
        }
    }
    if (enabled != requested) return false;

    if (!reg_write(0xFF, 0x01) || !reg_write(0x4F, 0x00) ||
        !reg_write(0x4E, 0x2C) || !reg_write(0xFF, 0x00) ||
        !reg_write(0xB6, 0xB4)) return false;
    if (bus == NULL) return false;
    return HAL_I2C_Mem_Write(bus, TOF_HAL_ADDR, REG_GLOBAL_CONFIG_SPAD_REF_0,
                             I2C_MEMADD_SIZE_8BIT, enabled_map,
                             sizeof(enabled_map), TOF_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool load_default_tuning(void) {
    for (size_t i = 0; i < sizeof(default_tuning) / sizeof(default_tuning[0]); ++i) {
        if (!reg_write(default_tuning[i][0], default_tuning[i][1])) return false;
    }
    return true;
}

static bool wait_measurement_ready(void) {
    uint8_t status = 0;
    uint32_t start = hal_get_tick_ms();
    do {
        if (!reg_read(REG_RESULT_INTERRUPT_STATUS, &status, 1)) return false;
        if ((status & 0x07U) != 0U) return true;
    } while ((hal_get_tick_ms() - start) < TOF_READY_TIMEOUT_MS);
    return false;
}

static bool reference_calibration(uint8_t start_value) {
    if (!reg_write(REG_SYSRANGE_START, start_value)) return false;
    if (!wait_measurement_ready()) return false;
    return reg_write(REG_SYSTEM_INTERRUPT_CLEAR, 0x01) &&
           reg_write(REG_SYSRANGE_START, 0x00);
}

static bool start_continuous(void) {
    return reg_write(0x80, 0x01) && reg_write(0xFF, 0x01) &&
           reg_write(0x00, 0x00) && reg_write(REG_STOP_VARIABLE, stop_variable) &&
           reg_write(0x00, 0x01) && reg_write(0xFF, 0x00) &&
           reg_write(0x80, 0x00) && reg_write(REG_SYSRANGE_START, 0x02);
}

bool tof_init(void) {
    uint8_t model_id = 0;
    uint8_t pad_config = 0;
    uint8_t gpio_polarity = 0;

    initialized = false;
    consecutive_timeouts = 0;
    last_valid_mm = 0;

    if (!reg_read(REG_IDENTIFICATION_MODEL_ID, &model_id, 1) ||
        model_id != MODEL_ID_EXPECTED) return false;
    init_stage = 1U;

    if (!reg_read(REG_VHV_CONFIG_PAD_SCL_SDA, &pad_config, 1) ||
        !reg_write(REG_VHV_CONFIG_PAD_SCL_SDA, (uint8_t)(pad_config | 0x01U)) ||
        !reg_write(0x88, 0x00) ||
        !reg_write(0x80, 0x01) || !reg_write(0xFF, 0x01) ||
        !reg_write(0x00, 0x00) ||
        !reg_read(REG_STOP_VARIABLE, &stop_variable, 1) ||
        !reg_write(0x00, 0x01) || !reg_write(0xFF, 0x00) ||
        !reg_write(0x80, 0x00)) return false;
    init_stage = 2U;

    if (!configure_reference_spads() || !load_default_tuning()) return false;
    init_stage = 3U;

    if (!reg_write(REG_SYSTEM_INTERRUPT_CONFIG, 0x04) ||
        !reg_read(REG_GPIO_HV_MUX_ACTIVE_HIGH, &gpio_polarity, 1) ||
        !reg_write(REG_GPIO_HV_MUX_ACTIVE_HIGH,
                   (uint8_t)(gpio_polarity & (uint8_t)~0x10U)) ||
        !reg_write(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) return false;
    init_stage = 4U;

    if (!reg_write(REG_SYSTEM_SEQUENCE_CONFIG, 0x01) ||
        !reference_calibration(0x41) ||
        !reg_write(REG_SYSTEM_SEQUENCE_CONFIG, 0x02) ||
        !reference_calibration(0x01) ||
        !reg_write(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8) ||
        !start_continuous()) return false;

    initialized = true;
    init_stage = 8U;
    return true;
}

static uint16_t timeout_result(tof_status_t *status) {
    if (consecutive_timeouts < UINT8_MAX) ++consecutive_timeouts;
    if (status != NULL) {
        *status = consecutive_timeouts >= TOF_TIMEOUT_ERR_THRESHOLD
                      ? TOF_TIMEOUT : TOF_NO_SAMPLE;
    }
    return last_valid_mm;
}

uint16_t tof_read_mm(tof_status_t *status) {
    uint8_t ready = 0;
    uint8_t range_status = 0;
    uint8_t range_msb = 0;
    uint8_t range_lsb = 0;
    uint8_t device_status;
    uint16_t mm;

    if (!initialized ||
        !reg_read(REG_RESULT_INTERRUPT_STATUS, &ready, 1) ||
        (ready & 0x07U) == 0U) return timeout_result(status);

    /* The YB-MVV18/VL53L0X breakout returns a corrupted low byte when this
     * window is fetched as one 12-byte I2C transaction on STM32F1 I2C2.
     * Read the status and two range bytes separately: 20 Hz adds negligible
     * bus time and preserves the actual 16-bit millimetre result. */
    if (!reg_read(REG_RESULT_RANGE_STATUS, &range_status, 1) ||
        !reg_read((uint8_t)(REG_RESULT_RANGE_STATUS + 10U), &range_msb, 1) ||
        !reg_read((uint8_t)(REG_RESULT_RANGE_STATUS + 11U), &range_lsb, 1) ||
        !reg_write(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        return timeout_result(status);
    }

    consecutive_timeouts = 0;
    device_status = (uint8_t)((range_status & 0x78U) >> 3);
    mm = (uint16_t)(((uint16_t)range_msb << 8) | range_lsb);
    /* The module's specified near limit is 15 mm.  A zero result is never a
     * physical range; accepting it would make an invalid sample look like a
     * real obstacle at the sensor face. */
    if (device_status != RANGE_STATUS_VALID || mm < MIN_VALID_MM || mm > MAX_VALID_MM) {
        if (status != NULL) *status = TOF_OUT_OF_RANGE;
        return last_valid_mm;
    }

    last_valid_mm = mm;
    if (status != NULL) *status = TOF_OK;
    return mm;
}
