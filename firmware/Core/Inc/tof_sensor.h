/**
 * @file tof_sensor.h
 * @brief Compact VL53L0X I2C driver for obstacle ranging.
 */

#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#define TOF_I2C_ADDR 0x29U

typedef enum {
    TOF_OK = 0,
    TOF_OUT_OF_RANGE,
    TOF_TIMEOUT,
} tof_status_t;

/** Bind the I2C bus used by the sensor. Must precede tof_init(). */
void tof_sensor_set_i2c(void *hi2c);

/**
 * Verify identity, load ST tuning/SPAD calibration and start continuous range.
 * Returns false on an absent/wrong device, I2C error or calibration timeout.
 */
bool tof_init(void);
/** Last completed init stage (0 before identity check, 8 ready). */
uint8_t tof_init_stage(void);

/**
 * Read the latest continuous-ranging result without blocking for a new sample.
 * A missing sample becomes TOF_TIMEOUT after three consecutive 20 Hz polls.
 * Errors retain and return the last known-good distance.
 */
uint16_t tof_read_mm(tof_status_t *status);

#endif /* TOF_SENSOR_H */
