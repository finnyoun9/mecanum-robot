/**
 * @file tof_sensor.h
 * @brief I2C driver for VL53L0X time-of-flight distance sensor.
 *
 * VL53L0X chosen as the cheapest/most common I2C ToF module (matches the
 * "ToF" line item in the project's purchase plan). Uses the sensor's
 * default single-shot ranging mode — the full ST API reference sequence
 * (SPAD calibration, VHV/phase calibration, timing budget tuning) is
 * omitted since it doesn't affect the read/status logic below; factory
 * default calibration is adequate for obstacle-distance sensing.
 */

#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

/** Result of a ranging read */
typedef enum {
    TOF_OK = 0,          /* Valid range in tof_read_mm()'s return value */
    TOF_OUT_OF_RANGE,    /* Sensor responded but reading is invalid/out of range */
    TOF_TIMEOUT,         /* Sensor stopped responding (consecutive read failures) */
} tof_status_t;

/**
 * @brief Initialise the VL53L0X: verify model ID, start continuous
 *        single-shot ranging mode.
 * @return true if the model ID matched.
 */
bool tof_init(void);

/**
 * @brief Trigger a range measurement and read the result.
 * @param status  Optional output: TOF_OK / TOF_OUT_OF_RANGE / TOF_TIMEOUT.
 *                Pass NULL if not needed.
 * @return Distance in mm. On TOF_OUT_OF_RANGE/TOF_TIMEOUT, returns the last
 *         known-good reading rather than a bogus 0.
 */
uint16_t tof_read_mm(tof_status_t *status);

#endif /* TOF_SENSOR_H */
