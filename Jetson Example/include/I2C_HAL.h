// I2C_HAL.h
#ifndef I2C_HAL_H
#define I2C_HAL_H

#include "sh2_hal.h"
#include <stdint.h>
#include <memory>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration (to avoid including I2C_Device.h in C context)
class I2CDevice;

// Default I2C configuration for BNO085
#define BNO085_DEFAULT_I2C_ADDRESS 0x4A  // Common address (can be 0x4B depending on SA0 pin)
#define BNO085_DEFAULT_I2C_BUS "/dev/i2c-1"  // Jetson Nano default I2C bus

/**
 * @brief Create I2C-based HAL for SH2 sensor hub communication
 * @param i2c_bus Path to I2C device (e.g., "/dev/i2c-1")
 * @param i2c_address I2C slave address of the sensor
 * @return Pointer to HAL instance, or NULL on failure
 */
sh2_Hal_t* createI2CHAL(const char* i2c_bus, uint8_t i2c_address);

/**
 * @brief Create I2C HAL with default settings for BNO085
 * Uses default bus (/dev/i2c-1) and address (0x4A)
 * @return Pointer to HAL instance, or NULL on failure
 */
sh2_Hal_t* createBNO085HAL(void);

/**
 * @brief Destroy I2C HAL instance and free resources
 * @param hal Pointer to HAL instance to destroy
 */
void destroyI2CHAL(sh2_Hal_t* hal);

/**
 * @brief Get the last error message from the HAL
 * @param hal Pointer to HAL instance
 * @return Error string, or NULL if no error
 */
const char* getI2CHALError(sh2_Hal_t* hal);

/**
 * @brief Check if I2C device is responding
 * @param hal Pointer to HAL instance
 * @return true if device responds, false otherwise
 */
bool isI2CDevicePresent(sh2_Hal_t* hal);

#ifdef __cplusplus
}
#endif

#endif // I2C_HAL_H