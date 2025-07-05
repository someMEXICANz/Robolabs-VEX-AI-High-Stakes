#ifndef I2C_HAL_H
#define I2C_HAL_H

#include "I2C_Device.h"
#include "sh2_hal.h"
#include <memory>
#include <chrono>
#include <mutex>
#include <unordered_map>

class I2C_HAL {
public:
    I2C_HAL(const std::string& i2c_device_path, uint8_t i2c_address = 0x4A);
    ~I2C_HAL();
    
    // Prevent copying (maintain single instance per HAL)
    I2C_HAL(const I2C_HAL&) = delete;
    I2C_HAL& operator=(const I2C_HAL&) = delete;
    
    // Get the HAL interface for use with SH2 library
    sh2_Hal_t* getHalInterface() { return &hal_interface; }
    
    // Check if I2C device is accessible
    bool testConnection();

private:
    std::unique_ptr<I2CDevice> i2c_device;
    sh2_Hal_t hal_interface;
    std::chrono::steady_clock::time_point start_time;
    
    // Thread-safe instance management
    static std::mutex instances_mutex;
    static std::unordered_map<sh2_Hal_t*, I2C_HAL*> hal_instances;
    
    // Static callback functions for SH2 HAL interface
    static int hal_open(sh2_Hal_t *self);
    static void hal_close(sh2_Hal_t *self);
    static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
    static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
    static uint32_t hal_getTimeUs(sh2_Hal_t *self);
    
    // Thread-safe helper to get I2C_HAL instance from sh2_Hal_t
    static I2C_HAL* getHalInstance(sh2_Hal_t *self);
    
    // Internal methods
    uint32_t getCurrentTimeUs();
    void registerInstance();
    void unregisterInstance();
};

#endif // I2C_HAL_H