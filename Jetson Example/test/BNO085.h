#ifndef BNO085_H
#define BNO085_H

#include "I2C_Device.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>
#include <memory>
#include <functional>

// BNO085 I2C address
#define BNO085_I2C_ADDR 0x4A

// Common sensor report rates (microseconds)
#define BNO085_REPORT_INTERVAL_10HZ  100000   // 10 Hz
#define BNO085_REPORT_INTERVAL_50HZ  20000    // 50 Hz
#define BNO085_REPORT_INTERVAL_100HZ 10000    // 100 Hz
#define BNO085_REPORT_INTERVAL_400HZ 2500     // 400 Hz

struct BNO085Data {
    // Quaternion (rotation vector)
    float quat_i, quat_j, quat_k, quat_real;
    float quat_accuracy;
    
    // Linear acceleration (m/s²)
    float accel_x, accel_y, accel_z;
    float accel_accuracy;
    
    // Gyroscope (rad/s)
    float gyro_x, gyro_y, gyro_z;
    float gyro_accuracy;
    
    // Magnetometer (µT)
    float mag_x, mag_y, mag_z;
    float mag_accuracy;
    
    // Game rotation vector (no magnetometer)
    float game_quat_i, game_quat_j, game_quat_k, game_quat_real;
    float game_quat_accuracy;
    
    // Timestamp and validity
    uint32_t timestamp;
    bool valid;
    std::chrono::system_clock::time_point system_timestamp;
    
    BNO085Data() : quat_i(0), quat_j(0), quat_k(0), quat_real(1), quat_accuracy(0),
                   accel_x(0), accel_y(0), accel_z(0), accel_accuracy(0),
                   gyro_x(0), gyro_y(0), gyro_z(0), gyro_accuracy(0),
                   mag_x(0), mag_y(0), mag_z(0), mag_accuracy(0),
                   game_quat_i(0), game_quat_j(0), game_quat_k(0), game_quat_real(1), game_quat_accuracy(0),
                   timestamp(0), valid(false), system_timestamp(std::chrono::system_clock::now()) {}
};

class BNO085 {
public:
    explicit BNO085(const std::string& i2c_device = "/dev/i2c-1", int reset_pin = -1);
    ~BNO085();
    
    // Delete copy constructor and assignment operator
    BNO085(const BNO085&) = delete;
    BNO085& operator=(const BNO085&) = delete;

    // Lifecycle management
    bool initialize();
    bool start();
    void stop();
    bool restart();
    bool reconnect();

    // Status checks
    bool isRunning() const { return running.load(); }
    bool isInitialized() const { return initialized.load(); }
    bool isConnected() const { return i2c_device.isOpen(); }
    bool wasReset() const { return reset_occurred.load(); }

    // Sensor configuration
    bool enableRotationVector(uint32_t interval_us = BNO085_REPORT_INTERVAL_100HZ);
    bool enableLinearAccelerometer(uint32_t interval_us = BNO085_REPORT_INTERVAL_100HZ);
    bool enableGyroscope(uint32_t interval_us = BNO085_REPORT_INTERVAL_100HZ);
    bool enableMagnetometer(uint32_t interval_us = BNO085_REPORT_INTERVAL_100HZ);
    bool enableGameRotationVector(uint32_t interval_us = BNO085_REPORT_INTERVAL_100HZ);

    // Data access
    BNO085Data getSensorData() const;
    bool getQuaternion(float& i, float& j, float& k, float& real, float& accuracy) const;
    bool getLinearAcceleration(float& x, float& y, float& z, float& accuracy) const;
    bool getGyroscope(float& x, float& y, float& z, float& accuracy) const;
    bool getMagnetometer(float& x, float& y, float& z, float& accuracy) const;
    bool getGameRotationVector(float& i, float& j, float& k, float& real, float& accuracy) const;
    
    // Utility
    const std::string& getLastError() const;
    sh2_ProductIds_t getProductIds() const { return product_ids; }

private:
    // Core functionality
    void readLoop();
    bool initializeSH2();
    void processSensorEvent(sh2_SensorValue_t* sensor_value);
    void handleAsyncEvent(sh2_AsyncEvent_t* event);
    void hardwareReset();
    
    // HAL implementation functions (static)
    static int halOpen(sh2_Hal_t* self);
    static void halClose(sh2_Hal_t* self);
    static int halRead(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len, uint32_t* t_us);
    static int halWrite(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len);
    static uint32_t halGetTimeUs(sh2_Hal_t* self);
    
    // Static callbacks for SH2 library
    static void sensorEventCallback(void* cookie, sh2_SensorEvent_t* pEvent);
    static void asyncEventCallback(void* cookie, sh2_AsyncEvent_t* pEvent);
    
    // Utility functions
    void setError(const std::string& error);
    bool performSoftReset();
    
    // Hardware interface
    I2CDevice i2c_device;
    int reset_pin;
    
    // SH2 library interface
    sh2_Hal_t hal;
    sh2_ProductIds_t product_ids;
    
    // Threading
    std::unique_ptr<std::thread> read_thread;
    std::atomic<bool> running;
    std::atomic<bool> initialized;
    std::atomic<bool> reset_occurred;
    
    // Data protection
    mutable std::mutex data_mutex;
    BNO085Data current_data;
    
    // Error handling
    mutable std::mutex error_mutex;
    std::string last_error;
    
    // Constants
    static constexpr uint32_t DEFAULT_LOOP_DELAY_MS = 1;
    static constexpr uint32_t RESET_DELAY_MS = 100;
    static constexpr uint32_t INIT_TIMEOUT_MS = 5000;
};

#endif // BNO085_H