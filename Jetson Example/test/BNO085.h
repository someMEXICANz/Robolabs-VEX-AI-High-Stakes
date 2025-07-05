#ifndef BNO085_H
#define BNO085_H

#include "I2C_Device.h"
#include "SH2/sh2.h"
#include "SH2/sh2_SensorValue.h"
#include "SH2/sh2_err.h"

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>
#include <memory>
#include <functional>

// BNO085 I2C Address
#define BNO085_I2C_ADDR 0x4A

// Data structures for sensor readings
struct IMUData {
    // Raw sensor data
    float accel_x, accel_y, accel_z;          // m/s²
    float gyro_x, gyro_y, gyro_z;             // rad/s
    float mag_x, mag_y, mag_z;                // µTesla
    
    // Processed data
    float linear_accel_x, linear_accel_y, linear_accel_z;  // m/s²
    float gravity_x, gravity_y, gravity_z;                  // m/s²
    
    // Rotation data
    float quat_i, quat_j, quat_k, quat_real;  // Quaternion
    float euler_roll, euler_pitch, euler_yaw; // Degrees
    
    // Status
    bool valid;
    uint8_t accuracy;
    std::chrono::system_clock::time_point timestamp;
    
    // Constructor for easy initialization
    IMUData() : accel_x(0), accel_y(0), accel_z(0),
                gyro_x(0), gyro_y(0), gyro_z(0),
                mag_x(0), mag_y(0), mag_z(0),
                linear_accel_x(0), linear_accel_y(0), linear_accel_z(0),
                gravity_x(0), gravity_y(0), gravity_z(0),
                quat_i(0), quat_j(0), quat_k(0), quat_real(1),
                euler_roll(0), euler_pitch(0), euler_yaw(0),
                valid(false), accuracy(0),
                timestamp(std::chrono::system_clock::now()) {}
};

// Calibration status structure
struct CalibrationStatus {
    uint8_t system;
    uint8_t gyroscope;
    uint8_t accelerometer;
    uint8_t magnetometer;
    bool is_calibrated;
    
    CalibrationStatus() : system(0), gyroscope(0), accelerometer(0), 
                         magnetometer(0), is_calibrated(false) {}
};

class BNO085 {
public:
    // Constructor with default I2C path
    explicit BNO085(const std::string& i2c_device = "/dev/i2c-1", 
                    int8_t reset_pin = -1);
    
    // Destructor
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

    // Sensor configuration
    bool enableAccelerometer(uint32_t interval_us = 10000);
    bool enableGyroscope(uint32_t interval_us = 10000);
    bool enableMagnetometer(uint32_t interval_us = 10000);
    bool enableRotationVector(uint32_t interval_us = 10000);
    bool enableLinearAcceleration(uint32_t interval_us = 10000);
    bool enableGravity(uint32_t interval_us = 10000);
    bool enableGameRotationVector(uint32_t interval_us = 10000);
    bool testI2CConnection();
    bool debugGetProductIds();
    bool verifyHALSetup();
    void checkSH2State();

    
    // Disable sensors
    bool disableAllSensors();
    
    // Data access
    IMUData getIMUData() const;
    CalibrationStatus getCalibrationStatus() const;
    
    // Individual sensor data getters
    bool getAcceleration(float& x, float& y, float& z) const;
    bool getGyroscope(float& x, float& y, float& z) const;
    bool getMagnetometer(float& x, float& y, float& z) const;
    bool getQuaternion(float& i, float& j, float& k, float& real) const;
    bool getEulerAngles(float& roll, float& pitch, float& yaw) const;
    
    // Calibration
    bool isCalibrated() const;
    bool saveCalibration();
    bool resetCalibration();
    
    // Configuration
    bool setUpdateRate(uint32_t rate_hz);
    uint32_t getUpdateRate() const { return update_rate_hz; }
    
    // Utility
    const std::string& getLastError() const;
    sh2_ProductIds_t getProductInfo() const { return product_ids; }
    I2CDevice& getI2CDevice() { return i2c_device; }

    // Hardware reset (if reset pin is connected)
    void hardwareReset();
    bool waitForResetComplete();


private:
    // Core functionality
    void readLoop();
    bool configureDevice();
    void updateEulerAngles();
    
    // SH2 HAL interface functions (static callbacks)
    static int hal_open(sh2_Hal_t *self);
    static void hal_close(sh2_Hal_t *self);
    static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
    static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
    static uint32_t hal_getTimeUs(sh2_Hal_t *self);
    
    // SH2 callback functions (static callbacks)
    static void eventCallback(void* cookie, sh2_AsyncEvent_t* event);
    static void sensorCallback(void* cookie, sh2_SensorEvent_t* event);
    
    // Helper functions
    void processSensorEvent(const sh2_SensorValue_t& sensor_value);
    void setError(const std::string& error);
    
    // Hardware interface
    I2CDevice i2c_device;
    int8_t reset_pin;
    
    // SH2 interface
    sh2_Hal_t hal;
    sh2_ProductIds_t product_ids;
    
    // Threading
    std::unique_ptr<std::thread> read_thread;
    std::atomic<bool> running;
    std::atomic<bool> initialized;
    
    // Data protection
    mutable std::mutex data_mutex;
    IMUData current_data;
    CalibrationStatus calibration_status;
    
    // Configuration
    std::atomic<uint32_t> update_rate_hz;
    
    // Error handling
    mutable std::mutex error_mutex;
    std::string last_error;
    
    // Sensor enable tracking
    std::atomic<bool> accel_enabled;
    std::atomic<bool> gyro_enabled;
    std::atomic<bool> mag_enabled;
    std::atomic<bool> rotation_vector_enabled;
    
    // Constants
    static constexpr uint32_t DEFAULT_UPDATE_RATE_HZ = 100;
    static constexpr uint32_t DEFAULT_SENSOR_INTERVAL_US = 10000; // 100Hz
    static constexpr float RAD_TO_DEG = 180.0f / 3.14159265359f;
};

#endif // BNO085_H