// IMU.h - Restructured
#ifndef IMU_H
#define IMU_H

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

class IMU {
public:
    // Constructor with default I2C bus path
    IMU(const std::string& i2cBus = "/dev/i2c-1");
    
    // Destructor
    ~IMU();

    // Core operations (similar to GPS)
    bool initialize();
    bool start();
    void stop();
    bool restart();
    bool reconnect();
    
    // Check running state
    bool isRunning() const { return running_; }
    bool isConnected() const { return i2c_fd_ >= 0; }
    
    // Read sensor data
    bool readAccelerometer(float& ax, float& ay, float& az) const;
    bool readGyroscope(float& gx, float& gy, float& gz) const;
    bool readMagnetometer(float& mx, float& my, float& mz) const;
    bool readTemperature(float& temp) const;
    bool readAll(float& ax, float& ay, float& az,
                float& gx, float& gy, float& gz,
                float& mx, float& my, float& mz,
                float& temp) const;
    
    // Calibration methods
    bool calibrateAccelerometer();
    bool calibrateGyroscope();
    bool calibrateMagnetometer();
    
    // Error handling
    std::string getLastError() const { return last_error_; }

private:


    void readLoop();
    bool readSensors();
    
    // I2C communication
    bool writeByte(uint8_t devAddr, uint8_t reg, uint8_t data);
    bool readBytes(uint8_t devAddr, uint8_t reg, uint8_t* buffer, size_t length);

    // I2C device properties
    std::string i2c_bus_;
    int i2c_fd_;
    std::string last_error_;
    
    // Sensor addresses
    static constexpr uint8_t LSM6DS3_ADDR = 0x6A;
    static constexpr uint8_t LIS3MDL_ADDR = 0x1C;
    
    // Calibration offsets
    float accel_offset_x_, accel_offset_y_, accel_offset_z_;
    float gyro_offset_x_, gyro_offset_y_, gyro_offset_z_;
    float mag_offset_x_, mag_offset_y_, mag_offset_z_;
    
    // Scale factors
    float accel_scale_;  // g per LSB
    float gyro_scale_;   // dps per LSB
    float mag_scale_;    // gauss per LSB
    
    // Thread and running state
    std::unique_ptr<std::thread> read_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> connected_;
    
    // Sensor data and mutex
    mutable std::mutex data_mutex_;
    struct SensorData {
        float ax, ay, az;  // Accelerometer (g)
        float gx, gy, gz;  // Gyroscope (dps)
        float mx, my, mz;  // Magnetometer (gauss)
        float temperature; // Temperature (°C)
        bool valid;        // Flag indicating if data is valid
    } sensor_data_;
    
  
    
   
};

#endif // IMU_H