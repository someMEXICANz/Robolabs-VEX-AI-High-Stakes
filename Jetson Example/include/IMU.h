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
    bool isRunning() const { return running; }
    bool isConnected() const { return i2c_fd >= 0; }
    
    // Read sensor data
    bool getAccelerometer(float& ax, float& ay, float& az) const;
    bool getGyroscope(float& gx, float& gy, float& gz) const;
    bool getMagnetometer(float& mx, float& my, float& mz) const;
    bool getTemperature(float& temp) const;
    bool getAll(float& ax, float& ay, float& az,
                float& gx, float& gy, float& gz,
                float& mx, float& my, float& mz,
                float& temp) const;
    
    // Calibration methods
    bool calibrateAccelerometer();
    bool calibrateGyroscope();
    bool calibrateMagnetometer();
    
    // Error handling
    std::string getLastError() const { return last_error; }

private:


    void readLoop();
    bool readData();
    
    // I2C communication
    bool writeByte(uint8_t devAddr, uint8_t reg, uint8_t data);
    bool readBytes(uint8_t devAddr, uint8_t reg, uint8_t* buffer, size_t length);

    // I2C device properties
    std::string i2c_bus;
    int i2c_fd;
    std::string last_error;
    
    // Sensor addresses
    static constexpr uint8_t LSM6DS3_ADDR = 0x6A;
    static constexpr uint8_t LIS3MDL_ADDR = 0x1C;
    
    // Calibration offsets
    float accel_offset_x, accel_offset_y, accel_offset_z;
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;
    float mag_offset_x, mag_offset_y, mag_offset_z;
    
    // Scale factors
    float accel_scale;  // g per LSB
    float gyro_scale;   // dps per LSB
    float mag_scale;    // gauss per LSB
    
    // Thread and running state
    mutable std::mutex data_mutex;
    std::unique_ptr<std::thread> read_thread;
    std::atomic<bool> running;
    std::atomic<bool> connected;
    
    // Sensor data and mutex
   
    struct IMUData {
        float ax, ay, az;  // Accelerometer (g)
        float gx, gy, gz;  // Gyroscope (dps)
        float mx, my, mz;  // Magnetometer (gauss)
        float temperature; // Temperature (°C)
        bool valid;        // Flag indicating if data is valid
    } sensor_data;
    
  
    
   
};

#endif // IMU_H