#include "IMU.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>

// LSM6DS3TR-C registers
#define LSM6DS3_WHO_AM_I          0x0F
#define LSM6DS3_CTRL1_XL          0x10  // Accelerometer control register
#define LSM6DS3_CTRL2_G           0x11  // Gyroscope control register
#define LSM6DS3_CTRL3_C           0x12  // Control register 3
#define LSM6DS3_OUT_TEMP_L        0x20  // Temperature LSB
#define LSM6DS3_OUT_TEMP_H        0x21  // Temperature MSB
#define LSM6DS3_OUTX_L_G          0x22  // Gyroscope X-axis LSB
#define LSM6DS3_OUTX_H_G          0x23  // Gyroscope X-axis MSB
#define LSM6DS3_OUTY_L_G          0x24  // Gyroscope Y-axis LSB
#define LSM6DS3_OUTY_H_G          0x25  // Gyroscope Y-axis MSB
#define LSM6DS3_OUTZ_L_G          0x26  // Gyroscope Z-axis LSB
#define LSM6DS3_OUTZ_H_G          0x27  // Gyroscope Z-axis MSB
#define LSM6DS3_OUTX_L_XL         0x28  // Accelerometer X-axis LSB
#define LSM6DS3_OUTX_H_XL         0x29  // Accelerometer X-axis MSB
#define LSM6DS3_OUTY_L_XL         0x2A  // Accelerometer Y-axis LSB
#define LSM6DS3_OUTY_H_XL         0x2B  // Accelerometer Y-axis MSB
#define LSM6DS3_OUTZ_L_XL         0x2C  // Accelerometer Z-axis LSB
#define LSM6DS3_OUTZ_H_XL         0x2D  // Accelerometer Z-axis MSB

// LIS3MDL registers
#define LIS3MDL_WHO_AM_I          0x0F
#define LIS3MDL_CTRL_REG1         0x20  // Control register 1
#define LIS3MDL_CTRL_REG2         0x21  // Control register 2
#define LIS3MDL_CTRL_REG3         0x22  // Control register 3
#define LIS3MDL_CTRL_REG4         0x23  // Control register 4
#define LIS3MDL_OUT_TEMP_L        0x2E  // Temperature LSB
#define LIS3MDL_OUT_TEMP_H        0x2F  // Temperature MSB
#define LIS3MDL_OUT_X_L           0x28  // X-axis LSB
#define LIS3MDL_OUT_X_H           0x29  // X-axis MSB
#define LIS3MDL_OUT_Y_L           0x2A  // Y-axis LSB
#define LIS3MDL_OUT_Y_H           0x2B  // Y-axis MSB
#define LIS3MDL_OUT_Z_L           0x2C  // Z-axis LSB
#define LIS3MDL_OUT_Z_H           0x2D  // Z-axis MSB

// IMU.cpp - Restructured core functionality
#include "IMU.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>

// LSM6DS3TR-C registers (keeping the existing register definitions)

IMU::IMU(const std::string& i2cBus)
    : i2c_bus_(i2cBus),
      i2c_fd_(-1),
      accel_offset_x_(0.0f), accel_offset_y_(0.0f), accel_offset_z_(0.0f),
      gyro_offset_x_(0.0f), gyro_offset_y_(0.0f), gyro_offset_z_(0.0f),
      mag_offset_x_(0.0f), mag_offset_y_(0.0f), mag_offset_z_(0.0f),
      accel_scale_(0.061f / 1000.0f),  // ±2g range: 0.061 mg/LSB
      gyro_scale_(70.0f / 1000.0f),    // ±2000 dps range: 70 mdps/LSB
      mag_scale_(0.58f / 1000.0f),     // ±16 gauss range: 0.58 mgauss/LSB
      running_(false),
      connected_(false)
{
    // Initialize sensor data
    sensor_data_.ax = 0.0f;
    sensor_data_.ay = 0.0f;
    sensor_data_.az = 0.0f;
    sensor_data_.gx = 0.0f;
    sensor_data_.gy = 0.0f;
    sensor_data_.gz = 0.0f;
    sensor_data_.mx = 0.0f;
    sensor_data_.my = 0.0f;
    sensor_data_.mz = 0.0f;
    sensor_data_.temperature = 0.0f;
    sensor_data_.valid = false;
    
    // Try to initialize if bus is provided
    if (!i2c_bus_.empty()) 
    {
        if(initialize())
        {
            start();
        }
    }
}

IMU::~IMU() {
    stop();
    
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
        i2c_fd_ = -1;
    }
}

bool IMU::initialize() {
    // Open I2C device
    i2c_fd_ = open(i2c_bus_.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        std::cerr << "Failed to open I2C bus: " << i2c_bus_ << std::endl;
        return false;
    }

    // Initialize accelerometer and gyroscope
    if (!writeByte(LSM6DS3_ADDR, LSM6DS3_CTRL1_XL, 0x60)) {  // 208Hz, ±2g
        std::cerr << "Failed to configure accelerometer" << std::endl;
        return false;
    }

    if (!writeByte(LSM6DS3_ADDR, LSM6DS3_CTRL2_G, 0x6C)) {  // 208Hz, ±2000dps
        std::cerr << "Failed to configure gyroscope" << std::endl;
        return false;
    }

    // Initialize magnetometer
    if (!writeByte(LIS3MDL_ADDR, LIS3MDL_CTRL_REG1, 0x7C)) {  // Ultra-high performance, 80Hz
        std::cerr << "Failed to configure magnetometer" << std::endl;
        return false;
    }

    if (!writeByte(LIS3MDL_ADDR, LIS3MDL_CTRL_REG2, 0x40)) {  // ±16 gauss
        std::cerr << "Failed to set magnetometer range" << std::endl;
        return false;
    }

    if (!writeByte(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3, 0x00)) {  // Continuous mode
        std::cerr << "Failed to set magnetometer mode" << std::endl;
        return false;
    }
    
    connected_ = true;
    return true;
}

bool IMU::start() {
    if (running_) return true;
    
    if (i2c_fd_ < 0 && !reconnect()) {
        return false;
    }
    
    running_ = true;
    read_thread_ = std::make_unique<std::thread>(&IMU::readLoop, this);

    calibrateGyroscope();
    calibrateAccelerometer();
    calibrateMagnetometer();
    sleep(1);
    
    return true;
}

void IMU::stop() {
    running_ = false;
    
    if (read_thread_ && read_thread_->joinable()) {
        read_thread_->join();
    }
    
    read_thread_.reset();
}

bool IMU::restart() {
    stop();
    return start();
}

bool IMU::reconnect() {
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
        i2c_fd_ = -1;
    }
    return initialize();
}

// Main reading loop that runs in a separate thread
void IMU::readLoop() {
    std::cerr << "IMU read loop started" << std::endl;
    
    const std::chrono::milliseconds read_interval(10); // 100Hz update rate
    
    while (running_) {
        auto start_time = std::chrono::steady_clock::now();
        
        if (!connected_ && !reconnect()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        
        // Read all sensor data
        readSensors();
        
        // Calculate time to sleep
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed < read_interval) {
            std::this_thread::sleep_for(read_interval - elapsed);
        }
    }
    
    std::cerr << "IMU read loop stopped" << std::endl;
}

// Read all sensors and update internal data structure
bool IMU::readSensors() {
    if (i2c_fd_ < 0) {
        return false;
    }
    
    // Read accelerometer
    uint8_t accel_buffer[6];
    if (!readBytes(LSM6DS3_ADDR, LSM6DS3_OUTX_L_XL, accel_buffer, 6)) {
        std::cerr << "Failed to read accelerometer data" << std::endl;
        return false;
    }
    
    // Read gyroscope
    uint8_t gyro_buffer[6];
    if (!readBytes(LSM6DS3_ADDR, LSM6DS3_OUTX_L_G, gyro_buffer, 6)) {
         std::cerr << "Failed to read gyroscope data" << std::endl;
        return false;
    }
    
    // Read magnetometer
    uint8_t mag_buffer[6];
    if (!readBytes(LIS3MDL_ADDR, LIS3MDL_OUT_X_L, mag_buffer, 6)) {
         std::cerr << "Failed to read magnetometer data" << std::endl;
        return false;
    }
    
    // Read temperature
    uint8_t temp_buffer[2];
    if (!readBytes(LSM6DS3_ADDR, LSM6DS3_OUT_TEMP_L, temp_buffer, 2)) {
         std::cerr << "Failed to read temperature data" << std::endl;
        return false;
    }
    
    // Process all readings
    int16_t raw_ax = (accel_buffer[1] << 8) | accel_buffer[0];
    int16_t raw_ay = (accel_buffer[3] << 8) | accel_buffer[2];
    int16_t raw_az = (accel_buffer[5] << 8) | accel_buffer[4];
    
    int16_t raw_gx = (gyro_buffer[1] << 8) | gyro_buffer[0];
    int16_t raw_gy = (gyro_buffer[3] << 8) | gyro_buffer[2];
    int16_t raw_gz = (gyro_buffer[5] << 8) | gyro_buffer[4];
    
    int16_t raw_mx = (mag_buffer[1] << 8) | mag_buffer[0];
    int16_t raw_my = (mag_buffer[3] << 8) | mag_buffer[2];
    int16_t raw_mz = (mag_buffer[5] << 8) | mag_buffer[4];
    
    int16_t raw_temp = (temp_buffer[1] << 8) | temp_buffer[0];
    
    // Apply scaling and offsets
    float ax = (raw_ax * accel_scale_) - accel_offset_x_;
    float ay = (raw_ay * accel_scale_) - accel_offset_y_;
    float az = (raw_az * accel_scale_) - accel_offset_z_;
    
    float gx = (raw_gx * gyro_scale_) - gyro_offset_x_;
    float gy = (raw_gy * gyro_scale_) - gyro_offset_y_;
    float gz = (raw_gz * gyro_scale_) - gyro_offset_z_;
    
    float mx = (raw_mx * mag_scale_) - mag_offset_x_;
    float my = (raw_my * mag_scale_) - mag_offset_y_;
    float mz = (raw_mz * mag_scale_) - mag_offset_z_;
    
    float temp = static_cast<float>(raw_temp) / 16.0f + 25.0f;
    
    // Update data structure with mutex protection
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        sensor_data_.ax = ax;
        sensor_data_.ay = ay;
        sensor_data_.az = az;
        sensor_data_.gx = gx;
        sensor_data_.gy = gy;
        sensor_data_.gz = gz;
        sensor_data_.mx = mx;
        sensor_data_.my = my;
        sensor_data_.mz = mz;
        sensor_data_.temperature = temp;
        sensor_data_.valid = true;
    }
    
    return true;
}

// Get accelerometer data (thread-safe)
bool IMU::readAccelerometer(float& ax, float& ay, float& az) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (!sensor_data_.valid && !running_) {
        std::cerr << "No valid sensor data available" << std::endl;
        return false;
    }
    
    ax = sensor_data_.ax;
    ay = sensor_data_.ay;
    az = sensor_data_.az;
    
    return true;
}

// Additional sensor data access methods
bool IMU::readGyroscope(float& gx, float& gy, float& gz) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (!sensor_data_.valid && !running_) {
        std::cerr << "No valid sensor data available" << std::endl;
        return false;
    }
    
    gx = sensor_data_.gx;
    gy = sensor_data_.gy;
    gz = sensor_data_.gz;
    
    return true;
}

bool IMU::readMagnetometer(float& mx, float& my, float& mz) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (!sensor_data_.valid && !running_) {
        std::cerr << "No valid sensor data available" << std::endl;
        return false;
    }
    
    mx = sensor_data_.mx;
    my = sensor_data_.my;
    mz = sensor_data_.mz;
    
    return true;
}

bool IMU::readTemperature(float& temp) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (!sensor_data_.valid && !running_) {
        std::cerr << "No valid sensor data available" << std::endl;
        return false;
    }
    
    temp = sensor_data_.temperature;
    
    return true;
}

bool IMU::readAll(float& ax, float& ay, float& az,
                float& gx, float& gy, float& gz,
                float& mx, float& my, float& mz,
                float& temp) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (!sensor_data_.valid && !running_) {
        std::cerr << "No valid sensor data available" << std::endl;
        return false;
    }
    
    ax = sensor_data_.ax;
    ay = sensor_data_.ay;
    az = sensor_data_.az;
    gx = sensor_data_.gx;
    gy = sensor_data_.gy;
    gz = sensor_data_.gz;
    mx = sensor_data_.mx;
    my = sensor_data_.my;
    mz = sensor_data_.mz;
    temp = sensor_data_.temperature;
    
    return true;
}

// Calibration methods
bool IMU::calibrateAccelerometer() {
    if (!isRunning()) {
        std::cerr << "IMU must be running to calibrate" << std::endl;
        return false;
    }
    
    // Check if device is stationary
    float gx, gy, gz;
    if (readGyroscope(gx, gy, gz)) {
        float movement = std::abs(gx) + std::abs(gy) + std::abs(gz);
        if (movement > 1.0f) {
            std::cerr << "Device must be stationary for calibration" << std::endl;
            return false;
        }
    }
    
    const int numSamples = 100;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    
    for (int i = 0; i < numSamples; ++i) {
        float x, y, z;
        if (!readAccelerometer(x, y, z)) {
            return false;
        }
        
        sumX += x;
        sumY += y;
        sumZ += z;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    accel_offset_x_ = sumX / numSamples;
    accel_offset_y_ = sumY / numSamples;
    accel_offset_z_ = (sumZ / numSamples) - 1.0f;  // Remove gravity (1g)
    
    std::cerr << "Accelerometer calibration: offsets = (" 
              << accel_offset_x_ << ", " << accel_offset_y_ << ", " 
              << accel_offset_z_ << ")" << std::endl;
    
    return true;
}




// Calibrate gyroscope
bool IMU::calibrateGyroscope() 
{
    if (!isRunning()) 
    {
        std::cerr << "IMU must be running to calibrate" << std::endl;
        return false;
    }
    
    // Check if device is stationary
    float gx, gy, gz;
    if (readGyroscope(gx, gy, gz)) {
        float movement = std::abs(gx) + std::abs(gy) + std::abs(gz);
        if (movement > 1.0f) {
            std::cerr << "Device must be stationary for calibration" << std::endl;
            return false;
        }
    }


    const int numSamples = 100;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    
    for (int i = 0; i < numSamples; ++i) {
        float x, y, z;
        if (!readGyroscope(x, y, z)) {
            return false;
        }
        
        sumX += x;
        sumY += y;
        sumZ += z;
        
        usleep(10000);  // 10ms delay
    }
    
    gyro_offset_x_ = sumX / numSamples;
    gyro_offset_y_ = sumY / numSamples;
    gyro_offset_z_ = sumZ / numSamples;
    
    return true;
}

// Calibrate magnetometer
bool IMU::calibrateMagnetometer() {

    if (!isRunning()) 
    {
        std::cerr << "IMU must be running to calibrate" << std::endl;
        return false;
    }
    
    // Check if device is stationary
    float gx, gy, gz;
    if (readGyroscope(gx, gy, gz)) {
        float movement = std::abs(gx) + std::abs(gy) + std::abs(gz);
        if (movement > 1.0f) {
            std::cerr << "Device must be stationary for calibration" << std::endl;
            return false;
        }
    }

    const int numSamples = 100;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    
    for (int i = 0; i < numSamples; ++i) {
        float x, y, z;
        if (!readMagnetometer(x, y, z)) {
            return false;
        }
        
        sumX += x;
        sumY += y;
        sumZ += z;
        
        usleep(10000);  // 10ms delay
    }
    
    
    mag_offset_x_ = sumX / numSamples;
    mag_offset_y_ = sumY / numSamples;
    mag_offset_z_ = sumZ / numSamples;
    
    return true;
}






// I2C communication methods
bool IMU::writeByte(uint8_t devAddr, uint8_t reg, uint8_t data) {
    if (i2c_fd_ < 0) {
        std::cerr << "I2C device not open" << std::endl;
        return false;
    }
    
    if (ioctl(i2c_fd_, I2C_SLAVE, devAddr) < 0) {
        std::cerr << "Failed to set I2C slave address" << std::endl;
        return false;
    }
    
    uint8_t buffer[2] = {reg, data};
    if (write(i2c_fd_, buffer, 2) != 2) {
        std::cerr << "Failed to write to register" << std::endl;
        return false;
    }
    
    return true;
}

bool IMU::readBytes(uint8_t devAddr, uint8_t reg, uint8_t* buffer, size_t length) {
    if (i2c_fd_ < 0) {
        std::cerr << "I2C device not open" << std::endl;
        return false;
    }
    
    if (ioctl(i2c_fd_, I2C_SLAVE, devAddr) < 0) {
        std::cerr << "Failed to set I2C slave address" << std::endl;
        return false;
    }
    
    if (write(i2c_fd_, &reg, 1) != 1) {
        std::cerr << "Failed to write register address" << std::endl;
        return false;
    }
    
    if (read(i2c_fd_, buffer, length) != static_cast<ssize_t>(length)) {
        std::cerr << "Failed to read data" << std::endl;
        return false;
    }
    
    return true;
}



