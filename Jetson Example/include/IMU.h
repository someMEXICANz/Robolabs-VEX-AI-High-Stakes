#ifndef IMU_H
#define IMU_H

#include <string>
#include <cstring>
#include <cstdint>
#include <thread>
#include <mutex>
#include <chrono>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>



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

struct IMUData 
{
    float ax, ay, az;       // Accelerometer (g)
    float gx, gy, gz;       // Gyroscope (dps)
    float mx, my, mz;       // Magnetometer (gauss)
    float temperature;      // Temperature (°C)
    std::chrono::system_clock::time_point timestamp;
    bool valid;             // Flag indicating if data is valid
};

class IMU {
public:
    explicit IMU(const std::string& i2cBus = "/dev/i2c-1");     // Constructor
    ~IMU();                                                     // Destructor

    // Delete copy constructor and assignment operator
    IMU(const IMU&) = delete;
    IMU& operator=(const IMU&) = delete;

    // Core operations
    bool initialize();
    bool start();
    void stop();
    bool restart();
    bool reconnect();
    
    // Status Checks
    bool isRunning() const { return running; }
    bool isConnected() const { return i2c_fd >= 0; }
    
    // Calibration methods
    bool calibrateAccelerometer();
    bool calibrateGyroscope();
    bool calibrateMagnetometer();

    IMUData getIMUData() const;
        
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
    bool running;
    bool connected;
    IMUData current_data;



    

   
};

#endif // IMU_H