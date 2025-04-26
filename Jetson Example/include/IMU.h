#ifndef IMU_H
#define IMU_H

#include <string>
#include <cstring>
#include <cstdint>
#include <thread>
#include <mutex>
#include <chrono>
#include <iostream>
#include <deque>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <cmath>
#include <cfloat>
#include <algorithm>
#include <iomanip>
#include <LSM6DS3.h>
#include <LIS3MDL.h>


struct IMUData
{
  float ax, ay, az;  // Accelerometer (g)
  float gx, gy, gz;  // Gyroscope (dps)
  float mx, my, mz;  // Magnetometer (gauss)
  float temperature; // Temperature (°C)
  std::chrono::system_clock::time_point timestamp;
  bool valid; // Flag indicating if data is valid
};

struct OrientationData
{
  float roll;           // Rotation around X-axis (tilt left/right)
  float pitch;          // Rotation around Y-axis (tilt forward/backward)
  float yaw;            // Rotation around Z-axis (heading, compass direction)
  float qw, qx, qy, qz; // Quaternion
  std::chrono::system_clock::time_point timestamp;
  bool valid; // Flag indicating if data is valid
};

class IMU
{
  public:
    explicit IMU(const char* i2c_device = "/dev/i2c-1"); // Constructor
    ~IMU();                                                 // Destructor

    // Delete copy constructor and assignment operator
    IMU(const IMU &) = delete;
    IMU &operator=(const IMU &) = delete;

    // Core operations
    bool start();
    void stop();
    bool restart();
    bool initialize();

    // Status Checks
    bool isRunning() const { return running; }
    bool isInitialized() const { return initialized; }

    // Calibration methods
    bool calibrateAccelerometer();
    bool calibrateMagnetometer();

    bool isStationary(float threshold = 0.3f) const;

    void configureLSM6DS3();
    void configureLIS3MDL();
    


    IMUData getSensorData() const;
    OrientationData getOrientationData() const;


  private:

    void readLoop();
    bool readData();
    bool updateOrientation();

    bool writeRegister(int fd, uint8_t reg, uint8_t value);
    uint8_t readRegister(int fd, uint8_t reg);
    bool writeThenreadRegister(int fd, uint8_t reg, uint8_t* buffer, uint8_t length);

    // I2C device properties
    const char* i2c_device; 
    int lsm6ds3_fd;
    int lis3mdl_fd;
    std::string last_error;

    // Scale factors
    float accel_scale; // g per LSB
    float gyro_scale;  // dps per LSB
    float mag_scale;   // gauss per LSB

    // Thread and running state
    mutable std::mutex data_mutex;
    std::unique_ptr<std::thread> read_thread;
    bool running;
    bool initialized;
    IMUData current_data;
    OrientationData current_orientation;

    std::deque<IMUData> data_history;
    static constexpr size_t HISTORY_BUFFER_SIZE = 100;
    
    static constexpr float RAD_TO_DEG = 180.0f / M_PI;
    static constexpr float DEG_TO_RAD = M_PI / 180.0f;

};

#endif // IMU_H
