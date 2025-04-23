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
#include <cmath>
#include <cfloat>

#define LSM6DS_I2CADDR_DEFAULT 0x6A  ///< LSM6DS default i2c address
#define LSM6DS_FUNC_CFG_ACCESS 0x1 ///< Enable embedded functions register
#define LSM6DS_INT1_CTRL 0x0D      ///< Interrupt control for INT 1
#define LSM6DS_INT2_CTRL 0x0E      ///< Interrupt control for INT 2
#define LSM6DS_WHOAMI 0x0F         ///< Chip ID register
#define LSM6DS_CTRL1_XL 0x10       ///< Main accelerometer config register
#define LSM6DS_CTRL2_G 0x11        ///< Main gyro config register
#define LSM6DS_CTRL3_C 0x12        ///< Main configuration register
#define LSM6DS_CTRL8_XL 0x17       ///< High and low pass for accel
#define LSM6DS_CTRL10_C 0x19       ///< Main configuration register
#define LSM6DS_WAKEUP_SRC 0x1B     ///< Why we woke up
#define LSM6DS_STATUS_REG 0X1E     ///< Status register
#define LSM6DS_OUT_TEMP_L 0x20     ///< First data register (temperature low)
#define LSM6DS_OUTX_L_G 0x22       ///< First gyro data register
#define LSM6DS_OUTX_L_A 0x28       ///< First accel data register
#define LSM6DS_STEPCOUNTER 0x4B    ///< 16-bit step counter
#define LSM6DS_WAKEUP_THS 0x5B     ///< Single and double-tap function threshold register
#define LSM6DS_WAKEUP_DUR 0x5C     ///< Free-fall, wakeup, timestamp and sleep mode duration
#define LSM6DS_MD1_CFG 0x5E        ///< Functions routing on INT1 register


#define LIS3MDL_I2CADDR_DEFAULT 0x1C ///< LIS3MDL default i2c addres
#define LIS3MDL_REG_WHO_AM_I 0x0F  ///< Register that contains the part ID
#define LIS3MDL_REG_CTRL_REG1 0x20 ///< Register address for control 1
#define LIS3MDL_REG_CTRL_REG2 0x21 ///< Register address for control 2
#define LIS3MDL_REG_CTRL_REG3 0x22 ///< Register address for control 3
#define LIS3MDL_REG_CTRL_REG4 0x23 ///< Register address for control 3
#define LIS3MDL_REG_STATUS 0x27    ///< Register address for status
#define LIS3MDL_REG_OUT_X_L 0x28   ///< Register address for X axis lower byte
#define LIS3MDL_REG_INT_CFG 0x30   ///< Interrupt configuration register
#define LIS3MDL_REG_INT_THS_L 0x32 ///< Low byte of the irq threshold

enum class LSM6DS_DATA_RATE
{
  RATE_SHUTDOWN,
  RATE_12_5_HZ,
  RATE_26_HZ,
  RATE_52_HZ,
  RATE_104_HZ,
  RATE_208_HZ,
  RATE_416_HZ,
  RATE_833_HZ,
  RATE_1_66K_HZ,
  RATE_3_33K_HZ,
  RATE_6_66K_HZ,
};

enum class ACCEL_RAMGE
{
  ACCEL_RANGE_2_G,    
  ACCEL_RANGE_4_G,     
  ACCEL_RANGE_8_G,   
  ACCEL_RANGE_16_G      
};

enum class GYRO_RANGE
{
  GYRO_RANGE_125_DPS,
  GYRO_RANGE_250_DPS,
  GYRO_RANGE_500_DPS,
  GYRO_RANGE_1000_DPS,
  GYRO_RANGE_2000_DPS 
};

enum class LSM6DS_HPF_RAMGE
{
  HPF_ODR_DIV_50,         ///< Cutoff = ODR/50
  HPF_ODR_DIV_100,        ///< Cutoff = ODR/100
  HPF_ODR_DIV_9,          ///< Cutoff = ODR/9
  HPF_ODR_DIV_400         ///< Cutoff = ODR/400
};

enum class  LIS3MDL_DATA_RATE
{
  RATE_0_625_HZ,            ///<  0.625 Hz
  RATE_1_25_HZ,             ///<  1.25 Hz
  RATE_2_5_HZ,              ///<  2.5 Hz
  RATE_5_HZ,                ///<  5 Hz
  RATE_10_HZ,               ///<  10 Hz
  RATE_20_HZ,               ///<  20 Hz
  RATE_40_HZ,               ///<  40 Hz
  RATE_80_HZ,               ///<  80 Hz
  RATE_155_HZ,              ///<  155 Hz (FAST_ODR + UHP)
  RATE_300_HZ,              ///<  300 Hz (FAST_ODR + HP)
  RATE_560_HZ,              ///<  560 Hz (FAST_ODR + MP)
  RATE_1000_HZ,             ///<  1000 Hz (FAST_ODR + LP)
};

enum class MAG_RANGE
{
  LIS3MDL_RANGE_4_GAUSS,
  LIS3MDL_RANGE_8_GAUSS,
  LIS3MDL_RANGE_12_GAUSS,
  LIS3MDL_RANGE_16_GAUSS
};


enum class LIS3MDL_PERF_MODE
{
  LIS3MDL_LOWPOWERMODE,               ///< Low power mode
  LIS3MDL_MEDIUMMODE,                 ///< Medium performance mode
  LIS3MDL_HIGHMODE,                   ///< High performance mode
  LIS3MDL_ULTRAHIGHMODE               ///< Ultra-high performance mode
};


enum class LIS3MDL_OPER_MODE
{
  LIS3MDL_CONTINUOUSMODE,             ///< Continuous conversion
  LIS3MDL_SINGLEMODE,                 ///< Single-shot conversion
  LIS3MDL_POWERDOWNMODE               ///< Powered-down mode
};

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
  float heading;        // Compass heading (0-360 degrees, 0 = North)
  std::chrono::system_clock::time_point timestamp;
  bool valid; // Flag indicating if data is valid
};

class IMU
{
public:
  explicit IMU(const std::string &i2cBus = "/dev/i2c-1"); // Constructor
  ~IMU();                                                 // Destructor

  // Delete copy constructor and assignment operator
  IMU(const IMU &) = delete;
  IMU &operator=(const IMU &) = delete;

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

  void setHeading(float abs_angle);
  bool isStationary(float threshold = 0.3f) const;
  void configureRanges(ACCEL_RAMGE a_range, GYRO_RANGE g_range, MAG_RANGE m_range);
  void configureLIS3MDL(LIS3MDL_DATA_RATE data_rate, 
                        LIS3MDL_PERF_MODE p_mode, 
                        LIS3MDL_OPER_MODE op_mode);
  void configureLSM6DS(LSM6DS_DATA_RATE data_rate,
                       LSM6DS_HPF_RAMGE hpf_range);


  IMUData getSensorData() const;
  OrientationData getOrientationData() const;

  // Error handling
  std::string getLastError() const { return last_error; }

private:
  void readLoop();
  bool readData();
  bool updateOrientation();

  // I2C communication
  bool writeByte(uint8_t devAddr, uint8_t reg, uint8_t data);
  bool readBytes(uint8_t devAddr, uint8_t reg, uint8_t *buffer, size_t length);

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

  float heading_offset;

  // Scale factors
  float accel_scale; // g per LSB
  float gyro_scale;  // dps per LSB
  float mag_scale;   // gauss per LSB

  // Thread and running state
  mutable std::mutex data_mutex;
  std::unique_ptr<std::thread> read_thread;
  bool running;
  bool connected;
  IMUData current_data;
  OrientationData current_orientation;

  static constexpr size_t HISTORY_BUFFER_SIZE = 100;
  std::deque<IMUData> data_history;

  static constexpr float RAD_TO_DEG = 180.0f / M_PI;
  static constexpr float DEG_TO_RAD = M_PI / 180.0f;
};

#endif // IMU_H
