// IMUDataTypes.h
#ifndef IMU_DATA_TYPES_H
#define IMU_DATA_TYPES_H

#include <chrono>

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

#endif // IMU_DATA_TYPES_H