// KalmanIMU.h
#ifndef KALMAN_IMU_H
#define KALMAN_IMU_H

#include <Eigen/Dense>
#include <memory>
#include <chrono>
#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/LinearizedSystemModel.hpp>
#include <kalman/LinearizedMeasurementModel.hpp>
#include <IMUDataTypes.h>
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


namespace KalmanFilter {

// State vector definition for orientation (roll, pitch, yaw) and gyro bias
template <typename T>
class OrientationState : public Kalman::Vector<T, 6>
{
public:
    KALMAN_VECTOR(OrientationState, T, 6)
    
    // Accessors
    T roll() const { return (*this)[0]; }
    T& roll() { return (*this)[0]; }
    
    T pitch() const { return (*this)[1]; }
    T& pitch() { return (*this)[1]; }
    
    T yaw() const { return (*this)[2]; }
    T& yaw() { return (*this)[2]; }
    
    T roll_bias() const { return (*this)[3]; }
    T& roll_bias() { return (*this)[3]; }
    
    T pitch_bias() const { return (*this)[4]; }
    T& pitch_bias() { return (*this)[4]; }
    
    T yaw_bias() const { return (*this)[5]; }
    T& yaw_bias() { return (*this)[5]; }
};

// Control input vector (gyroscope measurements)
template <typename T>
class GyroInput : public Kalman::Vector<T, 3>
{
public:
    KALMAN_VECTOR(GyroInput, T, 3)
    
    // Accessors
    T gyro_x() const { return (*this)[0]; }
    T& gyro_x() { return (*this)[0]; }
    
    T gyro_y() const { return (*this)[1]; }
    T& gyro_y() { return (*this)[1]; }
    
    T gyro_z() const { return (*this)[2]; }
    T& gyro_z() { return (*this)[2]; }
};

// Measurement vector (accelerometer and magnetometer readings)
template <typename T>
class IMUMeasurement : public Kalman::Vector<T, 6>
{
public:
    KALMAN_VECTOR(IMUMeasurement, T, 6)
    
    // Accessors
    T accel_x() const { return (*this)[0]; }
    T& accel_x() { return (*this)[0]; }
    
    T accel_y() const { return (*this)[1]; }
    T& accel_y() { return (*this)[1]; }
    
    T accel_z() const { return (*this)[2]; }
    T& accel_z() { return (*this)[2]; }
    
    T mag_x() const { return (*this)[3]; }
    T& mag_x() { return (*this)[3]; }
    
    T mag_y() const { return (*this)[4]; }
    T& mag_y() { return (*this)[4]; }
    
    T mag_z() const { return (*this)[5]; }
    T& mag_z() { return (*this)[5]; }
};

// System model for orientation estimation
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class OrientationSystemModel : public Kalman::LinearizedSystemModel<OrientationState<T>, GyroInput<T>, CovarianceBase>
{
public:
    typedef Kalman::LinearizedSystemModel<OrientationState<T>, GyroInput<T>, CovarianceBase> Base;
    typedef typename Base::State State;
    typedef typename Base::Control Control;
    
    OrientationSystemModel();
    
    // Set the time delta between updates
    void setDeltaT(T dt) { delta_t = dt; }
    
    // Update Jacobians (linearization)
    void updateJacobians(const State& x, const Control& u) override;
    
    // Non-linear state transition function
    State f(const State& x, const Control& u) const override;
    
private:
    T delta_t; // Time delta between updates (seconds)
};

// Measurement model for IMU
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class IMUMeasurementModel : public Kalman::LinearizedMeasurementModel<OrientationState<T>, IMUMeasurement<T>, CovarianceBase>
{
public:
    typedef Kalman::LinearizedMeasurementModel<OrientationState<T>, IMUMeasurement<T>, CovarianceBase> Base;
    typedef typename Base::State State;
    typedef typename Base::Measurement Measurement;
    
    IMUMeasurementModel();
    
    // Set magnetic field calibration
    void setMagneticField(T mx, T my, T mz);
    
    // Set magnetic field calibration offsets (hard iron)
    void setMagneticOffsets(T offset_x, T offset_y, T offset_z);
    
    // Set magnetic field scale factors (soft iron)
    void setMagneticScaling(T scale_x, T scale_y, T scale_z);
    
    // Update Jacobians (linearization)
    void updateJacobians(const State& x) override;

 

    
    
    // Non-linear measurement function
    Measurement h(const State& x) const override;
    
private:
    // Earth's magnetic field components
    T mag_field_x;
    T mag_field_y;
    T mag_field_z;
    
    // Magnetometer calibration
    T mag_offset_x;
    T mag_offset_y;
    T mag_offset_z;
    T mag_scale_x;
    T mag_scale_y;
    T mag_scale_z;
    
    // Gravity constant
    static constexpr T GRAVITY = 9.81;
};

// Main Kalman filter class that ties everything together
class KalmanIMU
{
public:
    KalmanIMU();
    ~KalmanIMU();
    
    // Initialize the filter
    bool initialize();
    
    // Update the filter with new sensor data
    void update(const IMUData& imu_data, bool useMagnetometer = true);
    
    // Get filtered orientation
    OrientationData getOrientation() const;
    
    // Set process and measurement noise parameters
    void setProcessNoise(float gyro_noise, float gyro_bias_noise);
    void setMeasurementNoise(float accel_noise, float mag_noise);
    void setMagneticOffsets(float x, float y, float z);
    void setMagneticScaling(float x, float y, float z);
    void setYaw(float yaw_degrees);
    
private:
    // Convert Euler angles to quaternion
    void eulerToQuaternion(float roll, float pitch, float yaw, 
                          float& qw, float& qx, float& qy, float& qz) const;
    
    // Kalman filter and models
    std::unique_ptr<Kalman::ExtendedKalmanFilter<OrientationState<float>>> ekf;
    std::unique_ptr<OrientationSystemModel<float>> system_model;
    std::unique_ptr<IMUMeasurementModel<float>> measurement_model;
    
    // Last update timestamp
    std::chrono::system_clock::time_point last_update_time;
    
    // Filtered orientation data
    OrientationData current_orientation;
    
    // Magnetometer calibration values
    float mag_offset_x, mag_offset_y, mag_offset_z;
    float mag_scale_x, mag_scale_y, mag_scale_z;
    
    // Initialization flag
    bool initialized;
    
    // Constants
    static constexpr float DEG_TO_RAD = M_PI / 180.0f;
    static constexpr float RAD_TO_DEG = 180.0f / M_PI;
};

} // namespace KalmanFilter

#endif // KALMAN_IMU_H