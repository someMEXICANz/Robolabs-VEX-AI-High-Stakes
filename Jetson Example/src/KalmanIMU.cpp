#include "KalmanIMU.h"




namespace KalmanFilter {

// OrientationSystemModel implementation
template<typename T, template<class> class CovarianceBase>
OrientationSystemModel<T, CovarianceBase>::OrientationSystemModel()
    : delta_t(0.01) // Default 10ms
{
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<typename T, template<class> class CovarianceBase>
void OrientationSystemModel<T, CovarianceBase>::updateJacobians(const State& x, const Control& u)
{
    // Initialize state transition matrix as identity
    this->F.setIdentity();
    
    // State transition: orientation changes based on gyro measurements minus bias
    this->F(0, 3) = -delta_t; // Roll affected by roll_bias
    this->F(1, 4) = -delta_t; // Pitch affected by pitch_bias
    this->F(2, 5) = -delta_t; // Yaw affected by yaw_bias
    
    // Process noise
    this->W.setIdentity();
}

template<typename T, template<class> class CovarianceBase>
typename OrientationSystemModel<T, CovarianceBase>::State 
OrientationSystemModel<T, CovarianceBase>::f(const State& x, const Control& u) const
{
    // Initialize the next state as the current one
    State x_next = x;
    
    // Update orientation based on gyro - bias
    x_next.roll() = x.roll() + delta_t * (u.gyro_x() - x.roll_bias());
    x_next.pitch() = x.pitch() + delta_t * (u.gyro_y() - x.pitch_bias());
    x_next.yaw() = x.yaw() + delta_t * (u.gyro_z() - x.yaw_bias());
    
    // Normalize yaw to [0, 2π]
    while (x_next.yaw() < 0) x_next.yaw() += 2 * M_PI;
    while (x_next.yaw() >= 2 * M_PI) x_next.yaw() -= 2 * M_PI;
    
    // Biases evolve as random walks (already handled by process noise)
    
    return x_next;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


// IMUMeasurementModel implementation
template<typename T, template<class> class CovarianceBase>
IMUMeasurementModel<T, CovarianceBase>::IMUMeasurementModel()
    : mag_field_x(1.0), mag_field_y(0.0), mag_field_z(0.0),
      mag_offset_x(0.0), mag_offset_y(0.0), mag_offset_z(0.0),
      mag_scale_x(1.0), mag_scale_y(1.0), mag_scale_z(1.0)
{
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<typename T, template<class> class CovarianceBase>
void IMUMeasurementModel<T, CovarianceBase>::setMagneticField(T mx, T my, T mz)
{
    mag_field_x = mx;
    mag_field_y = my;
    mag_field_z = mz;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<typename T, template<class> class CovarianceBase>
void IMUMeasurementModel<T, CovarianceBase>::setMagneticOffsets(T offset_x, T offset_y, T offset_z)
{
    mag_offset_x = offset_x;
    mag_offset_y = offset_y;
    mag_offset_z = offset_z;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<typename T, template<class> class CovarianceBase>
void IMUMeasurementModel<T, CovarianceBase>::setMagneticScaling(T scale_x, T scale_y, T scale_z)
{
    mag_scale_x = scale_x;
    mag_scale_y = scale_y;
    mag_scale_z = scale_z;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<typename T, template<class> class CovarianceBase>
void IMUMeasurementModel<T, CovarianceBase>::updateJacobians(const State& x)
{
    // Initialize measurement Jacobian as zero matrix
    this->H.setZero();
    
    // Compute trig functions of Euler angles for convenience
    T cr = std::cos(x.roll());
    T sr = std::sin(x.roll());
    T cp = std::cos(x.pitch());
    T sp = std::sin(x.pitch());
    
    // Accelerometer Jacobian (partial derivatives of gravity vector w.r.t. roll and pitch)
    
    // For ax = -g*sin(pitch)
    this->H(0, 1) = -GRAVITY * cp;
    
    // For ay = g*sin(roll)*cos(pitch)
    this->H(1, 0) = GRAVITY * cr * cp;
    this->H(1, 1) = -GRAVITY * sr * sp;
    
    // For az = g*cos(roll)*cos(pitch)
    this->H(2, 0) = -GRAVITY * sr * cp;
    this->H(2, 1) = -GRAVITY * cr * sp;
    
    // Magnetometer Jacobian
    // For simplicity, we'll just focus on how yaw affects the x and y magnetometer readings
    // This is a simplified model - a more complete model would account for roll and pitch effects too
    T cy = std::cos(x.yaw());
    T sy = std::sin(x.yaw());
    
    this->H(3, 2) = -mag_field_y * cy - mag_field_x * sy; // Derivative of mx w.r.t. yaw
    this->H(4, 2) = -mag_field_y * sy + mag_field_x * cy; // Derivative of my w.r.t. yaw
    
    // V = measurement noise transformation (identity)
    this->V.setIdentity();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<typename T, template<class> class CovarianceBase>
typename IMUMeasurementModel<T, CovarianceBase>::Measurement 
IMUMeasurementModel<T, CovarianceBase>::h(const State& x) const
{
    Measurement z;
    
    // Compute trig functions
    T cr = std::cos(x.roll());
    T sr = std::sin(x.roll());
    T cp = std::cos(x.pitch());
    T sp = std::sin(x.pitch());
    T cy = std::cos(x.yaw());
    T sy = std::sin(x.yaw());
    
    // Expected accelerometer readings (gravity vector in body frame)
    z.accel_x() = -GRAVITY * sp;
    z.accel_y() = GRAVITY * sr * cp;
    z.accel_z() = GRAVITY * cr * cp;
    
    // Expected magnetometer readings (Earth's magnetic field in body frame)
    // Rotation matrix from NED frame to body frame
    T R11 = cp * cy;
    T R12 = cp * sy;
    T R13 = -sp;
    T R21 = sr * sp * cy - cr * sy;
    T R22 = sr * sp * sy + cr * cy;
    T R23 = sr * cp;
    T R31 = cr * sp * cy + sr * sy;
    T R32 = cr * sp * sy - sr * cy;
    T R33 = cr * cp;
    
    // Calibrated magnetic field vector in body frame
    z.mag_x() = (R11 * mag_field_x + R12 * mag_field_y + R13 * mag_field_z) * mag_scale_x + mag_offset_x;
    z.mag_y() = (R21 * mag_field_x + R22 * mag_field_y + R23 * mag_field_z) * mag_scale_y + mag_offset_y;
    z.mag_z() = (R31 * mag_field_x + R32 * mag_field_y + R33 * mag_field_z) * mag_scale_z + mag_offset_z;
    
    return z;
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////




// Instantiate the templates
template class OrientationSystemModel<float>;
template class IMUMeasurementModel<float>;

// KalmanIMU implementation
KalmanIMU::KalmanIMU()
    : mag_offset_x(0.0f), mag_offset_y(0.0f), mag_offset_z(0.0f),
      mag_scale_x(1.0f), mag_scale_y(1.0f), mag_scale_z(1.0f),
      initialized(false)
{
    // Initialize orientation to zeros
    current_orientation.roll = 0.0f;
    current_orientation.pitch = 0.0f;
    current_orientation.yaw = 0.0f;
    current_orientation.qw = 1.0f;
    current_orientation.qx = 0.0f;
    current_orientation.qy = 0.0f;
    current_orientation.qz = 0.0f;
    current_orientation.timestamp = std::chrono::system_clock::now();
    current_orientation.valid = false;
    
    // Set initial timestamp
    last_update_time = std::chrono::system_clock::now();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


KalmanIMU::~KalmanIMU()
{
}

bool KalmanIMU::initialize()
{
    try {
        // Create models
        system_model = std::make_unique<OrientationSystemModel<float>>();
        measurement_model = std::make_unique<IMUMeasurementModel<float>>();
        
        // Create EKF
        ekf = std::make_unique<Kalman::ExtendedKalmanFilter<OrientationState<float>>>();
        
        // Initialize state
        OrientationState<float> initial_state;
        initial_state.roll() = 0.0f;
        initial_state.pitch() = 0.0f;
        initial_state.yaw() = 0.0f;
        initial_state.roll_bias() = 0.0f;
        initial_state.pitch_bias() = 0.0f;
        initial_state.yaw_bias() = 0.0f;
        
        // Set initial state
        ekf->init(initial_state);
        
        // Set initial process noise
        setProcessNoise(0.01f, 0.02f);
        // Set initial measurement noise
        setMeasurementNoise(0.01f, 0.03f); 
        // Set Earth's magnetic field (north-aligned by default)
        measurement_model->setMagneticField(1.0f, 0.0f, 0.0f);
        
        // Set magnetometer calibration
        measurement_model->setMagneticOffsets(mag_offset_x, mag_offset_y, mag_offset_z);
        measurement_model->setMagneticScaling(mag_scale_x, mag_scale_y, mag_scale_z);
        
        initialized = true;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error initializing Kalman filter: " << e.what() << std::endl;
        initialized = false;
        return false;
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void KalmanIMU::update(const IMUData& imu_data, bool useMagnetometer)
{
    if (!initialized || !imu_data.valid) 
    {
        current_orientation.valid = false;
        return;
    }
    
    // Calculate time delta
    auto current_time = imu_data.timestamp;
    float dt = std::chrono::duration<float>(current_time - last_update_time).count();
    last_update_time = current_time;
    
    // Update system model with the time delta
    system_model->setDeltaT(dt);
    
    // Create control input from gyro data (converting from degrees to radians)
    GyroInput<float> u;
    u.gyro_x() = imu_data.gx * DEG_TO_RAD;
    u.gyro_y() = imu_data.gy * DEG_TO_RAD;
    u.gyro_z() = imu_data.gz * DEG_TO_RAD;
    
    // Predict next state
    ekf->predict(*system_model, u);
    
    if (useMagnetometer) {
        // Create measurement from accel and mag data
        IMUMeasurement<float> z;
        z.accel_x() = imu_data.ax;
        z.accel_y() = imu_data.ay;
        z.accel_z() = imu_data.az;
        
        // Apply magnetometer calibration
        z.mag_x() = (imu_data.mx - mag_offset_x) * mag_scale_x;
        z.mag_y() = (imu_data.my - mag_offset_y) * mag_scale_y;
        z.mag_z() = (imu_data.mz - mag_offset_z) * mag_scale_z;
        
        // IMPROVEMENT 1: Calculate the magnetometer-based heading
        float mag_heading = std::atan2(z.mag_y(), z.mag_x());
        
        // Get current Kalman heading
        const auto& state = ekf->getState();
        float kalman_heading = state.yaw();
        
        // Calculate heading difference (accounting for wrap-around)
        float heading_diff = mag_heading - kalman_heading;
        while (heading_diff > M_PI) heading_diff -= 2.0f * M_PI;
        while (heading_diff < -M_PI) heading_diff += 2.0f * M_PI;
        
        // IMPROVEMENT 2: Save original measurement noise
        Eigen::Matrix<float, 6, 6> R_original = measurement_model->getCovariance();
        Eigen::Matrix<float, 6, 6> R_temp = R_original;
        
        // IMPROVEMENT 3: Adaptive measurement noise - trust magnetometer more when diff is large
        if (std::abs(heading_diff) > 0.5f) { // More than ~30 degrees difference
            // Significantly reduce magnetometer noise to force faster convergence
            R_temp(3, 3) = 0.1f;  // Much lower variance for mag_x
            R_temp(4, 4) = 0.1f;  // Much lower variance for mag_y
            
            // Apply temporary covariance
            measurement_model->setCovariance(R_temp);
            
            // Optional: Debug output
            // std::cerr << "Large heading diff detected: " << (heading_diff * RAD_TO_DEG) 
            //          << " degrees. Boosting mag trust." << std::endl;
        }
        
        // Update with measurements
        ekf->update(*measurement_model, z);
        
        // Restore original covariance if it was changed
        if (std::abs(heading_diff) > 0.5f) {
            measurement_model->setCovariance(R_original);
        }
        
        // IMPROVEMENT 4: For very large changes, consider a partial direct update
        if (std::abs(heading_diff) > 1.2f) { // More than ~70 degrees
            // Get updated state after Kalman update
            auto updated_state = ekf->getState();
            
            // Apply a complementary filter approach for heading
            float alpha = 0.3f; // Blend factor - adjust as needed
            float blended_yaw = kalman_heading + alpha * heading_diff;
            
            // Update the state with the blended yaw
            updated_state.yaw() = blended_yaw;
            
            // Reinitialize the filter with the modified state
            ekf->init(updated_state);
            
            // Optional: Debug output
            // std::cerr << "Very large heading diff: " << (heading_diff * RAD_TO_DEG) 
            //          << " degrees. Applying complementary filter." << std::endl;
        }
    } 
    else 
    {
        // Only use accelerometer data, create a custom update with higher uncertainty for yaw
        IMUMeasurement<float> z;
        z.accel_x() = imu_data.ax;
        z.accel_y() = imu_data.ay;
        z.accel_z() = imu_data.az;
        
        // Set magnetometer values to zero with very high variance (will be ignored)
        z.mag_x() = 0.0f;
        z.mag_y() = 0.0f;
        z.mag_z() = 0.0f;
        
        // Save original magnetic noise setting
        Eigen::Matrix<float, 6, 6> R = measurement_model->getCovariance();
        
        // Create a temporary higher noise matrix for magnetometer readings
        Eigen::Matrix<float, 6, 6> R_temp = R;
        R_temp(3, 3) = 1000.0f;  // Very high variance for mag_x
        R_temp(4, 4) = 1000.0f;  // Very high variance for mag_y
        R_temp(5, 5) = 1000.0f;  // Very high variance for mag_z
        
        // Set the temporary covariance
        measurement_model->setCovariance(R_temp);
        
        // Update with only accelerometer measurements
        ekf->update(*measurement_model, z);
        
        // Restore original covariance
        measurement_model->setCovariance(R);
    }
    
    // Get filtered state
    const auto& state = ekf->getState();
    
    // Update orientation data
    current_orientation.roll = state.roll() * RAD_TO_DEG;
    current_orientation.pitch = state.pitch() * RAD_TO_DEG;
    current_orientation.yaw = state.yaw() * RAD_TO_DEG;
    
    // Update quaternion
    eulerToQuaternion(state.roll(), state.pitch(), state.yaw(),
                      current_orientation.qw, current_orientation.qx, 
                      current_orientation.qy, current_orientation.qz);
    
    current_orientation.timestamp = current_time;
    current_orientation.valid = true;
}


// Add this method to your KalmanIMU class
void KalmanIMU::setYaw(float yaw_degrees)
{
    if (!initialized) {
        std::cerr << "Cannot set yaw: KalmanIMU not initialized" << std::endl;
        return;
    }
    
    // Get current state
    OrientationState<float> current_state = ekf->getState();
    
    // Convert desired yaw to radians
    float desired_yaw = yaw_degrees * DEG_TO_RAD;
    
    // Normalize to range [0, 2π]
    while (desired_yaw < 0) desired_yaw += 2.0f * M_PI;
    while (desired_yaw >= 2.0f * M_PI) desired_yaw -= 2.0f * M_PI;
    
    // Update the yaw in the state vector
    current_state.yaw() = desired_yaw;
    
    // Reinitialize the filter with the modified state
    ekf->init(current_state);
    
    // Update the orientation data
    current_orientation.yaw = yaw_degrees;
    eulerToQuaternion(current_state.roll(), current_state.pitch(), current_state.yaw(),
                      current_orientation.qw, current_orientation.qx, 
                      current_orientation.qy, current_orientation.qz);
    
    std::cout << "Yaw reference set to " << yaw_degrees << " degrees" << std::endl;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


OrientationData KalmanIMU::getOrientation() const
{
    return current_orientation;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void KalmanIMU::setProcessNoise(float gyro_noise, float gyro_bias_noise)
{
    if (!initialized) return;
    
    // Create process noise covariance matrix
    Eigen::Matrix<float, 6, 6> Q = Eigen::Matrix<float, 6, 6>::Zero();
    
    // Gyro noise affects orientation states
    Q(0, 0) = gyro_noise;
    Q(1, 1) = gyro_noise;
    Q(2, 2) = gyro_noise;
    
    // Gyro bias noise affects bias states
    Q(3, 3) = gyro_bias_noise;
    Q(4, 4) = gyro_bias_noise;
    Q(5, 5) = gyro_bias_noise;
    
    system_model->setCovariance(Q);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void KalmanIMU::setMeasurementNoise(float accel_noise, float mag_noise)
{
    if (!initialized) return;
    
    // Create measurement noise covariance matrix
    Eigen::Matrix<float, 6, 6> R = Eigen::Matrix<float, 6, 6>::Zero();
    
    // Accelerometer noise
    R(0, 0) = accel_noise;
    R(1, 1) = accel_noise;
    R(2, 2) = accel_noise;
    
    // Magnetometer noise
    R(3, 3) = mag_noise;
    R(4, 4) = mag_noise;
    R(5, 5) = mag_noise;
    
    measurement_model->setCovariance(R);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void KalmanIMU::eulerToQuaternion(float roll, float pitch, float yaw, 
                                float& qw, float& qx, float& qy, float& qz) const
{
    // Calculate trig functions once
    float cy = std::cos(yaw * 0.5f);
    float sy = std::sin(yaw * 0.5f);
    float cp = std::cos(pitch * 0.5f);
    float sp = std::sin(pitch * 0.5f);
    float cr = std::cos(roll * 0.5f);
    float sr = std::sin(roll * 0.5f);
    
    // Calculate quaternion components
    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void KalmanIMU::setMagneticOffsets(float x, float y, float z)
{
    mag_offset_x = x;
    mag_offset_y = y;
    mag_offset_z = z;
    
    if (initialized && measurement_model) {
        measurement_model->setMagneticOffsets(x, y, z);
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void KalmanIMU::setMagneticScaling(float x, float y, float z)
{
    mag_scale_x = x;
    mag_scale_y = y;
    mag_scale_z = z;
    
    if (initialized && measurement_model) {
        measurement_model->setMagneticScaling(x, y, z);
    }
}





} // namespace KalmanFilter
















// void KalmanIMU::update(const IMUData& imu_data, bool useMagnetometer)
// {
//     // std::cerr << "Test5" << std::endl;

//     if (!initialized || !imu_data.valid) 
//     {
//         current_orientation.valid = false;
//         return;
//     }
    
//     // std::cerr << "Test6" << std::endl;

//     // Calculate time delta
//     auto current_time = imu_data.timestamp;
//     float dt = std::chrono::duration<float>(current_time - last_update_time).count();
//     last_update_time = current_time;
    
//     // Update system model with the time delta
//     system_model->setDeltaT(dt);
    
//     // Create control input from gyro data (converting from degrees to radians)
//     GyroInput<float> u;
//     u.gyro_x() = imu_data.gx * DEG_TO_RAD;
//     u.gyro_y() = imu_data.gy * DEG_TO_RAD;
//     u.gyro_z() = imu_data.gz * DEG_TO_RAD;
    
//     // Predict next state
//     ekf->predict(*system_model, u);
    
//     if (useMagnetometer) {
//         // Create measurement from accel and mag data
//         IMUMeasurement<float> z;
//         z.accel_x() = imu_data.ax;
//         z.accel_y() = imu_data.ay;
//         z.accel_z() = imu_data.az;
        
//         // Apply magnetometer calibration
//         z.mag_x() = (imu_data.mx - mag_offset_x) * mag_scale_x;
//         z.mag_y() = (imu_data.my - mag_offset_y) * mag_scale_y;
//         z.mag_z() = (imu_data.mz - mag_offset_z) * mag_scale_z;
        
//         // Update with measurements
//         ekf->update(*measurement_model, z);
//     } 
//     else 
//     {
//         // Only use accelerometer data, create a custom update with higher uncertainty for yaw
//         IMUMeasurement<float> z;
//         z.accel_x() = imu_data.ax;
//         z.accel_y() = imu_data.ay;
//         z.accel_z() = imu_data.az;
        
//         // Set magnetometer values to zero with very high variance (will be ignored)
//         z.mag_x() = 0.0f;
//         z.mag_y() = 0.0f;
//         z.mag_z() = 0.0f;
        
//         // Save original magnetic noise setting
//         Eigen::Matrix<float, 6, 6> R = measurement_model->getCovariance();
        
//         // Create a temporary higher noise matrix for magnetometer readings
//         Eigen::Matrix<float, 6, 6> R_temp = R;
//         R_temp(3, 3) = 1000.0f;  // Very high variance for mag_x
//         R_temp(4, 4) = 1000.0f;  // Very high variance for mag_y
//         R_temp(5, 5) = 1000.0f;  // Very high variance for mag_z
        
//         // Set the temporary covariance
//         measurement_model->setCovariance(R_temp);
        
//         // Update with only accelerometer measurements
//         ekf->update(*measurement_model, z);
        
//         // Restore original covariance
//         measurement_model->setCovariance(R);
//     }
    
//     // Get filtered state
//     const auto& state = ekf->getState();
    
//     // Update orientation data
//     current_orientation.roll = state.roll() * RAD_TO_DEG;
//     current_orientation.pitch = state.pitch() * RAD_TO_DEG;
//     current_orientation.yaw = state.yaw() * RAD_TO_DEG;
    
//     // Update quaternion
//     eulerToQuaternion(state.roll(), state.pitch(), state.yaw(),
//                       current_orientation.qw, current_orientation.qx, 
//                       current_orientation.qy, current_orientation.qz);
    
//     current_orientation.timestamp = current_time;
//     current_orientation.valid = true;
// }

