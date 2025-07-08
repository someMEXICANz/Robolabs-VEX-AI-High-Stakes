// BNO085.cpp
#include "BNO085.h"
#include "I2C_HAL.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <chrono>

// =============================================================================
// UTILITY IMPLEMENTATIONS
// =============================================================================

float BNO085::Vector3::magnitude() const {
    return std::sqrt(x*x + y*y + z*z);
}

BNO085::Vector3 BNO085::Vector3::normalized() const {
    float mag = magnitude();
    if (mag > 0) {
        return Vector3(x/mag, y/mag, z/mag);
    }
    return Vector3();
}

std::string BNO085::Vector3::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) 
        << "(" << x << ", " << y << ", " << z << ")";
    return oss.str();
}

BNO085::Vector3 BNO085::Quaternion::toEulerAngles() const {
    float yaw, pitch, roll;
    q_to_ypr(w, x, y, z, &yaw, &pitch, &roll);
    
    // Convert from radians to degrees
    return Vector3(yaw * 180.0f / M_PI, 
                   pitch * 180.0f / M_PI, 
                   roll * 180.0f / M_PI);
}

std::string BNO085::Quaternion::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) 
        << "w=" << w << " x=" << x << " y=" << y << " z=" << z;
    if (accuracy > 0) {
        oss << " (acc=" << accuracy << ")";
    }
    return oss.str();
}

// =============================================================================
// CONSTRUCTOR/DESTRUCTOR
// =============================================================================

BNO085::BNO085() 
    : BNO085("/dev/i2c-1", 0x4A) {
}

BNO085::BNO085(const std::string& i2c_bus, uint8_t i2c_address)
    : hal_(nullptr)
    , i2c_bus_(i2c_bus)
    , i2c_address_(i2c_address)
    , initialized_(false)
    , service_running_(false) {
}

BNO085::~BNO085() {
    shutdown();
}

// =============================================================================
// INITIALIZATION AND CONTROL
// =============================================================================

bool BNO085::initialize() {
    if (initialized_) {
        return true;
    }
    
    std::cerr << "Initializing BNO085 sensor..." << std::endl;
    
    // Create HAL
    hal_ = createI2CHAL(i2c_bus_.c_str(), i2c_address_);
    if (!hal_) {
        setError("Failed to create I2C HAL");
        return false;
    }
    
    // Open SH2 library
    int result = sh2_open(hal_, eventCallbackWrapper, this);
    if (result != SH2_OK) {
        setError("Failed to open SH2 library: " + sh2ErrorToString(result));
        destroyI2CHAL(hal_);
        hal_ = nullptr;
        return false;
    }
    
    // Set sensor callback
    sh2_setSensorCallback(sensorCallbackWrapper, this);
    
    // Wait a moment for initialization
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    initialized_ = true;
    std::cerr << "BNO085 initialized successfully" << std::endl;
    
    return true;
}

void BNO085::shutdown() {
    if (!initialized_) {
        return;
    }
    
    std::cerr << "Shutting down BNO085..." << std::endl;
    
    // Stop service thread first
    stopService();
    
    // Close SH2 library
    sh2_close();
    
    // Destroy HAL
    if (hal_) {
        destroyI2CHAL(hal_);
        hal_ = nullptr;
    }
    
    initialized_ = false;
    std::cerr << "BNO085 shutdown complete" << std::endl;
}

void BNO085::startService() {
    if (service_running_ || !initialized_) {
        return;
    }
    
    service_running_ = true;
    service_thread_ = std::make_unique<std::thread>(&BNO085::serviceLoop, this);
    
    std::cerr << "BNO085 service thread started" << std::endl;
}

void BNO085::stopService() {
    if (!service_running_) {
        return;
    }
    
    service_running_ = false;
    
    if (service_thread_ && service_thread_->joinable()) {
        service_thread_->join();
    }
    service_thread_.reset();
    
    std::cerr << "BNO085 service thread stopped" << std::endl;
}

// =============================================================================
// SENSOR CONFIGURATION
// =============================================================================

bool BNO085::enableSensor(SensorType sensor, const SensorConfig& config) {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    sh2_SensorId_t sensorId = sensorTypeToId(sensor);
    if (sensorId == 0xFF) {
        setError("Invalid sensor type");
        return false;
    }
    
    sh2_SensorConfig_t sh2Config = {};
    sh2Config.reportInterval_us = config.reportInterval_us;
    sh2Config.changeSensitivityEnabled = false;
    sh2Config.wakeupEnabled = config.wakeupEnabled;
    sh2Config.alwaysOnEnabled = config.alwaysOnEnabled;
    sh2Config.sniffEnabled = false;
    sh2Config.changeSensitivity = 0;
    sh2Config.batchInterval_us = 0;
    sh2Config.sensorSpecific = 0;
    
    int result = sh2_setSensorConfig(sensorId, &sh2Config);
    if (result != SH2_OK) {
        setError("Failed to configure sensor: " + sh2ErrorToString(result));
        return false;
    }
    
    std::cerr << "Enabled " << sensorIdToString(sensorId) 
              << " at " << config.getFrequency() << " Hz" << std::endl;
    
    return true;
}

bool BNO085::disableSensor(SensorType sensor) {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    sh2_SensorId_t sensorId = sensorTypeToId(sensor);
    if (sensorId == 0xFF) {
        setError("Invalid sensor type");
        return false;
    }
    
    sh2_SensorConfig_t config = {};
    config.reportInterval_us = 0;  // Setting to 0 disables the sensor
    
    int result = sh2_setSensorConfig(sensorId, &config);
    if (result != SH2_OK) {
        setError("Failed to disable sensor: " + sh2ErrorToString(result));
        return false;
    }
    
    std::cerr << "Disabled " << sensorIdToString(sensorId) << std::endl;
    return true;
}

bool BNO085::enableBasicIMU() {
    SensorConfig config;
    config.setFrequency(100.0f);  // 100 Hz
    
    bool success = true;
    success &= enableSensor(SensorType::ACCELEROMETER, config);
    success &= enableSensor(SensorType::GYROSCOPE, config);
    success &= enableSensor(SensorType::MAGNETOMETER, config);
    
    if (success) {
        std::cerr << "Basic IMU sensors enabled" << std::endl;
    }
    
    return success;
}

bool BNO085::enableOrientation() {
    SensorConfig config;
    config.setFrequency(50.0f);  // 50 Hz
    
    bool success = enableSensor(SensorType::ROTATION_VECTOR, config);
    
    if (success) {
        std::cerr << "Orientation sensing enabled" << std::endl;
    }
    
    return success;
}

bool BNO085::getSensorConfig(SensorType sensor, SensorConfig& config) {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    sh2_SensorId_t sensorId = sensorTypeToId(sensor);
    if (sensorId == 0xFF) {
        setError("Invalid sensor type");
        return false;
    }
    
    sh2_SensorConfig_t sh2Config;
    int result = sh2_getSensorConfig(sensorId, &sh2Config);
    if (result != SH2_OK) {
        setError("Failed to get sensor config: " + sh2ErrorToString(result));
        return false;
    }
    
    config.reportInterval_us = sh2Config.reportInterval_us;
    config.enabled = (sh2Config.reportInterval_us > 0);
    config.wakeupEnabled = sh2Config.wakeupEnabled;
    config.alwaysOnEnabled = sh2Config.alwaysOnEnabled;
    
    return true;
}

// =============================================================================
// DATA READING
// =============================================================================

bool BNO085::getAcceleration(Vector3& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto it = latest_sensor_data_.find(SH2_ACCELEROMETER);
    if (it == latest_sensor_data_.end()) {
        return false;
    }
    
    const auto& accel = it->second.un.accelerometer;
    data = Vector3(accel.x, accel.y, accel.z);
    return true;
}

bool BNO085::getGyroscope(Vector3& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto it = latest_sensor_data_.find(SH2_GYROSCOPE_CALIBRATED);
    if (it == latest_sensor_data_.end()) {
        return false;
    }
    
    const auto& gyro = it->second.un.gyroscope;
    data = Vector3(gyro.x, gyro.y, gyro.z);
    return true;
}

bool BNO085::getMagnetometer(Vector3& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto it = latest_sensor_data_.find(SH2_MAGNETIC_FIELD_CALIBRATED);
    if (it == latest_sensor_data_.end()) {
        return false;
    }
    
    const auto& mag = it->second.un.magneticField;
    data = Vector3(mag.x, mag.y, mag.z);
    return true;
}

bool BNO085::getOrientation(Quaternion& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto it = latest_sensor_data_.find(SH2_ROTATION_VECTOR);
    if (it == latest_sensor_data_.end()) {
        return false;
    }
    
    const auto& rot = it->second.un.rotationVector;
    data = Quaternion(rot.real, rot.i, rot.j, rot.k, rot.accuracy);
    return true;
}

bool BNO085::getEulerAngles(float& yaw, float& pitch, float& roll) {
    Quaternion quat;
    if (!getOrientation(quat)) {
        return false;
    }
    
    Vector3 euler = quat.toEulerAngles();
    yaw = euler.x;
    pitch = euler.y;
    roll = euler.z;
    return true;
}

bool BNO085::getIMUData(IMUData& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data = latest_imu_;
    
    // Check if we have valid data for all IMU sensors
    return (latest_sensor_data_.count(SH2_ACCELEROMETER) > 0 &&
            latest_sensor_data_.count(SH2_GYROSCOPE_CALIBRATED) > 0 &&
            latest_sensor_data_.count(SH2_MAGNETIC_FIELD_CALIBRATED) > 0);
}

bool BNO085::getOrientationData(OrientationData& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data = latest_orientation_;
    
    return latest_sensor_data_.count(SH2_ROTATION_VECTOR) > 0;
}

// =============================================================================
// CALIBRATION
// =============================================================================

void BNO085::getCalibrationStatus(CalibrationStatus& accel, CalibrationStatus& gyro, 
                                 CalibrationStatus& mag, CalibrationStatus& system) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Get status from latest sensor readings
    auto getStatus = [this](sh2_SensorId_t id) -> CalibrationStatus {
        auto it = latest_sensor_data_.find(id);
        if (it != latest_sensor_data_.end()) {
            return static_cast<CalibrationStatus>(it->second.status & 0x03);
        }
        return CalibrationStatus::UNCALIBRATED;
    };
    
    accel = getStatus(SH2_ACCELEROMETER);
    gyro = getStatus(SH2_GYROSCOPE_CALIBRATED);
    mag = getStatus(SH2_MAGNETIC_FIELD_CALIBRATED);
    
    // System status is typically from rotation vector
    system = getStatus(SH2_ROTATION_VECTOR);
}

bool BNO085::isFullyCalibrated() {
    CalibrationStatus accel, gyro, mag, system;
    getCalibrationStatus(accel, gyro, mag, system);
    
    return (accel == CalibrationStatus::HIGH_ACCURACY &&
            gyro == CalibrationStatus::HIGH_ACCURACY &&
            mag == CalibrationStatus::HIGH_ACCURACY &&
            system == CalibrationStatus::HIGH_ACCURACY);
}

bool BNO085::startCalibration(uint32_t interval_us) {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    int result = sh2_startCal(interval_us);
    if (result != SH2_OK) {
        setError("Failed to start calibration: " + sh2ErrorToString(result));
        return false;
    }
    
    std::cerr << "Calibration started" << std::endl;
    return true;
}

bool BNO085::finishCalibration() {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    sh2_CalStatus_t status;
    int result = sh2_finishCal(&status);
    if (result != SH2_OK) {
        setError("Failed to finish calibration: " + sh2ErrorToString(result));
        return false;
    }
    
    if (status == SH2_CAL_SUCCESS) {
        std::cerr << "Calibration completed successfully" << std::endl;
        return true;
    } else {
        setError("Calibration failed with status: " + std::to_string(status));
        return false;
    }
}

bool BNO085::saveCalibration() {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    int result = sh2_saveDcdNow();
    if (result != SH2_OK) {
        setError("Failed to save calibration: " + sh2ErrorToString(result));
        return false;
    }
    
    std::cerr << "Calibration data saved" << std::endl;
    return true;
}

// =============================================================================
// DIAGNOSTICS AND STATUS
// =============================================================================

std::string BNO085::getProductInfo() {

    if (!initialized_) {
        return "Sensor not initialized";
    }
    
    sh2_ProductIds_t prodIds;
    int result = sh2_getProdIds(&prodIds);
    if (result != SH2_OK) {
        return "Failed to get product info: " + sh2ErrorToString(result);
    }
    
    std::ostringstream oss;
    oss << "BNO085 Product Information:\n";
    for (int i = 0; i < prodIds.numEntries; i++) {
        const auto& prod = prodIds.entry[i];
        oss << "  Entry " << i << ":\n";
        oss << "    Part Number: 0x" << std::hex << prod.swPartNumber << std::dec << "\n";
        oss << "    Version: " << static_cast<int>(prod.swVersionMajor) 
            << "." << static_cast<int>(prod.swVersionMinor) 
            << "." << prod.swVersionPatch << "\n";
        oss << "    Build: " << prod.swBuildNumber << "\n";
        oss << "    Reset Cause: " << static_cast<int>(prod.resetCause) << "\n";
    }
    
    return oss.str();
}

bool BNO085::isConnected() {
    if (!hal_) {
        return false;
    }
    
    return isI2CDevicePresent(hal_);
}

bool BNO085::reset() {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    int result = sh2_devReset();
    if (result != SH2_OK) {
        setError("Failed to reset sensor: " + sh2ErrorToString(result));
        return false;
    }
    
    std::cerr << "Sensor reset initiated" << std::endl;
    return true;
}

std::string BNO085::getSensorMetadata(SensorType sensor) {
    if (!initialized_) {
        return "Sensor not initialized";
    }
    
    sh2_SensorId_t sensorId = sensorTypeToId(sensor);
    if (sensorId == 0xFF) {
        return "Invalid sensor type";
    }
    
    sh2_SensorMetadata_t metadata;
    int result = sh2_getMetadata(sensorId, &metadata);
    if (result != SH2_OK) {
        return "Failed to get metadata: " + sh2ErrorToString(result);
    }
    
    std::ostringstream oss;
    oss << "Sensor Metadata for " << sensorIdToString(sensorId) << ":\n";
    oss << "  Range: " << metadata.range << "\n";
    oss << "  Resolution: " << metadata.resolution << "\n";
    oss << "  Power: " << (metadata.power_mA / 1024.0f) << " mA\n";
    oss << "  Min Period: " << metadata.minPeriod_uS << " µs\n";
    oss << "  Max Period: " << metadata.maxPeriod_uS << " µs\n";
    oss << "  Vendor: " << std::string(metadata.vendorId, metadata.vendorIdLen) << "\n";
    
    return oss.str();
}

// =============================================================================
// PRIVATE METHODS
// =============================================================================

void BNO085::setError(const std::string& error) {
    last_error_ = error;
    std::cerr << "BNO085 Error: " << error << std::endl;
    
    if (error_callback_) {
        error_callback_(error);
    }
}

sh2_SensorId_t BNO085::sensorTypeToId(SensorType sensor) {
    switch (sensor) {
        case SensorType::ACCELEROMETER:       return SH2_ACCELEROMETER;
        case SensorType::GYROSCOPE:           return SH2_GYROSCOPE_CALIBRATED;
        case SensorType::MAGNETOMETER:        return SH2_MAGNETIC_FIELD_CALIBRATED;
        case SensorType::ROTATION_VECTOR:     return SH2_ROTATION_VECTOR;
        case SensorType::GAME_ROTATION_VECTOR: return SH2_GAME_ROTATION_VECTOR;
        case SensorType::LINEAR_ACCELERATION: return SH2_LINEAR_ACCELERATION;
        case SensorType::GRAVITY:             return SH2_GRAVITY;
        default:                              return 0xFF;  // Invalid
    }
}

void BNO085::serviceLoop() {
    std::cerr << "BNO085 service loop started" << std::endl;
    
    while (service_running_) {
        if (initialized_) {
            // Service the SH2 library - this processes incoming data
            sh2_service();
        }
        
        // Small delay to prevent busy waiting
        std::this_thread::sleep_for(std::chrono::microseconds(1000));  // 1ms
    }
    
    std::cerr << "BNO085 service loop ended" << std::endl;
}

// =============================================================================
// SH2 CALLBACK HANDLERS (STATIC)
// =============================================================================

void BNO085::eventCallbackWrapper(void* cookie, sh2_AsyncEvent_t* event) {
    BNO085* instance = static_cast<BNO085*>(cookie);
    if (instance) {
        instance->handleAsyncEvent(event);
    }
}

void BNO085::sensorCallbackWrapper(void* cookie, sh2_SensorEvent_t* event) {
    BNO085* instance = static_cast<BNO085*>(cookie);
    if (instance) {
        instance->handleSensorEvent(event);
    }
}

// =============================================================================
// INSTANCE CALLBACK HANDLERS
// =============================================================================

void BNO085::handleAsyncEvent(sh2_AsyncEvent_t* event) {
    switch (event->eventId) {
        case SH2_RESET:
            std::cerr << "BNO085: Reset complete" << std::endl;
            break;
            
        case SH2_GET_FEATURE_RESP:
            std::cerr << "BNO085: Feature response for sensor " 
                      << static_cast<int>(event->sh2SensorConfigResp.sensorId) << std::endl;
            break;
            
        case SH2_SHTP_EVENT:
            //std::cerr << "BNO085: SHTP event " << event->shtpEvent << std::endl;
            break;
            
        default:
            std::cerr << "BNO085: Unknown async event " << event->eventId << std::endl;
            break;
    }
}

void BNO085::handleSensorEvent(sh2_SensorEvent_t* event) {
    sh2_SensorValue_t value;
    
    // Decode the sensor event
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
        setError("Failed to decode sensor event");
        return;
    }
    
    // Store the latest data
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_sensor_data_[value.sensorId] = value;
        
        // Update aggregate data structures
        updateIMUData(value);
        updateOrientationData(value);
    }
    
    // Call user callbacks
    if (sensor_callback_) {
        sensor_callback_(value.sensorId, value);
    }
}

void BNO085::updateIMUData(const sh2_SensorValue_t& value) {
    switch (value.sensorId) {
        case SH2_ACCELEROMETER:
            latest_imu_.acceleration = Vector3(
                value.un.accelerometer.x,
                value.un.accelerometer.y,
                value.un.accelerometer.z
            );
            latest_imu_.timestamp_us = value.timestamp;
            latest_imu_.status = value.status;
            latest_imu_.sequence = value.sequence;
            break;
            
        case SH2_GYROSCOPE_CALIBRATED:
            latest_imu_.gyroscope = Vector3(
                value.un.gyroscope.x,
                value.un.gyroscope.y,
                value.un.gyroscope.z
            );
            break;
            
        case SH2_MAGNETIC_FIELD_CALIBRATED:
            latest_imu_.magnetometer = Vector3(
                value.un.magneticField.x,
                value.un.magneticField.y,
                value.un.magneticField.z
            );
            break;
    }
    
    // Call IMU callback if we have complete data
    if (imu_callback_ && 
        latest_sensor_data_.count(SH2_ACCELEROMETER) > 0 &&
        latest_sensor_data_.count(SH2_GYROSCOPE_CALIBRATED) > 0 &&
        latest_sensor_data_.count(SH2_MAGNETIC_FIELD_CALIBRATED) > 0) {
        imu_callback_(latest_imu_);
    }
}

void BNO085::updateOrientationData(const sh2_SensorValue_t& value) {
    if (value.sensorId == SH2_ROTATION_VECTOR) {
        latest_orientation_.rotation = Quaternion(
            value.un.rotationVector.real,
            value.un.rotationVector.i,
            value.un.rotationVector.j,
            value.un.rotationVector.k,
            value.un.rotationVector.accuracy
        );
        
        latest_orientation_.euler = latest_orientation_.rotation.toEulerAngles();
        latest_orientation_.timestamp_us = value.timestamp;
        latest_orientation_.status = value.status;
        latest_orientation_.sequence = value.sequence;
        
        // Call orientation callback
        if (orientation_callback_) {
            orientation_callback_(latest_orientation_);
        }
    }
}