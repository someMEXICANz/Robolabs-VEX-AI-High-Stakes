#include "BNO085.h"
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <cstring>

// Global pointer for HAL callbacks (single instance support)
static BNO085* g_bno085_instance = nullptr;

BNO085::BNO085(const std::string& i2c_device_path, int8_t reset_pin)
    : i2c_device(i2c_device_path, BNO085_I2C_ADDR),
      reset_pin(reset_pin),
      running(false),
      initialized(false),
      update_rate_hz(DEFAULT_UPDATE_RATE_HZ),
      accel_enabled(false),
      gyro_enabled(false),
      mag_enabled(false),
      rotation_vector_enabled(false) {
    
    // Set up HAL structure
    hal.open = hal_open;
    hal.close = hal_close;
    hal.read = hal_read;
    hal.write = hal_write;
    hal.getTimeUs = hal_getTimeUs;
    
    // Store instance pointer for HAL callbacks
    g_bno085_instance = this;
    
    // Initialize reset pin if specified
    if (reset_pin >= 0) {
        // Note: GPIO setup would go here for reset pin control
        // This is platform-specific and would need GPIO library
    }
    
    // Auto-initialize and start if possible
    if (initialize()) {
        start();
    }
}

BNO085::~BNO085() {
    stop();
    g_bno085_instance = nullptr;
}

bool BNO085::initialize() {
    if (initialized.load()) {
        return true;
    }
    
    // Open I2C connection
    if (!i2c_device.open()) {
        setError("Failed to open I2C device: " + i2c_device.getLastError());
        return false;
    }
    
    // Perform hardware reset if possible
    hardwareReset();
    
    // Initialize SH2 interface
    int status = sh2_open(&hal, eventCallback, this);
    if (status != SH2_OK) {
        setError("Failed to initialize SH2 interface: " + std::to_string(status));
        return false;
    }
    
    // Get product IDs to verify connection
    status = sh2_getProdIds(&product_ids);
    if (status != SH2_OK) {
        setError("Failed to get product IDs: " + std::to_string(status));
        return false;
    }
    
    // Register sensor callback
    status = sh2_setSensorCallback(sensorCallback, this);
    if (status != SH2_OK) {
        setError("Failed to register sensor callback: " + std::to_string(status));
        return false;
    }
    
    initialized.store(true);
    std::cout << "BNO085 initialized successfully" << std::endl;
    
    // Print product information
    if (product_ids.numEntries > 0) {
        std::cout << "BNO085 Product Info:" << std::endl;
        std::cout << "  SW Version: " << static_cast<int>(product_ids.entry[0].swVersionMajor) 
                  << "." << static_cast<int>(product_ids.entry[0].swVersionMinor) 
                  << "." << product_ids.entry[0].swVersionPatch << std::endl;
        std::cout << "  SW Part Number: " << product_ids.entry[0].swPartNumber << std::endl;
        std::cout << "  SW Build Number: " << product_ids.entry[0].swBuildNumber << std::endl;
    }
    
    return true;
}

bool BNO085::start() {
    if (running.load()) {
        std::cout << "BNO085 read thread is already running" << std::endl;
        return true;
    }
    
    if (!initialized.load()) {
        setError("BNO085 not initialized - call initialize() first");
        return false;
    }
    
    try {
        running.store(true);
        read_thread = std::make_unique<std::thread>(&BNO085::readLoop, this);
        std::cout << "BNO085 read thread started" << std::endl;
        return true;
    } catch (const std::exception& e) {
        setError("Failed to start BNO085 read thread: " + std::string(e.what()));
        running.store(false);
        return false;
    }
}

void BNO085::stop() {
    if (!running.load()) {
        return;
    }
    
    running.store(false);
    
    if (read_thread && read_thread->joinable()) {
        read_thread->join();
        read_thread.reset();
    }
    
    // Close SH2 interface
    sh2_close();
    
    std::cout << "BNO085 read thread stopped" << std::endl;
}

bool BNO085::restart() {
    stop();
    return start();
}

bool BNO085::reconnect() {
    stop();
    initialized.store(false);
    i2c_device.close();
    return initialize() && start();
}

void BNO085::readLoop() {
    std::cout << "BNO085 read loop started at " << update_rate_hz.load() << " Hz" << std::endl;
    
    while (running.load()) {
        auto start_time = std::chrono::steady_clock::now();
        
        if (!initialized.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        
        // Service the SH2 interface to process incoming data
        sh2_service();
        
        // Calculate sleep time to maintain update rate
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        auto target_interval = std::chrono::microseconds(1000000 / update_rate_hz.load());
        
        if (elapsed < target_interval) {
            std::this_thread::sleep_for(target_interval - elapsed);
        }
    }
    
    std::cout << "BNO085 read loop finished" << std::endl;
}

// Sensor enable functions
bool BNO085::enableAccelerometer(uint32_t interval_us) {
    sh2_SensorConfig_t config = {};
    config.reportInterval_us = interval_us;
    
    int status = sh2_setSensorConfig(SH2_ACCELEROMETER, &config);
    if (status == SH2_OK) {
        accel_enabled.store(true);
        return true;
    }
    
    setError("Failed to enable accelerometer: " + std::to_string(status));
    return false;
}

bool BNO085::enableGyroscope(uint32_t interval_us) {
    sh2_SensorConfig_t config = {};
    config.reportInterval_us = interval_us;
    
    int status = sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &config);
    if (status == SH2_OK) {
        gyro_enabled.store(true);
        return true;
    }
    
    setError("Failed to enable gyroscope: " + std::to_string(status));
    return false;
}

bool BNO085::enableMagnetometer(uint32_t interval_us) {
    sh2_SensorConfig_t config = {};
    config.reportInterval_us = interval_us;
    
    int status = sh2_setSensorConfig(SH2_MAGNETIC_FIELD_CALIBRATED, &config);
    if (status == SH2_OK) {
        mag_enabled.store(true);
        return true;
    }
    
    setError("Failed to enable magnetometer: " + std::to_string(status));
    return false;
}

bool BNO085::enableRotationVector(uint32_t interval_us) {
    sh2_SensorConfig_t config = {};
    config.reportInterval_us = interval_us;
    
    int status = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);
    if (status == SH2_OK) {
        rotation_vector_enabled.store(true);
        return true;
    }
    
    setError("Failed to enable rotation vector: " + std::to_string(status));
    return false;
}

bool BNO085::enableLinearAcceleration(uint32_t interval_us) {
    sh2_SensorConfig_t config = {};
    config.reportInterval_us = interval_us;
    
    int status = sh2_setSensorConfig(SH2_LINEAR_ACCELERATION, &config);
    return (status == SH2_OK);
}

bool BNO085::enableGravity(uint32_t interval_us) {
    sh2_SensorConfig_t config = {};
    config.reportInterval_us = interval_us;
    
    int status = sh2_setSensorConfig(SH2_GRAVITY, &config);
    return (status == SH2_OK);
}

bool BNO085::enableGameRotationVector(uint32_t interval_us) {
    sh2_SensorConfig_t config = {};
    config.reportInterval_us = interval_us;
    
    int status = sh2_setSensorConfig(SH2_GAME_ROTATION_VECTOR, &config);
    return (status == SH2_OK);
}

bool BNO085::disableAllSensors() {
    sh2_SensorConfig_t config = {};
    config.reportInterval_us = 0; // Disable by setting interval to 0
    
    // Disable all sensor types
    sh2_setSensorConfig(SH2_ACCELEROMETER, &config);
    sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &config);
    sh2_setSensorConfig(SH2_MAGNETIC_FIELD_CALIBRATED, &config);
    sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);
    sh2_setSensorConfig(SH2_LINEAR_ACCELERATION, &config);
    sh2_setSensorConfig(SH2_GRAVITY, &config);
    sh2_setSensorConfig(SH2_GAME_ROTATION_VECTOR, &config);
    
    accel_enabled.store(false);
    gyro_enabled.store(false);
    mag_enabled.store(false);
    rotation_vector_enabled.store(false);
    
    return true;
}

// Data access functions
IMUData BNO085::getIMUData() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return current_data;
}

CalibrationStatus BNO085::getCalibrationStatus() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return calibration_status;
}

bool BNO085::getAcceleration(float& x, float& y, float& z) const {
    std::lock_guard<std::mutex> lock(data_mutex);
    if (!current_data.valid) return false;
    
    x = current_data.accel_x;
    y = current_data.accel_y;
    z = current_data.accel_z;
    return true;
}

bool BNO085::getGyroscope(float& x, float& y, float& z) const {
    std::lock_guard<std::mutex> lock(data_mutex);
    if (!current_data.valid) return false;
    
    x = current_data.gyro_x;
    y = current_data.gyro_y;
    z = current_data.gyro_z;
    return true;
}

bool BNO085::getMagnetometer(float& x, float& y, float& z) const {
    std::lock_guard<std::mutex> lock(data_mutex);
    if (!current_data.valid) return false;
    
    x = current_data.mag_x;
    y = current_data.mag_y;
    z = current_data.mag_z;
    return true;
}

bool BNO085::getQuaternion(float& i, float& j, float& k, float& real) const {
    std::lock_guard<std::mutex> lock(data_mutex);
    if (!current_data.valid) return false;
    
    i = current_data.quat_i;
    j = current_data.quat_j;
    k = current_data.quat_k;
    real = current_data.quat_real;
    return true;
}

bool BNO085::getEulerAngles(float& roll, float& pitch, float& yaw) const {
    std::lock_guard<std::mutex> lock(data_mutex);
    if (!current_data.valid) return false;
    
    roll = current_data.euler_roll;
    pitch = current_data.euler_pitch;
    yaw = current_data.euler_yaw;
    return true;
}

// Calibration functions
bool BNO085::isCalibrated() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return calibration_status.is_calibrated;
}

bool BNO085::saveCalibration() {
    int status = sh2_saveDcdNow();
    return (status == SH2_OK);
}

bool BNO085::resetCalibration() {
    int status = sh2_clearDcdAndReset();
    return (status == SH2_OK);
}

// Configuration
bool BNO085::setUpdateRate(uint32_t rate_hz) {
    if (rate_hz < 1 || rate_hz > 1000) {
        setError("Update rate must be between 1 and 1000 Hz");
        return false;
    }
    
    update_rate_hz.store(rate_hz);
    std::cout << "BNO085 update rate set to " << rate_hz << " Hz" << std::endl;
    return true;
}

void BNO085::hardwareReset() {
    if (reset_pin >= 0) {
        // Platform-specific GPIO reset would go here
        std::cout << "Hardware reset triggered" << std::endl;
        usleep(100000); // 100ms delay
    }
}

// HAL Interface Functions (Static callbacks)
int BNO085::hal_open(sh2_Hal_t *self) {
    if (!g_bno085_instance) return -1;
    
    // Send software reset packet
    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
    if (!g_bno085_instance->i2c_device.writeRaw(softreset_pkt, 5)) {
        return -1;
    }
    
    usleep(300000); // 300ms delay
    return 0;
}

void BNO085::hal_close(sh2_Hal_t *self) {
    // Nothing specific needed for close
}

int BNO085::hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    if (!g_bno085_instance) return 0;
    
    // Read 4-byte header first
    uint8_t header[4];
    if (!g_bno085_instance->i2c_device.readRaw(header, 4)) {
        return 0;
    }
    
    // Get packet size from header
    uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
    packet_size &= ~0x8000; // Remove continue bit
    
    if (packet_size > len) {
        return 0; // Packet too large
    }
    
    // Copy header to buffer
    memcpy(pBuffer, header, 4);
    
    // Read remaining data if needed
    if (packet_size > 4) {
        if (!g_bno085_instance->i2c_device.readRaw(pBuffer + 4, packet_size - 4)) {
            return 0;
        }
    }
    
    // Set timestamp
    *t_us = hal_getTimeUs(self);
    
    return packet_size;
}

int BNO085::hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    if (!g_bno085_instance) return 0;
    
    return g_bno085_instance->i2c_device.writeRaw(pBuffer, len) ? len : 0;
}

uint32_t BNO085::hal_getTimeUs(sh2_Hal_t *self) {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration);
    return static_cast<uint32_t>(microseconds.count());
}

// SH2 Callback Functions (Static callbacks)
void BNO085::eventCallback(void* cookie, sh2_AsyncEvent_t* event) {
    BNO085* instance = static_cast<BNO085*>(cookie);
    if (!instance) return;
    
    if (event->eventId == SH2_RESET) {
        std::cout << "BNO085 reset detected" << std::endl;
    }
}

void BNO085::sensorCallback(void* cookie, sh2_SensorEvent_t* event) {
    BNO085* instance = static_cast<BNO085*>(cookie);
    if (!instance) return;
    
    sh2_SensorValue_t sensor_value;
    int rc = sh2_decodeSensorEvent(&sensor_value, event);
    if (rc == SH2_OK) {
        instance->processSensorEvent(sensor_value);
    }
}

// Process sensor events and update data
void BNO085::processSensorEvent(const sh2_SensorValue_t& sensor_value) {
    std::lock_guard<std::mutex> lock(data_mutex);
    
    current_data.timestamp = std::chrono::system_clock::now();
    current_data.accuracy = sensor_value.status & 0x03;
    current_data.valid = true;
    
    switch (sensor_value.sensorId) {
        case SH2_ACCELEROMETER:
            current_data.accel_x = sensor_value.un.accelerometer.x;
            current_data.accel_y = sensor_value.un.accelerometer.y;
            current_data.accel_z = sensor_value.un.accelerometer.z;
            break;
            
        case SH2_GYROSCOPE_CALIBRATED:
            current_data.gyro_x = sensor_value.un.gyroscope.x;
            current_data.gyro_y = sensor_value.un.gyroscope.y;
            current_data.gyro_z = sensor_value.un.gyroscope.z;
            break;
            
        case SH2_MAGNETIC_FIELD_CALIBRATED:
            current_data.mag_x = sensor_value.un.magneticField.x;
            current_data.mag_y = sensor_value.un.magneticField.y;
            current_data.mag_z = sensor_value.un.magneticField.z;
            break;
            
        case SH2_ROTATION_VECTOR:
            current_data.quat_i = sensor_value.un.rotationVector.i;
            current_data.quat_j = sensor_value.un.rotationVector.j;
            current_data.quat_k = sensor_value.un.rotationVector.k;
            current_data.quat_real = sensor_value.un.rotationVector.real;
            updateEulerAngles();
            break;
            
        case SH2_LINEAR_ACCELERATION:
            current_data.linear_accel_x = sensor_value.un.linearAcceleration.x;
            current_data.linear_accel_y = sensor_value.un.linearAcceleration.y;
            current_data.linear_accel_z = sensor_value.un.linearAcceleration.z;
            break;
            
        case SH2_GRAVITY:
            current_data.gravity_x = sensor_value.un.gravity.x;
            current_data.gravity_y = sensor_value.un.gravity.y;
            current_data.gravity_z = sensor_value.un.gravity.z;
            break;
    }
    
    // Update calibration status based on accuracy
    calibration_status.system = current_data.accuracy;
    calibration_status.is_calibrated = (current_data.accuracy >= 2); // Medium or high accuracy
}

void BNO085::updateEulerAngles() {
    // Convert quaternion to Euler angles (in degrees)
    float qw = current_data.quat_real;
    float qx = current_data.quat_i;
    float qy = current_data.quat_j;
    float qz = current_data.quat_k;
    
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (qw * qx + qy * qz);
    float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    current_data.euler_roll = std::atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
    
    // Pitch (y-axis rotation)
    float sinp = 2 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1) {
        current_data.euler_pitch = std::copysign(M_PI / 2, sinp) * RAD_TO_DEG;
    } else {
        current_data.euler_pitch = std::asin(sinp) * RAD_TO_DEG;
    }
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (qw * qz + qx * qy);
    float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    current_data.euler_yaw = std::atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

const std::string& BNO085::getLastError() const {
    std::lock_guard<std::mutex> lock(error_mutex);
    return last_error;
}

void BNO085::setError(const std::string& error)
{

}