#include "BNO085.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <unistd.h>
#include <sys/time.h>

// Global pointer for HAL callbacks (necessary for C library interface)
static BNO085* g_bno085_instance = nullptr;

BNO085::BNO085(const std::string& i2c_device_path, int reset_pin) 
    : i2c_device(i2c_device_path, BNO085_I2C_ADDR),
      reset_pin(reset_pin),
      running(false),
      initialized(false),
      reset_occurred(false) {
    
    // Set global instance for HAL callbacks
    g_bno085_instance = this;
    
    // Initialize HAL structure
    memset(&hal, 0, sizeof(hal));
    hal.open = halOpen;
    hal.close = halClose;
    hal.read = halRead;
    hal.write = halWrite;
    hal.getTimeUs = halGetTimeUs;
    
    // Initialize product IDs
    memset(&product_ids, 0, sizeof(product_ids));
}

BNO085::~BNO085() {
    stop();
    if (g_bno085_instance == this) {
        g_bno085_instance = nullptr;
    }
}

bool BNO085::initialize() {
    if (initialized.load()) {
        return true;
    }
    
    // Perform hardware reset if pin is specified
    if (reset_pin >= 0) {
        hardwareReset();
    }
    
    // Open I2C connection
    if (!i2c_device.open()) {
        setError("Failed to open I2C device: " + i2c_device.getLastError());
        return false;
    }
    
    // Initialize SH2 library
    if (!initializeSH2()) {
        setError("Failed to initialize SH2 library");
        return false;
    }
    
    initialized.store(true);
    reset_occurred.store(false);
    std::cout << "BNO085 initialized successfully" << std::endl;
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
    
    // Close SH2 connection
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
    std::cout << "BNO085 read loop started" << std::endl;
    
    while (running.load()) {
        if (!initialized.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        
        // Service the SH2 library
        sh2_service();
        
        // Check for reset
        if (reset_occurred.load()) {
            std::cout << "BNO085 reset detected, reconfiguring sensors..." << std::endl;
            reset_occurred.store(false);
            // Note: You may want to re-enable sensors here if needed
        }
        
        // Small delay to prevent CPU hogging
        std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_LOOP_DELAY_MS));
    }
    
    std::cout << "BNO085 read loop finished" << std::endl;
}

bool BNO085::initializeSH2() {
    // Open SH2 session
    int result = sh2_open(&hal, asyncEventCallback, this);
    if (result != SH2_OK) {
        setError("Failed to open SH2 session: " + std::to_string(result));
        return false;
    }
    
    // Wait for initialization to complete
    auto start_time = std::chrono::steady_clock::now();
    while (!sh2_isOpened() && 
           std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(INIT_TIMEOUT_MS)) {
        sh2_service();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    if (!sh2_isOpened()) {
        setError("SH2 session failed to open within timeout");
        return false;
    }
    
    // Get product IDs
    result = sh2_getProdIds(&product_ids);
    if (result != SH2_OK) {
        setError("Failed to get product IDs: " + std::to_string(result));
        return false;
    }
    
    std::cout << "BNO085 Product ID: " << std::hex << product_ids.entry[0].swPartNumber << std::dec << std::endl;
    
    return true;
}

void BNO085::hardwareReset() {
    if (reset_pin < 0) {
        return;
    }
    
    // This is a placeholder for GPIO control
    // You'll need to implement actual GPIO control based on your platform
    std::cout << "Performing hardware reset on pin " << reset_pin << std::endl;
    
    // Typical reset sequence:
    // 1. Set pin low
    // 2. Wait
    // 3. Set pin high
    // 4. Wait for device to boot
    
    std::this_thread::sleep_for(std::chrono::milliseconds(RESET_DELAY_MS));
}

bool BNO085::performSoftReset() {
    // Send soft reset command via I2C
    uint8_t reset_cmd[] = {5, 0, 1, 0, 1};
    
    for (int attempts = 0; attempts < 5; attempts++) {
        if (i2c_device.writeBytes(0, reset_cmd, sizeof(reset_cmd))) {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    
    return false;
}

bool BNO085::enableRotationVector(uint32_t interval_us) {
    sh2_SensorConfig_t config;
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.reportInterval_us = interval_us;
    config.batchInterval_us = 0;
    
    int result = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);
    if (result != SH2_OK) {
        setError("Failed to enable rotation vector: " + std::to_string(result));
        return false;
    }
    
    return true;
}

bool BNO085::enableLinearAccelerometer(uint32_t interval_us) {
    sh2_SensorConfig_t config;
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.reportInterval_us = interval_us;
    config.batchInterval_us = 0;
    
    int result = sh2_setSensorConfig(SH2_LINEAR_ACCELERATION, &config);
    if (result != SH2_OK) {
        setError("Failed to enable linear accelerometer: " + std::to_string(result));
        return false;
    }
    
    return true;
}

bool BNO085::enableGyroscope(uint32_t interval_us) {
    sh2_SensorConfig_t config;
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.reportInterval_us = interval_us;
    config.batchInterval_us = 0;
    
    int result = sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &config);
    if (result != SH2_OK) {
        setError("Failed to enable gyroscope: " + std::to_string(result));
        return false;
    }
    
    return true;
}

bool BNO085::enableMagnetometer(uint32_t interval_us) {
    sh2_SensorConfig_t config;
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.reportInterval_us = interval_us;
    config.batchInterval_us = 0;
    
    int result = sh2_setSensorConfig(SH2_MAGNETIC_FIELD_CALIBRATED, &config);
    if (result != SH2_OK) {
        setError("Failed to enable magnetometer: " + std::to_string(result));
        return false;
    }
    
    return true;
}

bool BNO085::enableGameRotationVector(uint32_t interval_us) {
    sh2_SensorConfig_t config;
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.reportInterval_us = interval_us;
    config.batchInterval_us = 0;
    
    int result = sh2_setSensorConfig(SH2_GAME_ROTATION_VECTOR, &config);
    if (result != SH2_OK) {
        setError("Failed to enable game rotation vector: " + std::to_string(result));
        return false;
    }
    
    return true;
}

void BNO085::processSensorEvent(sh2_SensorValue_t* sensor_value) {
    std::lock_guard<std::mutex> lock(data_mutex);
    
    current_data.timestamp = sensor_value->timestamp;
    current_data.system_timestamp = std::chrono::system_clock::now();
    current_data.valid = true;
    
    switch (sensor_value->sensorId) {
        case SH2_ROTATION_VECTOR:
            current_data.quat_i = sensor_value->un.rotationVector.i;
            current_data.quat_j = sensor_value->un.rotationVector.j;
            current_data.quat_k = sensor_value->un.rotationVector.k;
            current_data.quat_real = sensor_value->un.rotationVector.real;
            current_data.quat_accuracy = sensor_value->un.rotationVector.accuracy;
            break;
            
        case SH2_LINEAR_ACCELERATION:
            current_data.accel_x = sensor_value->un.linearAcceleration.x;
            current_data.accel_y = sensor_value->un.linearAcceleration.y;
            current_data.accel_z = sensor_value->un.linearAcceleration.z;
            current_data.accel_accuracy = 0; // Linear acceleration doesn't have accuracy
            break;
            
        case SH2_GYROSCOPE_CALIBRATED:
            current_data.gyro_x = sensor_value->un.gyroscope.x;
            current_data.gyro_y = sensor_value->un.gyroscope.y;
            current_data.gyro_z = sensor_value->un.gyroscope.z;
            current_data.gyro_accuracy = 0; // Accuracy not always available
            break;
            
        case SH2_MAGNETIC_FIELD_CALIBRATED:
            current_data.mag_x = sensor_value->un.magneticField.x;
            current_data.mag_y = sensor_value->un.magneticField.y;
            current_data.mag_z = sensor_value->un.magneticField.z;
            current_data.mag_accuracy = sensor_value->un.magneticField.accuracy;
            break;
            
        case SH2_GAME_ROTATION_VECTOR:
            current_data.game_quat_i = sensor_value->un.gameRotationVector.i;
            current_data.game_quat_j = sensor_value->un.gameRotationVector.j;
            current_data.game_quat_k = sensor_value->un.gameRotationVector.k;
            current_data.game_quat_real = sensor_value->un.gameRotationVector.real;
            current_data.game_quat_accuracy = sensor_value->un.gameRotationVector.accuracy;
            break;
            
        default:
            // Ignore other sensor types
            break;
    }
}

void BNO085::handleAsyncEvent(sh2_AsyncEvent_t* event) {
    if (event->eventId == SH2_RESET) {
        std::cout << "BNO085 reset event received" << std::endl;
        reset_occurred.store(true);
    }
}

// HAL Implementation Functions
int BNO085::halOpen(sh2_Hal_t* self) {
    if (!g_bno085_instance) return -1;
    
    // Perform soft reset
    if (!g_bno085_instance->performSoftReset()) {
        return -1;
    }
    
    return 0;
}

void BNO085::halClose(sh2_Hal_t* self) {
    // Nothing specific needed for close
}

int BNO085::halRead(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len, uint32_t* t_us) {
    if (!g_bno085_instance) return 0;
    
    // Read SHTP header first
    uint8_t header[4];
    if (!g_bno085_instance->i2c_device.readBytes(0, header, 4)) {
        return 0;
    }
    
    // Get packet size from header
    uint16_t packet_size = (uint16_t)header[0] | ((uint16_t)header[1] << 8);
    packet_size &= ~0x8000; // Clear continue bit
    
    if (packet_size > len || packet_size == 0) {
        return 0;
    }
    
    // Read the rest of the packet
    if (!g_bno085_instance->i2c_device.readBytes(0, pBuffer, packet_size)) {
        return 0;
    }
    
    // Set timestamp
    if (t_us) {
        *t_us = g_bno085_instance->halGetTimeUs(self);
    }
    
    return packet_size;
}

int BNO085::halWrite(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len) {
    if (!g_bno085_instance) return 0;
    
    if (g_bno085_instance->i2c_device.writeBytes(0, pBuffer, len)) {
        return len;
    }
    
    return 0;
}

uint32_t BNO085::halGetTimeUs(sh2_Hal_t* self) {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return tv.tv_sec * 1000000 + tv.tv_usec;
}

// Static callback functions
void BNO085::sensorEventCallback(void* cookie, sh2_SensorEvent_t* pEvent) {
    if (cookie && pEvent) {
        BNO085* instance = static_cast<BNO085*>(cookie);
        
        sh2_SensorValue_t sensor_value;
        if (sh2_decodeSensorEvent(&sensor_value, pEvent) == SH2_OK) {
            instance->processSensorEvent(&sensor_value);
        }
    }
}

void BNO085::asyncEventCallback(void* cookie, sh2_AsyncEvent_t* pEvent) {
    if (cookie && pEvent) {
        BNO085* instance = static_cast<BNO085*>(cookie);
        instance->handleAsyncEvent(pEvent);
    }
}

// Data access methods
BNO085Data BNO085::getSensorData() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return current_data;
}

bool BNO085::getQuaternion(float& i, float& j, float& k, float& real, float& accuracy) const {
    std::lock_guard<std::mutex> lock(data_mutex);
    if (!current_data.valid) return false;
    
    i = current_data.quat_i;
    j = current_data.quat_j;
    k = current_data.quat_k;
    real = current_data.quat_real;
    accuracy = current_data.quat_accuracy;
    return true;
}

bool BNO085::getLinearAcceleration(float& x, float& y, float& z, float& accuracy) const {
    std::lock_guard<std::mutex> lock(data_mutex);
    if (!current_data.valid) return false;
    
    x = current_data.accel_x;
    y = current_data.accel_y;
    z = current_data.accel_z;
    accuracy = current_data.accel_accuracy;
    return true;
}

bool BNO085::getGyroscope(float& x, float& y, float& z, float& accuracy) const {
    std::lock_guard<std::mutex> lock(data_mutex);
    if (!current_data.valid) return false;
    
    x = current_data.gyro_x;
    y = current_data.gyro_y;
    z = current_data.gyro_z;
    accuracy = current_data.gyro_accuracy;
    return true;
}

bool BNO085::getMagnetometer(float& x, float& y, float& z, float& accuracy) const {
    std::lock_guard<std::mutex> lock(data_mutex);
    if (!current_data.valid) return false;
    
    x = current_data.mag_x;
    y = current_data.mag_y;
    z = current_data.mag_z;
    accuracy = current_data.mag_accuracy;
    return true;
}

bool BNO085::getGameRotationVector(float& i, float& j, float& k, float& real, float& accuracy) const {
    std::lock_guard<std::mutex> lock(data_mutex);
    if (!current_data.valid) return false;
    
    i = current_data.game_quat_i;
    j = current_data.game_quat_j;
    k = current_data.game_quat_k;
    real = current_data.game_quat_real;
    accuracy = current_data.game_quat_accuracy;
    return true;
}

const std::string& BNO085::getLastError() const {
    std::lock_guard<std::mutex> lock(error_mutex);
    return last_error;
}

void BNO085::setError(const std::string& error) {
    {
        std::lock_guard<std::mutex> lock(error_mutex);
        last_error = error;
    }
    std::cerr << "BNO085 Error: " << error << std::endl;
}