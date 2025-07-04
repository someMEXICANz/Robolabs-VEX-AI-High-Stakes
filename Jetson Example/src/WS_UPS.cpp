#include "WS_UPS.h"
#include <iostream>
#include <algorithm>
#include <cmath>

UPS::UPS(const std::string& i2c_device_path) 
    : i2c_device(i2c_device_path, UPS_ADDR),
      current_lsb(0.0f),
      power_lsb(0.0f),
      cal_value(0),
      running(false),
      initialized(false),
      update_rate_hz(DEFAULT_UPDATE_RATE_HZ) {
    
    // Auto-initialize and start if possible
    if (initialize()) {
        start();
    }
}

UPS::~UPS() {
    stop();
}

bool UPS::initialize() {
    if (initialized.load()) {
        return true;
    }
    
    // Open I2C connection
    if (!i2c_device.open()) {
        setError("Failed to open I2C device: " + i2c_device.getLastError());
        return false;
    }
    
    // Configure the INA219
    if (!configureDevice()) {
        setError("Failed to configure INA219");
        return false;
    }
    
    // Test read to verify communication
    UPSData test_data;
    if (!readSensorData()) {
        setError("Failed to read test data from INA219");
        return false;
    }
    
    initialized.store(true);
    std::cout << "UPS initialized successfully" << std::endl;
    return true;
}

bool UPS::start() {
    if (running.load()) {
        std::cout << "UPS read thread is already running" << std::endl;
        return true;
    }
    
    if (!initialized.load()) {
        setError("UPS not initialized - call initialize() first");
        return false;
    }
    
    try {
        running.store(true);
        read_thread = std::make_unique<std::thread>(&UPS::readLoop, this);
        std::cout << "UPS read thread started" << std::endl;
        return true;
    } catch (const std::exception& e) {
        setError("Failed to start UPS read thread: " + std::string(e.what()));
        running.store(false);
        return false;
    }
}

void UPS::stop() {
    if (!running.load()) {
        return;
    }
    
    running.store(false);
    
    if (read_thread && read_thread->joinable()) {
        read_thread->join();
        read_thread.reset();
    }
    
    std::cout << "UPS read thread stopped" << std::endl;
}

bool UPS::restart() {
    stop();
    return start();
}

bool UPS::reconnect() {
    stop();
    initialized.store(false);
    i2c_device.close();
    return initialize() && start();
}

void UPS::readLoop() {
    std::cout << "UPS read loop started at " << update_rate_hz.load() << " Hz" << std::endl;
    
    while (running.load()) {
        auto start_time = std::chrono::steady_clock::now();
        
        if (!initialized.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        
        // Read sensor data
        if (!readSensorData()) {
            // On read failure, mark data as invalid but continue
            std::lock_guard<std::mutex> lock(data_mutex);
            current_data.valid = false;
            current_data.timestamp = std::chrono::system_clock::now();
        }
        
        // Calculate sleep time to maintain update rate
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        auto target_interval = std::chrono::microseconds(1000000 / update_rate_hz.load());
        
        if (elapsed < target_interval) {
            std::this_thread::sleep_for(target_interval - elapsed);
        }
    }
    
    std::cout << "UPS read loop finished" << std::endl;
}

bool UPS::readSensorData() {
    // Read all registers
    uint16_t shunt_raw, bus_raw, current_raw, power_raw;
    
    if (!i2c_device.readWord(REG_SHUNTVOLTAGE, shunt_raw)) {
        setError("Failed to read shunt voltage: " + i2c_device.getLastError());
        return false;
    }
    
    if (!i2c_device.readWord(REG_BUSVOLTAGE, bus_raw)) {
        setError("Failed to read bus voltage: " + i2c_device.getLastError());
        return false;
    }
    
    if (!i2c_device.readWord(REG_CURRENT, current_raw)) {
        setError("Failed to read current: " + i2c_device.getLastError());
        return false;
    }
    
    if (!i2c_device.readWord(REG_POWER, power_raw)) {
        setError("Failed to read power: " + i2c_device.getLastError());
        return false;
    }
    
    // Convert raw values to engineering units
    float shunt_voltage = static_cast<int16_t>(shunt_raw) * 0.01f;  // mV
    float bus_voltage = (bus_raw >> 3) * 0.004f;                   // V
    float current = static_cast<int16_t>(current_raw) * current_lsb; // mA
    float power = static_cast<int16_t>(power_raw) * power_lsb;       // mW
    float battery_level = calculateBatteryLevel(bus_voltage);       // %
    
    // Update data structure with thread safety
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        current_data.shuntVoltage = shunt_voltage;
        current_data.busVoltage = bus_voltage;
        current_data.current = current;
        current_data.power = power;
        current_data.batteryLevel = battery_level;
        current_data.valid = true;
        current_data.timestamp = std::chrono::system_clock::now();
    }
    
    return true;
}

bool UPS::configureDevice() {
    setCalibration32V2A();
    
    // Configure measurement settings
    uint16_t config = (RANGE_32V << 13) |
                     (GAIN_8_320MV << 11) |
                     (ADCRES_12BIT_32S << 7) |
                     (ADCRES_12BIT_32S << 3) |
                     MODE_SANDBVOLT_CONTINUOUS;
    
    if (!i2c_device.writeWord(REG_CONFIG, config)) {
        setError("Failed to write configuration: " + i2c_device.getLastError());
        return false;
    }
    
    return true;
}

void UPS::setCalibration32V2A() {
    // Set calibration for 32V, 2A range
    current_lsb = 0.1f;     // 100µA per bit
    cal_value = 4096;       // Calibration register value
    power_lsb = 2.0f;       // 2mW per bit (20 * current_lsb)
    
    if (!i2c_device.writeWord(REG_CALIBRATION, cal_value)) {
        setError("Failed to write calibration: " + i2c_device.getLastError());
    }
}

float UPS::calculateBatteryLevel(float bus_voltage) {
    // Convert voltage to battery percentage
    // Assumes battery voltage range from 6.0V (0%) to 8.4V (100%)
    float percentage = ((bus_voltage - BATTERY_MIN_VOLTAGE) / BATTERY_VOLTAGE_RANGE) * 100.0f;
    
    // Clamp to valid range
    return std::max(0.0f, std::min(100.0f, percentage));
}

bool UPS::setUpdateRate(uint32_t rate_hz) {
    if (rate_hz < 1 || rate_hz > 1000) {
        setError("Update rate must be between 1 and 1000 Hz");
        return false;
    }
    
    update_rate_hz.store(rate_hz);
    std::cout << "UPS update rate set to " << rate_hz << " Hz" << std::endl;
    return true;
}

UPSData UPS::getUPSData() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return current_data;
}

float UPS::getBatteryPercentage() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return current_data.batteryLevel;
}

float UPS::getBusVoltage() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return current_data.busVoltage;
}

float UPS::getCurrent() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return current_data.current;
}

float UPS::getPower() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return current_data.power;
}

const std::string& UPS::getLastError() const {
    std::lock_guard<std::mutex> lock(error_mutex);
    return last_error;
}

void UPS::setError(const std::string& error) {
    {
        std::lock_guard<std::mutex> lock(error_mutex);
        last_error = error;
    }
    std::cerr << "UPS Error: " << error << std::endl;
}