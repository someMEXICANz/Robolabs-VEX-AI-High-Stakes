#ifndef UPS_H
#define UPS_H

#include "I2C_Device.h"
#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>
#include <memory>

// INA219 Register definitions
#define REG_CONFIG        0x00
#define REG_SHUNTVOLTAGE  0x01
#define REG_BUSVOLTAGE    0x02
#define REG_POWER         0x03
#define REG_CURRENT       0x04
#define REG_CALIBRATION   0x05

// Configuration constants
#define RANGE_16V         0x00
#define RANGE_32V         0x01
#define GAIN_1_40MV       0x00
#define GAIN_2_80MV       0x01
#define GAIN_4_160MV      0x02
#define GAIN_8_320MV      0x03
#define ADCRES_12BIT_32S  0x0D
#define MODE_SANDBVOLT_CONTINUOUS 0x07

struct UPSData {
    float shuntVoltage;     // mV
    float busVoltage;       // V
    float current;          // mA
    float power;            // mW
    float batteryLevel;     // %
    bool valid;
    std::chrono::system_clock::time_point timestamp;
    
    // Constructor for easy initialization
    UPSData() : shuntVoltage(0), busVoltage(0), current(0), power(0), 
                batteryLevel(0), valid(false), 
                timestamp(std::chrono::system_clock::now()) {}
};

class UPS {
public:
    // Constructor with default I2C path
    explicit UPS(const std::string& i2c_device = "/dev/i2c-1");
    
    // Destructor
    ~UPS();
    
    // Delete copy constructor and assignment operator
    UPS(const UPS&) = delete;
    UPS& operator=(const UPS&) = delete;

    // Lifecycle management
    bool initialize();
    bool start();
    void stop();
    bool restart();
    bool reconnect();

    // Status checks
    bool isRunning() const { return running.load(); }
    bool isInitialized() const { return initialized.load(); }
    bool isConnected() const { return i2c_device.isOpen(); }

    // Data access
    UPSData getUPSData() const;
    float getBatteryPercentage() const;
    float getBusVoltage() const;
    float getCurrent() const;
    float getPower() const;
    
    // Configuration
    bool setUpdateRate(uint32_t rate_hz);
    uint32_t getUpdateRate() const { return update_rate_hz; }
    
    // Utility
    const std::string& getLastError() const;

private:
    // Core functionality
    void readLoop();
    bool readSensorData();
    bool configureDevice();
    void setCalibration32V2A();
    
    // Utility functions
    static float calculateBatteryLevel(float bus_voltage);
    void setError(const std::string& error);
    
    // Hardware interface
    I2CDevice i2c_device;
    
    // Calibration values
    float current_lsb;      // mA per LSB
    float power_lsb;        // mW per LSB
    uint16_t cal_value;
    
    // Threading
    std::unique_ptr<std::thread> read_thread;
    std::atomic<bool> running;
    std::atomic<bool> initialized;
    
    // Data protection
    mutable std::mutex data_mutex;
    UPSData current_data;
    
    // Configuration
    std::atomic<uint32_t> update_rate_hz;
    
    // Error handling
    mutable std::mutex error_mutex;
    std::string last_error;
    
    // Constants
    static constexpr uint8_t UPS_ADDR = 0x42;
    static constexpr uint32_t DEFAULT_UPDATE_RATE_HZ = 100;
    static constexpr float BATTERY_MIN_VOLTAGE = 6.0f;
    static constexpr float BATTERY_VOLTAGE_RANGE = 2.4f;
};

#endif // UPS_H