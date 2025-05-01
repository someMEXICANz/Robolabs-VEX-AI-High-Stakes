#ifndef UPS_H
#define UPS_H

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <chrono>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <cmath>


// Register definitions
#define REG_CONFIG        0x00
#define REG_SHUNTVOLTAGE  0x01
#define REG_BUSVOLTAGE    0x02
#define REG_POWER         0x03
#define REG_CURRENT       0x04
#define REG_CALIBRATION   0x05

// Configuration constants
#define RANGE_16V         0x00  // Bus voltage range to 16V
#define RANGE_32V         0x01  // Bus voltage range to 32V
#define GAIN_1_40MV       0x00  // Gain: /1, 40 mV range
#define GAIN_2_80MV       0x01  // Gain: /2, 80 mV range
#define GAIN_4_160MV      0x02  // Gain: /4, 160 mV range
#define GAIN_8_320MV      0x03  // Gain: /8, 320 mV range
#define ADCRES_12BIT_32S  0x0D  // ADC: 12-bit, 32 samples
#define MODE_SANDBVOLT_CONTINUOUS 0x07  // Mode: Shunt and bus voltage continuous


struct UPSData 
{
        float shuntVoltage; 
        float busVoltage;
        float current;
        float power;
        float batteryLevel;
        bool valid;      
        std::chrono::system_clock::time_point timestamp;
          
};

class UPS {
public:
    explicit UPS(const char* i2c_device = "/dev/i2c-1");    // Constructor
    ~UPS();                                                     // Destructor
    
    // Delete copy constructor and assignment operator
    UPS(const UPS&) = delete;
    UPS& operator=(const UPS&) = delete;

    // Core operations
    bool initialize();
    bool start();
    void stop();
    bool restart();
    bool reconnect();

    // Status Checks
    bool isRunning() const { return running; }
    bool isInitialized() const { return initialized; }
    

    void set_calibration_32V_2A();
    uint32_t getBatteryPercentage();
    UPSData getUPSData() const;
    

private:
    
    void readLoop();
    bool readData();

    // I2C communication
    bool writeByte(uint8_t reg, uint8_t data);
    bool writeWord(uint8_t reg, uint16_t data);
    int16_t readWord(uint8_t reg);

    // I2C device properties
    const char* i2c_device; 
    int ina219_fd;

    // Calibration values
    uint16_t cal_value;
    float current_lsb;
    float power_lsb;

    // Configuration values
    uint8_t bus_voltage_range;
    uint8_t gain;
    uint8_t bus_adc_resolution;
    uint8_t shunt_adc_resolution;
    uint8_t mode;
    uint16_t config;

    // Thread and running state
    mutable std::mutex data_mutex;
    std::unique_ptr<std::thread> read_thread;
    bool running;
    bool initialized;
    UPSData current_data;

    // Sensor address
    static constexpr uint8_t UPS_ADDR = 0x42;
    




};

#endif // UPS_H