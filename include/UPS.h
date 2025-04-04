#ifndef UPS_H
#define UPS_H

#include <string>
#include <cstdint>

class UPS {
private:
    int i2c_fd;
    uint8_t addr;
    std::string _lastError;
    
    // Calibration values
    uint16_t _cal_value;
    float _current_lsb;
    float _power_lsb;
    
    // Configuration values
    uint8_t bus_voltage_range;
    uint8_t gain;
    uint8_t bus_adc_resolution;
    uint8_t shunt_adc_resolution;
    uint8_t mode;
    uint16_t config;
    
    // I2C helper methods
    bool writeByte(uint8_t reg, uint8_t data);
    bool writeWord(uint8_t reg, uint16_t data);
    int16_t readWord(uint8_t reg);
    
    // Set error message
    void setError(const std::string& error);

public:
    // Constructor
    UPS(uint8_t addr = 0x42, const std::string& i2c_device = "/dev/i2c-1");
    
    // Destructor
    ~UPS();
    
    // Initialize and calibrate
    bool initialize();
    void set_calibration_32V_2A();
    
    // Read sensor data
    float getShuntVoltage_mV();
    float getBusVoltage_V();
    float getCurrent_mA();
    float getPower_W();
    float getBatteryPercentage();
    
    // Get the last error message
    std::string getLastError() const;
};

#endif // UPS_H