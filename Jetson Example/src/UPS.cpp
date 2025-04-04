#include "UPS.h"
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

UPS::UPS(uint8_t addr, const std::string& i2c_device) : addr(addr), i2c_fd(-1) {
    // Open I2C device
    i2c_fd = open(i2c_device.c_str(), O_RDWR);
    if (i2c_fd < 0) {
        setError("Failed to open I2C device: " + i2c_device);
    }
    initialize();


}

UPS::~UPS() {
    if (i2c_fd >= 0) {
        close(i2c_fd);
        i2c_fd = -1;
    }
}

bool UPS::initialize() {
    if (i2c_fd < 0) {
        setError("I2C device not opened");
        return false;
    }

    // Set I2C slave address
    if (ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
        setError("Failed to set I2C slave address");
        return false;
    }

    // Initialize calibration values
    _cal_value = 0;
    _current_lsb = 0;
    _power_lsb = 0;
    
    // Set default configuration
    set_calibration_32V_2A();
    
    return true;
}

void UPS::set_calibration_32V_2A() {
    // Current LSB = 100uA per bit
    _current_lsb = 0.1;
    
    // Calibration value calculation
    _cal_value = 4096;
    
    // Power LSB = 20 * Current LSB = 2mW per bit
    _power_lsb = 0.002;
    
    // Set Calibration register
    writeWord(REG_CALIBRATION, _cal_value);
    
    // Set Config register
    bus_voltage_range = RANGE_32V;
    gain = GAIN_8_320MV;
    bus_adc_resolution = ADCRES_12BIT_32S;
    shunt_adc_resolution = ADCRES_12BIT_32S;
    mode = MODE_SANDBVOLT_CONTINUOUS;
    
    config = bus_voltage_range << 13 |
             gain << 11 |
             bus_adc_resolution << 7 |
             shunt_adc_resolution << 3 |
             mode;
             
    writeWord(REG_CONFIG, config);
}

float UPS::getShuntVoltage_mV() {
    writeWord(REG_CALIBRATION, _cal_value);
    int16_t value = readWord(REG_SHUNTVOLTAGE);
    return value * 0.01f;  // LSB = 10uV
}

float UPS::getBusVoltage_V() {
    writeWord(REG_CALIBRATION, _cal_value);
    uint16_t value = readWord(REG_BUSVOLTAGE);
    return ((value >> 3) * 0.004f);  // LSB = 4mV, shift right 3 to drop status bits
}

float UPS::getCurrent_mA() {
    int16_t value = readWord(REG_CURRENT);
    return value * _current_lsb;
}

float UPS::getPower_W() {
    writeWord(REG_CALIBRATION, _cal_value);
    int16_t value = readWord(REG_POWER);
    return value * _power_lsb;
}

float UPS::getBatteryPercentage() {
    float bus_voltage = getBusVoltage_V();
    
    // Calculate percentage using the formula in your Python code
    float p = (bus_voltage - 6.0f) / 2.4f * 100.0f;
    
    // Clamp between 0% and 100%
    if (p > 100.0f) p = 100.0f;
    if (p < 0.0f) p = 0.0f;
    
    return p;
}

bool UPS::writeByte(uint8_t reg, uint8_t data) {
    if (ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
        setError("Failed to set I2C slave address");
        return false;
    }
    
    uint8_t buffer[2] = {reg, data};
    if (write(i2c_fd, buffer, 2) != 2) {
        setError("Failed to write to register");
        return false;
    }
    
    return true;
}

bool UPS::writeWord(uint8_t reg, uint16_t data) {
    if (ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
        setError("Failed to set I2C slave address");
        return false;
    }
    
    uint8_t buffer[3];
    buffer[0] = reg;
    buffer[1] = (data >> 8) & 0xFF;  // MSB
    buffer[2] = data & 0xFF;         // LSB
    
    if (write(i2c_fd, buffer, 3) != 3) {
        setError("Failed to write to register");
        return false;
    }
    
    return true;
}

int16_t UPS::readWord(uint8_t reg) {
    if (ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
        setError("Failed to set I2C slave address");
        return -1;
    }
    
    // Write the register address
    if (write(i2c_fd, &reg, 1) != 1) {
        setError("Failed to write register address");
        return -1;
    }
    
    // Read the data (2 bytes)
    uint8_t buf[2] = {0};
    if (read(i2c_fd, buf, 2) != 2) {
        setError("Failed to read from register");
        return -1;
    }
    
    return (buf[0] << 8) | buf[1];
}

void UPS::setError(const std::string& error) {
    _lastError = error;
    std::cerr << "UPS Error: " << error << std::endl;
}

std::string UPS::getLastError() const {
    return _lastError;
}