#ifndef I2C_DEVICE_H
#define I2C_DEVICE_H

#include <string>
#include <cstdint>
#include <mutex>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

class I2CDevice {
public:
    I2CDevice(const std::string& device_path, uint8_t address);
    ~I2CDevice();
    
    // Prevent copying
    I2CDevice(const I2CDevice&) = delete;
    I2CDevice& operator=(const I2CDevice&) = delete;
    
    bool open();
    void close();
    bool isOpen() const { return fd >= 0; }
    
    // Existing register-based functions
    bool writeByte(uint8_t reg, uint8_t data);
    bool writeWord(uint8_t reg, uint16_t data);
    bool readByte(uint8_t reg, uint8_t& data);
    bool readWord(uint8_t reg, uint16_t& data);

    // NEW: Raw I2C functions needed for BNO085 SHTP protocol
    bool writeRaw(const uint8_t* data, size_t length);
    bool readRaw(uint8_t* data, size_t length);

    bool readRawAvailable(uint8_t* data, size_t max_length, size_t& bytes_read);


    // Utility functions
    bool isDevicePresent();
    uint8_t getAddress() const { return device_address; }
    const std::string& getLastError() const { return last_error; }

private:
    std::string device_path;
    uint8_t device_address;
    int fd;
    std::string last_error;
    mutable std::mutex i2c_mutex;  // Thread-safe access
    
    void setError(const std::string& error);
    bool setSlaveAddress();
};

#endif // I2C_DEVICE_H