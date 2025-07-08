#include "I2C_Device.h"
#include <iostream>
#include <cstring>
#include <errno.h>
#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>
#include <memory>
#include <functional>

I2CDevice::I2CDevice(const std::string& device_path, uint8_t address)
    : device_path(device_path), device_address(address), fd(-1) {
}

I2CDevice::~I2CDevice() {
    close();
}

bool I2CDevice::open() {
    std::lock_guard<std::mutex> lock(i2c_mutex);
    
    if (fd >= 0) {
        return true;  // Already open
    }
    
    fd = ::open(device_path.c_str(), O_RDWR);
    if (fd < 0) {
        setError("Failed to open " + device_path + ": " + std::strerror(errno));
        return false;
    }
    else
    {
        std::cerr << "Opened " << device_path << std::endl;
    }
    
    if (!setSlaveAddress()) {
        ::close(fd);
        fd = -1;
        return false;
    }
    
    return true;
}

void I2CDevice::close() {
    std::lock_guard<std::mutex> lock(i2c_mutex);
    
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }
}

bool I2CDevice::setSlaveAddress() {
    if (ioctl(fd, I2C_SLAVE, device_address) < 0) {
        setError("Failed to set I2C slave address 0x" + 
                std::to_string(device_address) + ": " + std::strerror(errno));
        return false;
    }
    return true;
}

// =============================================================================
// ORIGINAL REGISTER-BASED FUNCTIONS
// =============================================================================

bool I2CDevice::writeByte(uint8_t reg, uint8_t data) {
    std::lock_guard<std::mutex> lock(i2c_mutex);
    
    if (fd < 0) {
        setError("Device not open");
        return false;
    }
    
    if (!setSlaveAddress()) return false;
    
    uint8_t buffer[2] = {reg, data};
    if (write(fd, buffer, 2) != 2) {
        setError("Failed to write byte to register 0x" + std::to_string(reg));
        return false;
    }
    
    return true;
}

bool I2CDevice::writeWord(uint8_t reg, uint16_t data) {
    std::lock_guard<std::mutex> lock(i2c_mutex);
    
    if (fd < 0) {
        setError("Device not open");
        return false;
    }
    
    if (!setSlaveAddress()) return false;
    
    uint8_t buffer[3];
    buffer[0] = reg;
    buffer[1] = (data >> 8) & 0xFF;  // MSB first
    buffer[2] = data & 0xFF;         // LSB
    
    if (write(fd, buffer, 3) != 3) {
        setError("Failed to write word to register 0x" + std::to_string(reg));
        return false;
    }
    
    return true;
}

bool I2CDevice::readByte(uint8_t reg, uint8_t& data) {
    std::lock_guard<std::mutex> lock(i2c_mutex);
    
    if (fd < 0) {
        setError("Device not open");
        return false;
    }
    
    if (!setSlaveAddress()) return false;
    
    // Write register address
    if (write(fd, &reg, 1) != 1) {
        setError("Failed to write register address");
        return false;
    }
    
    // Read data
    if (read(fd, &data, 1) != 1) {
        setError("Failed to read byte from register 0x" + std::to_string(reg));
        return false;
    }
    
    return true;
}

bool I2CDevice::readWord(uint8_t reg, uint16_t& data) {
    std::lock_guard<std::mutex> lock(i2c_mutex);
    
    if (fd < 0) {
        setError("Device not open");
        return false;
    }
    
    if (!setSlaveAddress()) return false;
    
    // Write register address
    if (write(fd, &reg, 1) != 1) {
        setError("Failed to write register address");
        return false;
    }
    
    // Read data (2 bytes)
    uint8_t buffer[2];
    if (read(fd, buffer, 2) != 2) {
        setError("Failed to read word from register 0x" + std::to_string(reg));
        return false;
    }
    
    data = (buffer[0] << 8) | buffer[1];  // MSB first
    return true;
}



bool I2CDevice::isDevicePresent() 
{
    uint8_t dummy;
    return readByte(0x00, dummy);  // Try to read from register 0
}

void I2CDevice::setError(const std::string& error) {
    last_error = error;
    std::cerr << "I2C Device Error (0x" << std::hex << static_cast<int>(device_address) 
              << "): " << error << std::endl;
}


bool I2CDevice::writeRaw(const uint8_t* data, size_t length) {
    std::lock_guard<std::mutex> lock(i2c_mutex);
    
    if (fd < 0) {
        setError("Device not open");
        return false;
    }
    
    if (!setSlaveAddress()) return false;
    
    // Raw write - no register addressing, just send the data directly
    ssize_t bytes_written = write(fd, data, length);
    if (bytes_written != static_cast<ssize_t>(length)) {
        setError("Failed to write " + std::to_string(length) + " bytes. Wrote: " + std::to_string(bytes_written));
        return false;
    }
    
    return true;
}

bool I2CDevice::readRaw(uint8_t* data, size_t length) {
    std::lock_guard<std::mutex> lock(i2c_mutex);
    
    if (fd < 0) {
        setError("Device not open");
        return false;
    }
    
    if (!setSlaveAddress()) return false;
    
    // Raw read - no register addressing, just read data directly
    ssize_t bytes_read = read(fd, data, length);
    if (bytes_read != static_cast<ssize_t>(length)) {
        // For BNO085, sometimes there's no data available, which is normal
        // Don't treat this as an error initially
        if (bytes_read < 0) {
            setError("Failed to read " + std::to_string(length) + " bytes: " + std::strerror(errno));
            return false;
        }
        // If we read fewer bytes than expected, it might just mean no data available
        return false;
    }
    
    return true;
}

bool I2CDevice::readRawAvailable(uint8_t* data, size_t max_length, size_t& bytes_read) {
    std::lock_guard<std::mutex> lock(i2c_mutex);
    
    if (fd < 0) {
        setError("Device not open");
        return false;
    }
    
    if (!setSlaveAddress()) return false;
    
    // Try to read whatever data is available (non-blocking)
    ssize_t result = read(fd, data, max_length);
    
    if (result < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No data available - this is normal for non-blocking I2C
            bytes_read = 0;
            return true;  // Not an error, just no data
        } else {
            setError("I2C read error: " + std::string(std::strerror(errno)));
            bytes_read = 0;
            return false;
        }
    }
    
    bytes_read = static_cast<size_t>(result);
    return true;
}