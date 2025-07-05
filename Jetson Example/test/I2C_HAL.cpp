#include "I2C_HAL.h"
#include <iostream>
#include <thread>

I2C_HAL::I2C_HAL(const std::string& i2c_device_path, uint8_t i2c_address) 
    : start_time(std::chrono::steady_clock::now()) {
    
    // Create I2C device
    i2c_device = std::make_unique<I2CDevice>(i2c_device_path, i2c_address);
    
    // Initialize HAL interface structure
    hal_interface.open = hal_open;
    hal_interface.close = hal_close;
    hal_interface.read = hal_read;
    hal_interface.write = hal_write;
    hal_interface.getTimeUs = hal_getTimeUs;
    
    // Store pointer to this instance in HAL structure
    // We'll use a hack: store 'this' pointer in an unused field
    // (This is a common pattern for C callback interfaces)
    hal_interface.cookie = this;  // Assuming there's a cookie field, or we can add one
}

I2C_HAL::~I2C_HAL() {
    if (i2c_device && i2c_device->isOpen()) {
        i2c_device->close();
    }
}

bool I2C_HAL::testConnection() {
    if (!i2c_device->open()) {
        std::cerr << "Failed to open I2C device: " << i2c_device->getLastError() << std::endl;
        return false;
    }
    
    // Try to detect if device is present
    return i2c_device->isDevicePresent();
}

I2C_HAL* I2C_HAL::getHalInstance(sh2_Hal_t *self) {
    // We need to store 'this' pointer somewhere in sh2_Hal_t
    // Let's check if there's a cookie field we can use, or we might need to modify sh2_hal.h
    return static_cast<I2C_HAL*>(self->cookie);
}

uint32_t I2C_HAL::getCurrentTimeUs() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now - start_time;
    return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(duration).count());
}

// HAL Callback implementations
int I2C_HAL::hal_open(sh2_Hal_t *self) {
    I2C_HAL* hal = getHalInstance(self);
    if (!hal || !hal->i2c_device) {
        return -1;
    }
    
    if (!hal->i2c_device->open()) {
        std::cerr << "HAL: Failed to open I2C device" << std::endl;
        return -1;
    }
    
    // BNO085 requires a soft reset sequence over I2C
    // According to the Adafruit code, we need to send: {5, 0, 1, 0, 1}
    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
    
    // Try multiple attempts like the Arduino code does
    for (int attempts = 0; attempts < 5; attempts++) {
        if (hal->i2c_device->writeRaw(softreset_pkt, 5)) {
            // Wait for reset to complete
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            return 0;  // Success
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    
    std::cerr << "HAL: Failed to send soft reset to BNO085" << std::endl;
    return -1;
}

void I2C_HAL::hal_close(sh2_Hal_t *self) {
    I2C_HAL* hal = getHalInstance(self);
    if (hal && hal->i2c_device) {
        hal->i2c_device->close();
    }
}

int I2C_HAL::hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    I2C_HAL* hal = getHalInstance(self);
    if (!hal || !hal->i2c_device || !hal->i2c_device->isOpen()) {
        return 0;  // No data available
    }
    
    // Set timestamp
    *t_us = hal->getCurrentTimeUs();
    
    // BNO085 I2C protocol: First read 4-byte header to get packet length
    uint8_t header[4];
    if (!hal->i2c_device->readRaw(header, 4)) {
        return 0;  // No data or error
    }
    
    // Parse packet length from header
    uint16_t packet_size = static_cast<uint16_t>(header[0]) | (static_cast<uint16_t>(header[1]) << 8);
    packet_size &= ~0x8000;  // Clear continuation bit
    
    if (packet_size > len) {
        std::cerr << "HAL: Packet too large for buffer (" << packet_size << " > " << len << ")" << std::endl;
        return 0;
    }
    
    // Copy header to output buffer
    std::memcpy(pBuffer, header, 4);
    
    // Read remaining packet data if any
    if (packet_size > 4) {
        if (!hal->i2c_device->readRaw(pBuffer + 4, packet_size - 4)) {
            std::cerr << "HAL: Failed to read packet payload" << std::endl;
            return 0;
        }
    }
    
    return packet_size;
}

int I2C_HAL::hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    I2C_HAL* hal = getHalInstance(self);
    if (!hal || !hal->i2c_device || !hal->i2c_device->isOpen()) {
        return 0;
    }
    
    if (hal->i2c_device->writeRaw(pBuffer, len)) {
        return len;
    }
    
    return 0;  // Write failed
}

uint32_t I2C_HAL::hal_getTimeUs(sh2_Hal_t *self) {
    I2C_HAL* hal = getHalInstance(self);
    if (!hal) {
        return 0;
    }
    
    return hal->getCurrentTimeUs();
}