#include "I2C_HAL.h"
#include <iostream>
#include <thread>
#include <cstring>
#include <iomanip>

// Static member definitions
std::mutex I2C_HAL::instances_mutex;
std::unordered_map<sh2_Hal_t*, I2C_HAL*> I2C_HAL::hal_instances;

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
    
    // Register this instance for callback lookups
    registerInstance();
}

I2C_HAL::~I2C_HAL() {
    unregisterInstance();
    
    if (i2c_device && i2c_device->isOpen()) {
        i2c_device->close();
    }
}

void I2C_HAL::registerInstance() {
    std::lock_guard<std::mutex> lock(instances_mutex);
    hal_instances[&hal_interface] = this;
}

void I2C_HAL::unregisterInstance() {
    std::lock_guard<std::mutex> lock(instances_mutex);
    hal_instances.erase(&hal_interface);
}

I2C_HAL* I2C_HAL::getHalInstance(sh2_Hal_t *self) {
    std::lock_guard<std::mutex> lock(instances_mutex);
    auto it = hal_instances.find(self);
    return (it != hal_instances.end()) ? it->second : nullptr;
}

bool I2C_HAL::testConnection() {
    if (!i2c_device->open()) {
        std::cerr << "Failed to open I2C device: " << i2c_device->getLastError() << std::endl;
        return false;
    }
    
    return i2c_device->isDevicePresent();
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
    // According to the Adafruit code: {5, 0, 1, 0, 1}
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
    
    if (packet_size < 4) {
        std::cerr << "HAL: Invalid packet size: " << packet_size << std::endl;
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
    
    // Debug: Print packet contents for advertisement analysis
    static int packet_count = 0;
    packet_count++;
    
    if (packet_count <= 20) {  // Only show first 20 packets
        std::cout << "Packet " << packet_count << " (size=" << packet_size << "): ";
        for (int i = 0; i < std::min((int)packet_size, 16); i++) {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)pBuffer[i] << " ";
        }
        if (packet_size > 16) std::cout << "...";
        std::cout << std::dec << std::endl;
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