// I2C_HAL.cpp - Fixed version for BNO085
#include "I2C_HAL.h"
#include "I2C_Device.h"
#include "sh2_err.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <iostream>

// I2C HAL instance data
typedef struct {
    sh2_Hal_t hal;                    // Base HAL structure (MUST be first member)
    std::unique_ptr<I2CDevice> i2c;   // I2C device wrapper
    
    // Error handling
    std::string lastError;
    
    // Device state
    bool deviceOpen;
    
} I2CHAL_t;

// Forward declarations of HAL function implementations
static int i2cHalOpen(sh2_Hal_t* self);
static void i2cHalClose(sh2_Hal_t* self);
static int i2cHalRead(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len, uint32_t* t_us);
static int i2cHalWrite(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len);
static uint32_t i2cHalGetTimeUs(sh2_Hal_t* self);

// Utility functions
static void setHalError(I2CHAL_t* hal, const std::string& error);

// =============================================================================
// HAL IMPLEMENTATION FUNCTIONS
// =============================================================================

static int i2cHalOpen(sh2_Hal_t* self) {
    I2CHAL_t* hal = (I2CHAL_t*)self;
    
    std::cout << "Opening I2C HAL..." << std::endl;
    
    // Open I2C device
    if (!hal->i2c->open()) {
        setHalError(hal, "Failed to open I2C device: " + hal->i2c->getLastError());
        return SH2_ERR_IO;
    }
    
    // Check if device is present (try a simple read)
    uint8_t testByte;
    if (!hal->i2c->readRaw(&testByte, 1)) {
        std::cout << "Note: Initial device read failed (this may be normal for BNO085)" << std::endl;
    }
    
    std::cout << "BNO085 I2C communication established at address 0x" << std::hex << 
                 static_cast<int>(hal->i2c->getAddress()) << std::dec << std::endl;
    
    // Small delay to let device settle
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Initialize state
    hal->deviceOpen = true;
    
    std::cout << "I2C HAL opened successfully" << std::endl;
    return SH2_OK;
}

static void i2cHalClose(sh2_Hal_t* self) {
    I2CHAL_t* hal = (I2CHAL_t*)self;
    
    std::cout << "Closing I2C HAL..." << std::endl;
    
    if (hal->i2c && hal->deviceOpen) {
        hal->i2c->close();
        hal->deviceOpen = false;
    }
    
    std::cout << "I2C HAL closed" << std::endl;
}

static int i2cHalRead(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len, uint32_t* t_us) {
    I2CHAL_t* hal = (I2CHAL_t*)self;
    
    if (!hal->deviceOpen) {
        return 0;
    }
    
    *t_us = i2cHalGetTimeUs(self);
    
    // For BNO085, try to read data directly
    // The device will send complete SHTP packets when data is available
    if (!hal->i2c->readRaw(pBuffer, len)) {
        // No data available or read failed - return 0 (not an error)
        return 0;
    }
    
    // Validate that we got a reasonable packet
    if (len >= 2) {
        uint16_t packetLength = pBuffer[0] | (pBuffer[1] << 8);
        packetLength &= 0x7FFF;  // Clear continuation bit
        
        // Basic validation
        if (packetLength >= 4 && packetLength <= len) {
            return packetLength;  // Return actual packet length
        }
    }
    
    // If we got here, the data doesn't look like a valid SHTP packet
    // This might be normal noise, so just return 0
    return 0;
}

static int i2cHalWrite(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len) {
    I2CHAL_t* hal = (I2CHAL_t*)self;
    
    if (!hal->deviceOpen) {
        return -1;
    }
    
    if (len > SH2_HAL_MAX_TRANSFER_OUT) {
        setHalError(hal, "Transmit packet too large: " + std::to_string(len));
        return -1;
    }
    
    // Write SHTP packet to I2C
    if (!hal->i2c->writeRaw(pBuffer, len)) {
        setHalError(hal, "I2C write failed: " + hal->i2c->getLastError());
        return -1;
    }
    
    return len;
}

static uint32_t i2cHalGetTimeUs(sh2_Hal_t* self) {
    (void)self;  // Unused parameter
    
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration);
    return static_cast<uint32_t>(microseconds.count());
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

static void setHalError(I2CHAL_t* hal, const std::string& error) {
    hal->lastError = error;
    std::cerr << "I2C HAL Error: " << error << std::endl;
}

// =============================================================================
// PUBLIC API FUNCTIONS
// =============================================================================

sh2_Hal_t* createI2CHAL(const char* i2c_bus, uint8_t i2c_address) {
    if (!i2c_bus) {
        std::cerr << "I2C bus path cannot be null" << std::endl;
        return nullptr;
    }
    
    // Allocate HAL structure
    I2CHAL_t* hal = new I2CHAL_t();
    if (!hal) {
        std::cerr << "Failed to allocate I2C HAL structure" << std::endl;
        return nullptr;
    }
    
    // Initialize all fields
    memset(hal, 0, sizeof(I2CHAL_t));
    
    // Create I2C device instance
    try {
        hal->i2c = std::make_unique<I2CDevice>(i2c_bus, i2c_address);
    } catch (const std::exception& e) {
        std::cerr << "Failed to create I2C device: " << e.what() << std::endl;
        delete hal;
        return nullptr;
    }
    
    // Set up function pointers
    hal->hal.open = i2cHalOpen;
    hal->hal.close = i2cHalClose;
    hal->hal.read = i2cHalRead;
    hal->hal.write = i2cHalWrite;
    hal->hal.getTimeUs = i2cHalGetTimeUs;
    
    // Initialize state
    hal->deviceOpen = false;
    
    return &hal->hal;
}

sh2_Hal_t* createBNO085HAL(void) {
    return createI2CHAL(BNO085_DEFAULT_I2C_BUS, BNO085_DEFAULT_I2C_ADDRESS);
}

void destroyI2CHAL(sh2_Hal_t* hal_ptr) {
    if (hal_ptr) {
        I2CHAL_t* hal = (I2CHAL_t*)hal_ptr;
        
        // Close device if open
        if (hal->deviceOpen) {
            i2cHalClose(hal_ptr);
        }
        
        // Clean up
        delete hal;
    }
}

const char* getI2CHALError(sh2_Hal_t* hal_ptr) {
    if (!hal_ptr) {
        return "Invalid HAL pointer";
    }
    
    I2CHAL_t* hal = (I2CHAL_t*)hal_ptr;
    return hal->lastError.empty() ? nullptr : hal->lastError.c_str();
}

bool isI2CDevicePresent(sh2_Hal_t* hal_ptr) {
    if (!hal_ptr) {
        return false;
    }
    
    I2CHAL_t* hal = (I2CHAL_t*)hal_ptr;
    return hal->i2c && hal->deviceOpen;
}