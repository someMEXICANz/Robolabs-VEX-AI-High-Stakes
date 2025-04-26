#ifndef LIS3MDL_REGISTERS_H
#define LIS3MDL_REGISTERS_H

#include <cstdint>

namespace LIS3MDL {

// Device info
constexpr uint8_t DEFAULT_ADDR = 0x1C;      // Default I2C address
constexpr uint8_t CHIP_ID = 0x3D;           // Expected WHOAMI value

// Register addresses
namespace Reg 
{
    constexpr uint8_t OFFSET_X_REG_L = 0x05; // Hard-iron offset X low byte
    constexpr uint8_t OFFSET_X_REG_H = 0x06; // Hard-iron offset X high byte
    constexpr uint8_t OFFSET_Y_REG_L = 0x07; // Hard-iron offset Y low byte
    constexpr uint8_t OFFSET_Y_REG_H = 0x08; // Hard-iron offset Y high byte
    constexpr uint8_t OFFSET_Z_REG_L = 0x09; // Hard-iron offset Z low byte
    constexpr uint8_t OFFSET_Z_REG_H = 0x0A; // Hard-iron offset Z high byte
    constexpr uint8_t WHO_AM_I = 0x0F;       // Device identification
    constexpr uint8_t CTRL_REG1 = 0x20;      // Control register 1
    constexpr uint8_t CTRL_REG2 = 0x21;      // Control register 2
    constexpr uint8_t CTRL_REG3 = 0x22;      // Control register 3
    constexpr uint8_t CTRL_REG4 = 0x23;      // Control register 4
    constexpr uint8_t CTRL_REG5 = 0x24;      // Control register 5
    constexpr uint8_t STATUS_REG = 0x27;     // Status register
    constexpr uint8_t OUT_X_L = 0x28;        // X-axis output, low byte
    constexpr uint8_t OUT_X_H = 0x29;        // X-axis output, high byte
    constexpr uint8_t OUT_Y_L = 0x2A;        // Y-axis output, low byte
    constexpr uint8_t OUT_Y_H = 0x2B;        // Y-axis output, high byte
    constexpr uint8_t OUT_Z_L = 0x2C;        // Z-axis output, low byte
    constexpr uint8_t OUT_Z_H = 0x2D;        // Z-axis output, high byte
    constexpr uint8_t TEMP_OUT_L = 0x2E;     // Temperature output, low byte
    constexpr uint8_t TEMP_OUT_H = 0x2F;     // Temperature output, high byte
    constexpr uint8_t INT_CFG = 0x30;        // Interrupt configuration
    constexpr uint8_t INT_SRC = 0x31;        // Interrupt source
    constexpr uint8_t INT_THS_L = 0x32;      // Interrupt threshold, low byte
    constexpr uint8_t INT_THS_H = 0x33;      // Interrupt threshold, high byte
}

// Performance mode bits (CTRL_REG1)
enum class PerformanceMode : uint8_t 
{
    LOW_POWER  = 0x00, // 0000 0000
    MEDIUM     = 0x20, // 0010 0000
    HIGH       = 0x40, // 0100 0000
    ULTRA_HIGH = 0x60  // 0110 0000
};

// Output data rate (CTRL_REG1)
enum class DataRate : uint8_t 
{
    RATE_0_625_HZ = 0x00, // 0000 0000
    RATE_1_25_HZ  = 0x04, // 0000 0100
    RATE_2_5_HZ   = 0x08, // 0000 1000
    RATE_5_HZ     = 0x0C, // 0000 1100
    RATE_10_HZ    = 0x10, // 0001 0000
    RATE_20_HZ    = 0x14, // 0001 0100
    RATE_40_HZ    = 0x18, // 0001 1000
    RATE_80_HZ    = 0x1C, // 0001 1100
    // Fast ODR enabled modes
    RATE_155_HZ   = 0x02, // FAST_ODR=1, DO=00
    RATE_300_HZ   = 0x02, // FAST_ODR=1, DO=00 (with HIGH performance)
    RATE_560_HZ   = 0x02, // FAST_ODR=1, DO=00 (with MEDIUM performance)
    RATE_1000_HZ  = 0x02  // FAST_ODR=1, DO=00 (with LOW_POWER performance)
};

// CTRL_REG1 bit masks
namespace CTRL1_Bits 
{
    constexpr uint8_t TEMP_EN  = 0x80; // Temperature sensor enable
    constexpr uint8_t OM_MASK  = 0x60; // X/Y operating mode mask
    constexpr uint8_t DO_MASK  = 0x1C; // Output data rate mask
    constexpr uint8_t FAST_ODR = 0x02; // Fast ODR enable
    constexpr uint8_t ST       = 0x01; // Self-test enable
}

// Full scale selection (CTRL_REG2)
enum class Range : uint8_t {
    RANGE_4_GAUSS  = 0x00, // 0000 0000
    RANGE_8_GAUSS  = 0x20, // 0010 0000
    RANGE_12_GAUSS = 0x40, // 0100 0000
    RANGE_16_GAUSS = 0x60  // 0110 0000
};

// CTRL_REG2 bit masks
namespace CTRL2_Bits {
    constexpr uint8_t FS_MASK  = 0x60; // Full-scale selection mask
    constexpr uint8_t REBOOT   = 0x08; // Reboot memory content
    constexpr uint8_t SOFT_RST = 0x04; // Configuration registers and user register reset
}

// Operating mode (CTRL_REG3)
enum class OperationMode : uint8_t {
    CONTINUOUS = 0x00, // Continuous conversion
    SINGLE     = 0x01, // Single conversion
    POWERDOWN  = 0x03  // Power-down mode
};

// CTRL_REG3 bit masks
namespace CTRL3_Bits {
    constexpr uint8_t LP      = 0x20; // Low-power mode
    constexpr uint8_t SIM     = 0x04; // SPI serial interface mode selection
    constexpr uint8_t MD_MASK = 0x03; // Operating mode selection mask
}

// STATUS_REG bit masks
namespace STATUS_Bits {
    constexpr uint8_t ZYXOR = 0x80; // X, Y, Z-axis data overrun
    constexpr uint8_t ZOR   = 0x40; // Z-axis data overrun
    constexpr uint8_t YOR   = 0x20; // Y-axis data overrun
    constexpr uint8_t XOR   = 0x10; // X-axis data overrun
    constexpr uint8_t ZYXDA = 0x08; // X, Y, Z-axis new data available
    constexpr uint8_t ZDA   = 0x04; // Z-axis new data available
    constexpr uint8_t YDA   = 0x02; // Y-axis new data available
    constexpr uint8_t XDA   = 0x01; // X-axis new data available
}

} // namespace LIS3MDL

#endif // LIS3MDL_REGISTERS_H