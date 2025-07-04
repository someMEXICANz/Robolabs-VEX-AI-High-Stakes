#ifndef LIS3MDL_H
#define LIS3MDL_H

#include <cstdint>
namespace LIS3MDL {

// Device info
constexpr uint8_t DEFAULT_ADDR = 0x1C;      // Default I2C address
constexpr uint8_t CHIP_ID = 0x3D;           // Expected WHOAMI value

// Register addresses
namespace Reg 
{
    constexpr uint8_t OFFSET_X_REG_L_M = 0x05;              // r/w | Hard-iron offset X low byte
    constexpr uint8_t OFFSET_X_REG_H_M = 0x06;              // r/w | Hard-iron offset X high byte
    constexpr uint8_t OFFSET_Y_REG_L_M = 0x07;              // r/w | Hard-iron offset Y low byte
    constexpr uint8_t OFFSET_Y_REG_H_M = 0x08;              // r/w | Hard-iron offset Y high byte
    constexpr uint8_t OFFSET_Z_REG_L_M = 0x09;              // r/w | Hard-iron offset Z low byte
    constexpr uint8_t OFFSET_Z_REG_H_M = 0x0A;              // r/w | Hard-iron offset Z high byte
    constexpr uint8_t WHO_AM_I = 0x0F;                      //  r  | Chip ID register, Its value is fixed at 3Dh.
    constexpr uint8_t CTRL_REG1 = 0x20;                     // r/w | Control register 1
    constexpr uint8_t CTRL_REG2 = 0x21;                     // r/w | Control register 2
    constexpr uint8_t CTRL_REG3 = 0x22;                     // r/w | Control register 3
    constexpr uint8_t CTRL_REG4 = 0x23;                     // r/w | Control register 4
    constexpr uint8_t CTRL_REG5 = 0x24;                     // r/w | Control register 5
    constexpr uint8_t STATUS_REG = 0x27;                    //  r  | Status register
    constexpr uint8_t OUT_X_L = 0x28;                       //  r  | X-axis output, low byte
    constexpr uint8_t OUT_X_H = 0x29;                       //  r  | X-axis output, high byte
    constexpr uint8_t OUT_Y_L = 0x2A;                       //  r  | Y-axis output, low byte
    constexpr uint8_t OUT_Y_H = 0x2B;                       //  r  | Y-axis output, high byte
    constexpr uint8_t OUT_Z_L = 0x2C;                       //  r  | Z-axis output, low byte
    constexpr uint8_t OUT_Z_H = 0x2D;                       //  r  | Z-axis output, high byte
    constexpr uint8_t TEMP_OUT_L = 0x2E;                    //  r  | Temperature output, low byte
    constexpr uint8_t TEMP_OUT_H = 0x2F;                    //  r  | Temperature output, high byte
}

// (CTRL_REG1)
enum class OM : uint8_t 
{
    LOW_POWER  = 0x00, // 0 00 00000
    MEDIUM     = 0x20, // 0 01 00000
    HIGH       = 0x40, // 0 10 00000
    ULTRA_HIGH = 0x60  // 0 11 00000
};

// CTRL_REG1)
enum class DO : uint8_t 
{
    RATE_0_625_HZ = 0x00, // 000 000 00
    RATE_1_25_HZ  = 0x04, // 000 001 00
    RATE_2_5_HZ   = 0x08, // 000 010 00
    RATE_5_HZ     = 0x0C, // 000 011 00
    RATE_10_HZ    = 0x10, // 000 100 00
    RATE_20_HZ    = 0x14, // 000 101 00
    RATE_40_HZ    = 0x18, // 000 110 00
    RATE_80_HZ    = 0x1C, // 000 111 00
};

// (CTRL_REG2)
enum class FS : uint8_t 
{
    RANGE_4_GAUSS  = 0x00, // 0 00 00000
    RANGE_8_GAUSS  = 0x20, // 0 01 00000
    RANGE_12_GAUSS = 0x40, // 0 10 00000
    RANGE_16_GAUSS = 0x60  // 0 11 00000
};

// (CTRL_REG3)
enum class MD : uint8_t 
{
    CONTINUOUS = 0x00, // 000000 00
    SINGLE     = 0x01, // 000000 01
    POWERDOWN  = 0x03  // 000000 11
};
// (CTRL_REG4)
enum class OMZ : uint8_t
{
    LOW_POWER  = 0x00, // 0000 00 00
    MEDIUM     = 0x04, // 0000 01 00
    HIGH       = 0x08, // 0000 10 00
    ULTRA_HIGH = 0x0C  // 0000 11 00
};

namespace SB_MASK
{
    constexpr uint8_t CTRL1_TEMP_EN       = 0x80;   // Enable/Disable temperature sensor
    constexpr uint8_t CTRL1_FAST_ODR      = 0x02;   // Enable/Disable FAST_ODR which allows for data rates higher than 80 Hz
    constexpr uint8_t CTRL2_REBOOT        = 0x08;   // Reboots memory content
    constexpr uint8_t CTRL2_SOFT_RST      = 0x04;   // Configuration registers and user register reset function.
    constexpr uint8_t CTRL4_BLE           = 0x02;   // Big/little endian data selection
    constexpr uint8_t CTRL5_FAST_READ     = 0x80;   // Enable/Disable FAST READ to allow reading the high part of DATA OUT
    constexpr uint8_t CTRL5_BDU           = 0x40;   // Block data update for magnetic data  
}

} // namespace LIS3MDL

#endif // LIS3MDL_H