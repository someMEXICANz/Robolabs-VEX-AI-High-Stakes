#ifndef LSM6DS3__H
#define LSM6DS3__H

#include <cstdint>

namespace LSM6DS3 {

// Device info
constexpr uint8_t DEFAULT_ADDR = 0x6A;      // Default I2C address
constexpr uint8_t CHIP_ID = 0x6A;           // Expected WHOAMI value

// Register addresses
namespace Reg 
{
    constexpr uint8_t WHO_AM_I = 0x0F;                      //  r  | Chip ID register, Its value is fixed at 6Ah.
    constexpr uint8_t CTRL1_XL = 0x10;                      // r/w | Linear acceleration sensor control register 1
    constexpr uint8_t CTRL2_G = 0x11;                       // r/w | Angular rate sensor control register 2
    constexpr uint8_t CTRL3_C = 0x12;                       // r/w | Control register 3
    constexpr uint8_t CTRL6_C = 0x15;                       // r/w | Angular rate sensor control register 6
    constexpr uint8_t CTRL7_G = 0x16;                       // r/w | Angular rate sensor control register 7
    constexpr uint8_t CTRL8_XL = 0x17;                      // r/w | Angular rate sensor control register 8 
    constexpr uint8_t MASTER_CONFIG = 0x1A;                 // r/w | Master configuration register
    constexpr uint8_t WAKEUP_SRC = 0x1B;                    //  r  | Wake up interrupt source register
    constexpr uint8_t STATUS_REG = 0x1E;                    //  r  | The STATUS_REG register is read by the SPI/I2C interface
    constexpr uint8_t OUT_TEMP_L = 0x20;                    //  r  | Temperature sensor output data. The value is expressed as two’s complement sign extended on the MSB.
    constexpr uint8_t OUT_TEMP_H = 0x21;                    //  r  | Temperature sensor output data. L and H registers together express a 16-bit word in two’s complement.
    constexpr uint8_t OUTX_L_G = 0x22;                      //  r  | Pitch axis (X) angular rate value (LSbyte)
    constexpr uint8_t OUTX_H_G = 0x23;                      //  r  | Pitch axis (X) angular rate value (MSbyte)
    constexpr uint8_t OUTY_L_G = 0x24;                      //  r  | Roll axis (Y) angular rate value (LSbyte)
    constexpr uint8_t OUTY_H_G = 0x25;                      //  r  | Roll axis (Y) angular rate value (MSbyte)
    constexpr uint8_t OUTZ_L_G = 0x26;                      //  r  | Yaw axis (Z) angular rate value (LSbyte)
    constexpr uint8_t OUTZ_H_G = 0x27;                      //  r  | Yaw axis (Z) angular rate value (MSbyte)
    constexpr uint8_t OUTX_L_XL = 0x28;                     //  r  | X-axis linear acceleration value (LSbyte)
    constexpr uint8_t OUTX_H_XL = 0x29;                     //  r  | X-axis linear acceleration value (MSbyte)
    constexpr uint8_t OUTY_L_XL = 0x2A;                     //  r  | Y-axis linear acceleration value (LSbyte)
    constexpr uint8_t OUTY_H_XL = 0x2B;                     //  r  | Y-axis linear acceleration value (MSbyte)
    constexpr uint8_t OUTZ_L_XL = 0x2C;                     //  r  | Z-axis linear acceleration value (LSbyte)
    constexpr uint8_t OUTZ_H_XL = 0x2D;                     //  r  | Z-axis linear acceleration value (MSbyte)
    constexpr uint8_t  X_OFS_USR = 0x73;                    // r/w | Accelerometer X-axis user offset correction expressed in two’s complement, weight depends on CTRL6_C(4) bit. The value must be in the range [-127 127].
    constexpr uint8_t  Y_OFS_USR = 0x74;                    // r/w | Accelerometer Y-axis user offset correction expressed in two’s complement, weight depends on CTRL6_C(4) bit. The value must be in the range [-127 127].
    constexpr uint8_t  Z_OFS_USR = 0x75;                    // r/w | Accelerometer X-axis user offset correction expressed in two’s complement, weight depends on CTRL6_C(4) bit. The value must be in the range [-127 127].
         
}

// (CTRL1_XL) 
enum class ODR_XL : uint8_t
{
    POWER_DOWN              = 0x00,     // 0000 0000
    RATE_1_6_HP_12_5_HZ     = 0xB0,     // 1011 0000  
    RATE_12_5_HZ            = 0x10,     // 0001 0000
    RATE_26_HZ              = 0x20,     // 0010 0000
    RATE_52_HZ              = 0x30,     // 0011 0000
    RATE_104_HZ             = 0x40,     // 0100 0000
    RATE_208_HZ             = 0x50,     // 0101 0000
    RATE_416_HZ             = 0x60,     // 0110 0000
    RATE_833_HZ             = 0x70,     // 0111 0000
    RATE_1_66K_HZ           = 0x80,     // 1000 0000
    RATE_3_33K_HZ           = 0x90,     // 1001 0000
    RATE_6_66K_HZ           = 0xA0      // 1010 0000
};

// (CTRL1_XL)
enum class FS_XL : uint8_t
{
    RANGE_2_G  = 0x00, // 0000 00 00
    RANGE_16_G = 0x04, // 0000 01 00
    RANGE_4_G  = 0x08, // 0000 10 00
    RANGE_8_G  = 0x0C  // 0000 11 00
};

// (CTRL2_G)
enum class ODR_G : uint8_t 
{
    POWER_DOWN      = 0x00,     // 0000 0000
    RATE_12_5_HZ    = 0x10,     // 0001 0000
    RATE_26_HZ      = 0x20,     // 0010 0000
    RATE_52_HZ      = 0x30,     // 0011 0000
    RATE_104_HZ     = 0x40,     // 0100 0000
    RATE_208_HZ     = 0x50,     // 0101 0000
    RATE_416_HZ     = 0x60,     // 0110 0000
    RATE_833_HZ     = 0x70,     // 0111 0000
    RATE_1_66K_HZ   = 0x80,     // 1000 0000
    RATE_3_33K_HZ   = 0x90,     // 1001 0000
    RATE_6_66K_HZ   = 0xA0      // 1010 0000
};

// (CTRL2_G)
enum class FS_G : uint8_t 
{
    RANGE_125_DPS  = 0xFF,      // Special Case
    RANGE_245_DPS  = 0x00,      // 0000 00 00
    RANGE_500_DPS  = 0x04,      // 0000 01 00
    RANGE_1000_DPS = 0x08,      // 0000 10 00
    RANGE_2000_DPS = 0x0C       // 0000 11 00
};


namespace SB_MASK
{
    constexpr uint8_t CTRL1_LPF1_BW_SEL   = 0x02; // Accelerometer digital LPF (LPF1)
    constexpr uint8_t CTRL2_FS_125        = 0x02; // Enable gyroscope full-scale at 125 dps
    constexpr uint8_t CTRL3_BOOT          = 0x80; // Reboot memory content
    constexpr uint8_t CTRL3_BDU           = 0x40; // Block data update
    constexpr uint8_t CTRL3_H_LACTIVE     = 0x20; // Interrupt active low
    constexpr uint8_t CTRL3_PP_OD         = 0x10; // Push-pull/open-drain selection
    constexpr uint8_t CTRL3_IF_INC        = 0x04; // Register address automatically incremented
    constexpr uint8_t CTRL3_BLE           = 0x02; // Big/Little Endian Data selection
    constexpr uint8_t CTRL3_SW_RESET      = 0x01; // Software reset
    constexpr uint8_t CTRL6_XL_HM_MODE    = 0x10; // Enable/disable High-performance operating mode for accelerometer
    constexpr uint8_t CTRL6_USR_OFF_W     = 0x08; // Weight of XL user offset bits of registers (0 = 2 -10 g/LSB | 1 = 2 -6 g/LSB)
    constexpr uint8_t CTRL7_G_HM_MODE     = 0x80; // Enable/disable High-performance operating mode for gyroscope
    constexpr uint8_t CTRL8_LPF2_XL_EN    = 0x80; // Accelerometer low-pass filter LPF2 selection
}

} // namespace LSM6DS3

#endif // LSM6DS3_H