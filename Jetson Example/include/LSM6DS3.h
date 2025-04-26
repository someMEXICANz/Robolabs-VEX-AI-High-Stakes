#ifndef LSM6DS3__H
#define LSM6DS3__H

#include <cstdint>

namespace LSM6DS3 {

// Device info
constexpr uint8_t I2C_ADDR_DEFAULT = 0x6A;  // Default I2C address
constexpr uint8_t CHIP_ID = 0x6A;           // Expected WHOAMI value

// Register addresses
namespace Reg 
{
    // constexpr uint8_t FUNC_CFG_ACCESS = 0x01;               // r/w | Enable embedded functions register
    // constexpr uint8_t SENSOR_SYNC_TIME_FRAME = 0x04;        // r/w | Sensor synchronization time frame register 
    // constexpr uint8_t SENSOR_SYNC_RES_RATIO = 0x05;         // r/w | Sensor synchronization resolution ratio 
    // constexpr uint8_t FIFO_CTRL1 = 0x06;                    // r/w | FIFO control register
    // constexpr uint8_t FIFO_CTRL2 = 0x08;                    // r/w | FIFO control register
    // constexpr uint8_t FIFO_CTRL3 = 0x08;                    // r/w | FIFO control register
    // constexpr uint8_t FIFO_CTRL4 = 0x09;                    // r/w | FIFO control register
    // constexpr uint8_t FIFO_CTRL5 = 0x0A;                    // r/w | FIFO control register
    // constexpr uint8_t DRDY_PULSE_CFG_G = 0x0B;              // r/w | DataReady configuration register
    // constexpr uint8_t INT1_CTRL = 0x0D;                     // r/w | INT1 pad control register
    // constexpr uint8_t INT2_CTRL = 0x0E;                     // r/w | INT2 pad control register
    constexpr uint8_t WHO_AM_I = 0x0F;                      //  r  | Chip ID register, Its value is fixed at 6Ah.
    constexpr uint8_t CTRL1_XL = 0x10;                      // r/w | Linear acceleration sensor control register 1
    constexpr uint8_t CTRL2_G = 0x11;                       // r/w | Angular rate sensor control register 2
    constexpr uint8_t CTRL3_C = 0x12;                       // r/w | Control register 3
    // constexpr uint8_t CTRL4_C = 0x13;                       // r/w | Control register 4
    // constexpr uint8_t CTRL5_C = 0x14;                       // r/w | Control register 5
    constexpr uint8_t CTRL6_C = 0x15;                       // r/w | Angular rate sensor control register 6
    constexpr uint8_t CTRL7_G = 0x16;                       // r/w | Angular rate sensor control register 7
    constexpr uint8_t CTRL8_XL = 0x17;                      // r/w | Linear acceleration sensor control register 8
    // constexpr uint8_t CTRL9_XL = 0x18;                      // r/w | Linear acceleration sensor control register 9
    // constexpr uint8_t CTRL10_C = 0x19;                      // r/w | Control register 10     
    constexpr uint8_t MASTER_CONFIG = 0x1A;                 // r/w | Master configuration register
    constexpr uint8_t WAKEUP_SRC = 0x1B;                    //  r  | Wake up interrupt source register
    // constexpr uint8_t TAP_SRC = 0x1C;                       //  r  | Tap source registe
    // constexpr uint8_t D6D_SRC = 0x1D;                       //  r  | Portrait, landscape, face-up and face-down source register
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
    POWER_DOWN      = 0x00,     // 0000 0000
    LP_RATE_1_6_HZ  = 0xB0,     // 1011 0000  
    HP_RATE_12_5_HZ = 0XB0,     // 1011 0000
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

// (CTRL1_XL)
enum class FS_XL : uint8_t
{
    RANGE_2_G  = 0x00, // 0000 0000
    RANGE_16_G = 0x04, // 0000 0100
    RANGE_4_G  = 0x08, // 0000 1000
    RANGE_8_G  = 0x0C  // 0000 1100
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
    RANGE_245_DPS  = 0x00,      // 0000 0000
    RANGE_500_DPS  = 0x04,      // 0000 0100
    RANGE_1000_DPS = 0x08,      // 0000 1000
    RANGE_2000_DPS = 0x0C       // 0000 1100
};


namespace Mask
{
    constexpr uint8_t CTRL2_FS_125        = 0x02; // Enable gyroscope full-scale at 125 dps
    constexpr uint8_t CTRL3_BOOT          = 0x80; // Reboot memory content
    constexpr uint8_t CTRL3_BDU           = 0x40; // Block data update
    constexpr uint8_t CTRL3_H_LACTIVE     = 0x20; // Interrupt active low
    constexpr uint8_t CTRL3_PP_OD         = 0x10; // Push-pull/open-drain selection
    constexpr uint8_t CTRL3_SIM           = 0x08; // SPI serial interface mode
    constexpr uint8_t CTRL3_IF_INC        = 0x04; // Register address automatically incremented
    constexpr uint8_t CTRL3_SW_RESET      = 0x01; // Software reset
    constexpr uint8_t CTRL6_XL_HM_MODE    = 0x10; // Enable/disable High-performance operating mode for accelerometer
    constexpr uint8_t CTRL6_USR_OFF_W     = 0x08; // Weight of XL user offset bits of registers (0 = 2 -10 g/LSB | 1 = 2 -6 g/LSB)
    constexpr uint8_t CTRL7_G_HM_MODE     = 0x80; // Enable/disable High-performance operating mode for gyroscope
}



} // namespace LSM6DS3

#endif // LSM6DS3_REGISTERS_H