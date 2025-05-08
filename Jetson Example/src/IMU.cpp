#include "IMU.h"
#include "IMUDataTypes.h"

using std::chrono::system_clock;

IMU::IMU(const char* i2c_device_)
    : i2c_device(i2c_device_),
      lsm6ds3_fd(-1),
      lis3mdl_fd(-1),
      accel_scale(0.0f), 
      gyro_scale(0.0f),  
      mag_scale(0.0f),    
      running(false),
      initialized(false),
      current_data{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f, std::chrono::system_clock::now(),false},
      current_orientation{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, std::chrono::system_clock::now(),false},
      magCalibrated(false),
      accelCalibrated(false)
{
    if(initialize()) 
    {
       start();
    }
}

IMU::~IMU() 
{
    stop();
    
    if (lsm6ds3_fd>= 0) 
    {
        close(lsm6ds3_fd);
        lsm6ds3_fd = -1;
    }
    if(lis3mdl_fd >= 0)
    {
        close(lis3mdl_fd);
        lis3mdl_fd = -1;
    }
}

bool IMU::start() 
{
    if (running) 
    {
        std::cerr << "IMU thread is already running" << std::endl;
        return true;
    }
   
    try{

        running = true;
        read_thread= std::make_unique<std::thread>(&IMU::readLoop, this);
    } catch (const std::exception& e) 
    {
        std::cerr << "Failed to start IMU read thread: " << e.what() << std::endl;
        running = false;
        return false;  
    }
    
    return true;
}

void IMU::stop() 
{
    running = false;
    
    if (read_thread && read_thread->joinable()) 
    {
        read_thread->join();
        read_thread.reset();
    }
}

bool IMU::restart() 
{
    stop();
    return start();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool IMU::writeRegister(int fd, uint8_t reg, uint8_t value) 
{
    uint8_t buf[2] = {reg, value};
    if (write(fd, buf, 2) != 2) 
    {
        std::cerr << "Failed to write to register 0x" << std::hex << (int)reg << std::dec << std::endl;
        return false;
    }
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


uint8_t IMU::readRegister(int fd, uint8_t reg) 
{
    uint8_t value = 0;
    
    // Write the register address
    if (write(fd, &reg, 1) != 1) 
    {
        std::cerr << "Failed to write register address for reading" << std::endl;
        return 0;
    }
    
    // Read the register value
    if (read(fd, &value, 1) != 1) 
    {
        std::cerr << "Failed to read register value" << std::endl;
        return 0;
    }
    
    return value;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool IMU::readRegisters(int fd, uint8_t reg, uint8_t* buffer, uint8_t length) 
{
    // Set up the I2C message structures for a combined transaction
    struct i2c_msg messages[2];
    struct i2c_rdwr_ioctl_data ioctl_data;
    
    // First message: Write the register address (no STOP condition)
    messages[0].addr = (fd == lsm6ds3_fd) ? LSM6DS3::DEFAULT_ADDR : LIS3MDL::DEFAULT_ADDR;
    messages[0].flags = 0;        // Write
    messages[0].len = 1;
    messages[0].buf = &reg;
    
    // Second message: Read the data (with implied START condition)
    messages[1].addr = (fd == lsm6ds3_fd) ? LSM6DS3::DEFAULT_ADDR : LIS3MDL::DEFAULT_ADDR;
    messages[1].flags = I2C_M_RD; // Read
    messages[1].len = length;
    messages[1].buf = buffer;
    
    // Set up the ioctl data
    ioctl_data.msgs = messages;
    ioctl_data.nmsgs = 2;
    
    // Execute the combined write-then-read transaction
    if (ioctl(fd, I2C_RDWR, &ioctl_data) < 0) 
    {
        std::cerr << "Failed to perform I2C combined transaction" << std::endl;
        return false;
    }
    
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool IMU::setBit(int fd, uint8_t reg, uint8_t mask, bool enable) 
{
    // Read current register value
    uint8_t current_value = readRegister(fd, reg);
    
    // Set or clear the specified bits
    uint8_t new_value;
    if (enable) 
        new_value = current_value | mask;
    else 
        new_value = current_value & ~mask;
    
    // Write the new value
    return writeRegister(fd, reg, new_value);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool IMU::initialize() 
{
    initialized = false;

    // Open I2C device for LSM6DS3
    lsm6ds3_fd = open(i2c_device, O_RDWR);
    if (lsm6ds3_fd < 0) 
    {
        std::cerr << "Failed to open " << i2c_device << " for LSM6DS3TRC" << std::endl;
        return false;
    }
    
    // Set I2C slave address for LSM6DS3TRC
    if (ioctl(lsm6ds3_fd, I2C_SLAVE, LSM6DS3::DEFAULT_ADDR) < 0) 
    {
        std::cerr << "Failed to set LSM6DS3TRC I2C slave address" << std::endl;
        close(lsm6ds3_fd);
        lsm6ds3_fd = -1;
        return false;
    }
    
    // Open I2C device for LIS3MDL
    lis3mdl_fd = open(i2c_device, O_RDWR);
    if (lis3mdl_fd < 0) 
    {
        std::cerr << "Failed to open " << i2c_device << " for LIS3MDL" << std::endl;
        return false;
    }
    
    // Set I2C slave address for LIS3MDL
    if (ioctl(lis3mdl_fd, I2C_SLAVE, LIS3MDL::DEFAULT_ADDR) < 0) 
    {
        std::cerr << "Failed to set LIS3MDL I2C slave address" << std::endl;
        close(lis3mdl_fd);
        lis3mdl_fd = -1;
        return false;
    }

    // Check LSM6DS3 WHO_AM_I register
    uint8_t lsm6ds3_id = readRegister(lsm6ds3_fd, LSM6DS3::Reg::WHO_AM_I);
    if (lsm6ds3_id != LSM6DS3::CHIP_ID) 
    {
        std::cerr << "LSM6DS3TRC not found (got ID 0x" << std::hex << (int)lsm6ds3_id 
                  << ", expected 0x" << (int)LSM6DS3::CHIP_ID << ")" << std::dec << std::endl;
        return false;
    }
    
    // Check LIS3MDL WHO_AM_I register
    uint8_t lis_id = readRegister(lis3mdl_fd, LIS3MDL::Reg::WHO_AM_I);
    if (lis_id != LIS3MDL::CHIP_ID) 
    {
        std::cerr << "LIS3MDL not found (got ID 0x" << std::hex << (int)lis_id 
                  << ", expected 0x" << (int)LIS3MDL::CHIP_ID << ")" << std::dec << std::endl;
        return false;
    }

    configureSettings();
    kalman_filter.initialize();

    initialized = true;
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void IMU::configureSettings() 
{
    
    // Reset devices
    setBit(lsm6ds3_fd,LSM6DS3::Reg::CTRL3_C, LSM6DS3::SB_MASK::CTRL3_SW_RESET, true);         
    setBit(lis3mdl_fd, LIS3MDL::Reg::CTRL_REG2, LIS3MDL::SB_MASK::CTRL2_SOFT_RST, true);  

    // Set Block Data Update bit (prevents MSB/LSB data corruption during reads)
    setBit(lsm6ds3_fd, LSM6DS3::Reg::CTRL3_C, LSM6DS3::SB_MASK::CTRL3_BDU, false);
    setMagnetometerMode(LIS3MDL::MD::CONTINUOUS);

    
    setAccelerometerRate(LSM6DS3::ODR_XL::RATE_208_HZ);
    setGyroscopeRate(LSM6DS3::ODR_G::RATE_208_HZ);
    
    setAccelerometerRange(LSM6DS3::FS_XL::RANGE_4_G);
    setGyroscopeRange(LSM6DS3::FS_G::RANGE_245_DPS);

    // Enables First accel Low Pass Filter
    setBit(lsm6ds3_fd, LSM6DS3::Reg::CTRL1_XL, LSM6DS3::SB_MASK::CTRL1_LPF1_BW_SEL, true);
    // Enables second accel Low Pass Filter
    setBit(lsm6ds3_fd, LSM6DS3::Reg::CTRL8_XL, LSM6DS3::SB_MASK::CTRL8_LPF2_XL_EN, true);
    // Enables high performance mode for accel
    setBit(lsm6ds3_fd, LSM6DS3::Reg::CTRL6_C, LSM6DS3::SB_MASK::CTRL6_XL_HM_MODE, true);
    // Enables high performance mode for gyro
    setBit(lsm6ds3_fd, LSM6DS3::Reg::CTRL7_G, LSM6DS3::SB_MASK::CTRL7_G_HM_MODE, true);
    // Enables gyro digital High Pass Filter 
    setBit(lsm6ds3_fd, LSM6DS3::Reg::CTRL7_G, LSM6DS3::SB_MASK::CTRL7_HP_EN_G, true);


    setMagnetometerRate(LIS3MDL::DO::RATE_80_HZ);
    setMagnetometerRange(LIS3MDL::FS::RANGE_4_GAUSS);

    //Turns on FAST_ODR 
    setBit(lis3mdl_fd, LIS3MDL::Reg::CTRL_REG1, LIS3MDL::SB_MASK::CTRL1_FAST_ODR,true);
    setMagnetometerPower(LIS3MDL::OM::HIGH);

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void IMU::readLoop()
{
    std::cerr << "IMU read loop started" << std::endl;
    
    const std::chrono::milliseconds read_interval(10); // 100Hz update rate
    
    while (running) 
    {
         std::chrono::time_point start_time = std::chrono::system_clock::now();
        
        if (!initialized) 
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            break;
        }
        readData();
        std::chrono::duration elapsed = std::chrono::system_clock::now() - start_time;
        if (elapsed < read_interval) 
            std::this_thread::sleep_for(read_interval - elapsed);
    }
    running = false;
    std::cerr << "IMU read loop stopped" << std::endl;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool IMU::readData() 
{
    if (!initialized) 
    {
        return false;
    }

    bool error = false;

    uint8_t LSM6DS3_buffer[14];
    int16_t LSM6DS3_data[7]; 

    // Read temperature and accel/gyro data from LSM6DS3
    if (!readRegisters(lsm6ds3_fd, LSM6DS3::Reg::OUT_TEMP_L, LSM6DS3_buffer, 14)) 
    {
        error = true;
    }
    // Parse the LSM6DS3 data
    for (int i = 0; i < 7; i++)
    {
        LSM6DS3_data[i] = (LSM6DS3_buffer[i*2+1] << 8) | LSM6DS3_buffer[i*2];
    }
     
    // Convert temperature (LSB = 256 per degree C, 25°C = 0)
    float temp = 25.0f + ((float)LSM6DS3_data[0] / 256.0f);
    
    float gx = (float)LSM6DS3_data[1] * gyro_scale / 1000;
    float gy = (float)LSM6DS3_data[2] * gyro_scale / 1000;
    float gz = (float)LSM6DS3_data[3] * gyro_scale / 1000;
    
    float ax = (float)LSM6DS3_data[4] * accel_scale / 1000;
    float ay = (float)LSM6DS3_data[5] * accel_scale / 1000;
    float az = (float)LSM6DS3_data[6] * accel_scale / 1000;


    
    uint8_t LIS3MDL_buffer[6];
    int16_t LIS3MDL_data[3]; 

    // Read magnetometer data from LIS3MDL
    if (!readRegisters(lis3mdl_fd, LIS3MDL::Reg::OUT_X_L, LIS3MDL_buffer, 6)) 
    {
        error = true;
    }

    // Parse the LIS3MDL data
    for (int i = 0; i < 3; i++) 
    {
        LIS3MDL_data[i] = (LIS3MDL_buffer[i*2+1] << 8) | LIS3MDL_buffer[i*2];
    }

    float mx = (float)LIS3MDL_data[0] / mag_scale ;
    float my = (float)LIS3MDL_data[1] / mag_scale ;
    float mz = (float)LIS3MDL_data[2] / mag_scale ;

    if(!error)
    {
        // Update data structure with mutex protection
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            current_data.ax = ax;
            current_data.ay = ay;
            current_data.az = az;
            current_data.gx = gx;
            current_data.gy = gy;
            current_data.gz = gz;
            current_data.mx = mx;
            current_data.my = my;
            current_data.mz = mz;
            current_data.temperature = temp;
            current_data.timestamp = std::chrono::system_clock::now();
            current_data.valid = true;
            
            // Add to history buffer (as a queue)
            data_history.push_back(current_data);
            if (data_history.size() > HISTORY_BUFFER_SIZE) 
            {
                data_history.pop_front(); 
            }

        }
        updateOrientation();
        return true;
    }
    else
    {
        // Update data structure with mutex protection
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            current_data.timestamp = std::chrono::system_clock::now();
            current_data.valid = false;
            return false;
        }
    }

    
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void IMU::setAccelerometerRange(LSM6DS3::FS_XL range)
{
    
    // Read current register value to preserve other bits
    uint8_t current_value = readRegister(lsm6ds3_fd, LSM6DS3::Reg::CTRL1_XL);
    
    // Clear range bits (bits 2-3) and set new range
    uint8_t new_value = (current_value & 0xF3) | static_cast<uint8_t>(range);
    
    writeRegister(lsm6ds3_fd, LSM6DS3::Reg::CTRL1_XL, new_value);
    
    // Update scale factor
    switch (range) 
    {
        case LSM6DS3::FS_XL::RANGE_2_G:
            accel_scale = 0.061f; // mg/LSB
            break;
        case LSM6DS3::FS_XL::RANGE_4_G:
            accel_scale = 0.122f; // mg/LSB
            break;
        case LSM6DS3::FS_XL::RANGE_8_G:
            accel_scale = 0.244f; // mg/LSB
            break;
        case LSM6DS3::FS_XL::RANGE_16_G:
            accel_scale = 0.488f; // mg/LSB
            break;
   
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void IMU::setGyroscopeRange(LSM6DS3::FS_G range)
{
    // Read current register value to preserve other bits
    uint8_t current_value = readRegister(lsm6ds3_fd, LSM6DS3::Reg::CTRL2_G);
    
    if(range != LSM6DS3::FS_G::RANGE_125_DPS)
    {
        // Clear range bits (bits 2-3) and set new range
        setBit(lsm6ds3_fd, LSM6DS3::Reg::CTRL2_G, LSM6DS3::SB_MASK::CTRL2_FS_125, false);
        uint8_t new_value = (current_value & 0xF3) | static_cast<uint8_t>(range);
        writeRegister(lsm6ds3_fd, LSM6DS3::Reg::CTRL2_G, new_value);
    }
    else
    {
        setBit(lsm6ds3_fd, LSM6DS3::Reg::CTRL2_G, LSM6DS3::SB_MASK::CTRL2_FS_125, true);
        std::cerr << " *SPECIAL CASE* Enabled gyroscope full-scale at 125 dps" <<std::endl;
        
    }
    
    // Update scale factor
    switch (range) 
    {
        case LSM6DS3::FS_G::RANGE_125_DPS:
            gyro_scale = 4.375f; // mdps/LSB
            break;
        case LSM6DS3::FS_G::RANGE_245_DPS:
            gyro_scale = 8.75f; // mdps/LSB
            break;
        case LSM6DS3::FS_G::RANGE_500_DPS:
            gyro_scale = 17.5f; // mdps/LSB
            break;
        case LSM6DS3::FS_G::RANGE_1000_DPS:
            gyro_scale = 35.0f; // mdps/LSB
            break;
        case LSM6DS3::FS_G::RANGE_2000_DPS:
            gyro_scale = 70.0f; // mdps/LSB
            break;
    }

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void IMU::setMagnetometerRange(LIS3MDL::FS range)
{
    // Read current register value to preserve other bits
    uint8_t current_value = readRegister(lis3mdl_fd, LIS3MDL::Reg::CTRL_REG2);
    
    // Clear range bits (bits 5-6) and set new range
    uint8_t new_value = (current_value & 0x9F) | static_cast<uint8_t>(range);
    
    writeRegister(lis3mdl_fd, LIS3MDL::Reg::CTRL_REG2, new_value);
    
    // Update scale factor
    switch (range) 
    {
        case LIS3MDL::FS::RANGE_4_GAUSS:
            mag_scale = 6842.0f;        // gauss/LSB
            break;
        case LIS3MDL::FS::RANGE_8_GAUSS:
            mag_scale = 3421.0f;        // gauss/LSB
            break;
        case LIS3MDL::FS::RANGE_12_GAUSS:
            mag_scale = 2281.0f;        // gauss/LSB
            break;
        case LIS3MDL::FS::RANGE_16_GAUSS:
            mag_scale = 1711.0f;        // gauss/LSB
            break;
    }

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void IMU::setAccelerometerRate(LSM6DS3::ODR_XL rate)
{
    // Read current register value to preserve range bits
    uint8_t current_value = readRegister(lsm6ds3_fd, LSM6DS3::Reg::CTRL1_XL);
    
    // Clear rate bits (bits 4-7) and set new rate
    uint8_t new_value = (current_value & 0x0F) | static_cast<uint8_t>(rate);
    
    writeRegister(lsm6ds3_fd, LSM6DS3::Reg::CTRL1_XL, new_value);

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void IMU::setGyroscopeRate(LSM6DS3::ODR_G rate)
{
     // Read current register value to preserve range bits
    uint8_t current_value = readRegister(lsm6ds3_fd, LSM6DS3::Reg::CTRL2_G);
    
    // Clear rate bits (bits 4-7) and set new rate
    uint8_t new_value = (current_value & 0x0F) | static_cast<uint8_t>(rate);
    
    writeRegister(lsm6ds3_fd, LSM6DS3::Reg::CTRL2_G, new_value);

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void IMU::setMagnetometerRate(LIS3MDL::DO rate)
{
    // Read current register value to preserve other bits
    uint8_t current_value = readRegister(lis3mdl_fd, LIS3MDL::Reg::CTRL_REG1);
    
    // Clear rate bits (bits 2-4) and set new rate
    uint8_t new_value = (current_value & 0xE3) | static_cast<uint8_t>(rate);
    // Make sure Fast_ODR is disabled 
    setBit(lis3mdl_fd, LIS3MDL::Reg::CTRL_REG1, LIS3MDL::SB_MASK::CTRL1_FAST_ODR,false);
    
    writeRegister(lis3mdl_fd, LIS3MDL::Reg::CTRL_REG1, new_value);

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void IMU::setMagnetometerPower(LIS3MDL::OM power_mode)
{
    LIS3MDL::OMZ z_mode;

    switch (power_mode) 
    {
        case LIS3MDL::OM::LOW_POWER:
            z_mode = LIS3MDL::OMZ::LOW_POWER;
            break;
        case LIS3MDL::OM::MEDIUM:
            z_mode = LIS3MDL::OMZ::MEDIUM;
            break;
        case LIS3MDL::OM::HIGH:
            z_mode = LIS3MDL::OMZ::HIGH;
            break;
        case LIS3MDL::OM::ULTRA_HIGH:
            z_mode = LIS3MDL::OMZ::ULTRA_HIGH;
            break;
    }

    uint8_t current_REG1 = readRegister(lis3mdl_fd, LIS3MDL::Reg::CTRL_REG1);
    uint8_t current_REG4 = readRegister(lis3mdl_fd, LIS3MDL::Reg::CTRL_REG4);

    uint8_t new_REG1 = (current_REG1 & 0x9F) | static_cast<uint8_t>(power_mode);
    uint8_t new_REG4 = (current_REG4 & 0xF3) | static_cast<uint8_t>(z_mode);

    writeRegister(lis3mdl_fd, LIS3MDL::Reg::CTRL_REG1, new_REG1);
    writeRegister(lis3mdl_fd, LIS3MDL::Reg::CTRL_REG4, new_REG4);

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void IMU::setMagnetometerMode(LIS3MDL::MD op_mode)
{
    // Read current register value to preserve other bit
    uint8_t current_value = readRegister(lis3mdl_fd, LIS3MDL::Reg::CTRL_REG3);
    
    uint8_t new_value = (current_value & 0xFC) | static_cast<uint8_t>(op_mode);

    writeRegister(lis3mdl_fd, LIS3MDL::Reg::CTRL_REG3, new_value);

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////



// Modify your updateOrientation method to use Kalman filter when enabled
void IMU::updateOrientation()
{
    std::lock_guard<std::mutex> lock(data_mutex);
    
    if (!current_data.valid) 
    {
        current_orientation.valid = false;
        current_orientation.timestamp = std::chrono::system_clock::now();
        return;
    }
    
    kalman_filter.update(current_data, magCalibrated);
    current_orientation = kalman_filter.getOrientation();
   
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool IMU::calibrateAccelerometer() 
{
  
    int attempts = 0;
    while (!isStationary()) 
    {
        std::cerr << "Waiting for device to be stationary..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        attempts++;
        if (attempts > 30)
        {
            std::cerr << "Could not calibrate accelerometer, timed out waiting for device to be stationary" << std::endl;
            return false;
        }
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Make a local copy of the deque to reduce mutex lock time
    std::deque<IMUData> calibration_data;
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        calibration_data = data_history; // Copy the deque
    }
    
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    size_t count = 0;
    
    // Use copied data for calculations (no mutex needed)
    for (int i = 0; i < calibration_data.size(); i++) 
    {
        if(calibration_data[i].valid)
        {
            sumX += calibration_data[i].ax;
            sumY += calibration_data[i].ay;
            sumZ += calibration_data[i].az;
            count++;
        }
    }
        
    // Calculate average offsets
    float avgX = (sumX / count);
    float avgY = (sumY / count);
    float avgZ = (sumZ / count);
    float desiredZ = -1; // -1g for Z axis when flat
    
    std::cerr << "Current average readings (g): X=" << avgX << ", Y=" << avgY 
              << ", Z=" << avgZ << std::endl;
    
    // Read current USR_OFF_W setting (weight of the offset)
    uint8_t ctrl6_c = readRegister(lsm6ds3_fd, LSM6DS3::Reg::CTRL6_C);
    bool high_weight = (ctrl6_c & LSM6DS3::SB_MASK::CTRL6_USR_OFF_W) > 0;
    
    // Scale factor for offset registers
    // 2^(-6) g/LSB when USR_OFF_W=1, 2^(-10) g/LSB when USR_OFF_W=0
    float scale_factor = high_weight ? (1.0f / 64.0f) : (1.0f / 1024.0f);
    
    // Calculate offset values in register units (signed 8-bit values)
    int8_t offsetX = -static_cast<int8_t>(avgX / scale_factor);
    int8_t offsetY = -static_cast<int8_t>(avgY / scale_factor);
    int8_t offsetZ = -static_cast<int8_t>((avgZ - desiredZ) / scale_factor);
    
    // Ensure values are within valid range (-127 to +127)
    offsetX = std::max(std::min(offsetX, static_cast<int8_t>(127)), static_cast<int8_t>(-127));
    offsetY = std::max(std::min(offsetY, static_cast<int8_t>(127)), static_cast<int8_t>(-127));
    offsetZ = std::max(std::min(offsetZ, static_cast<int8_t>(127)), static_cast<int8_t>(-127));
    
    std::cerr << "Writing offsets: X=" << static_cast<int>(offsetX) 
              << ", Y=" << static_cast<int>(offsetY) 
              << ", Z=" << static_cast<int>(offsetZ) << std::endl;
    
    // Write offsets to registers
    writeRegister(lsm6ds3_fd, LSM6DS3::Reg::X_OFS_USR, static_cast<uint8_t>(offsetX));
    writeRegister(lsm6ds3_fd, LSM6DS3::Reg::Y_OFS_USR, static_cast<uint8_t>(offsetY));
    writeRegister(lsm6ds3_fd, LSM6DS3::Reg::Z_OFS_USR, static_cast<uint8_t>(offsetZ));
    
    std::cerr << "Accelerometer calibration complete" << std::endl;
    
    accelCalibrated = true;
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool IMU::calibrateMagnetometer()
{
    std::cerr << "Starting magnetometer calibration..." << std::endl;
    std::cerr << "Please rotate the robot slowly around its Z axis (full 360° rotation)" << std::endl;
    
    std::deque<IMUData> calibration_data;

    // Collection variables
    float min_mx = std::numeric_limits<float>::max();
    float max_mx = std::numeric_limits<float>::lowest();
    float min_my = std::numeric_limits<float>::max();
    float max_my = std::numeric_limits<float>::lowest();
    

    float gyro_rotation = 0.0f;
    float mag_rotation = 0.0f;
    bool calibration_started = false;
    float current_mag_angle = 0.0f;
    float mag_starting_angle = 0.0f;
    
    const float gyro_threshold = 0.5f; // dps - adjust based on your observations
    const float mag_threshold = 0.1f; // gauss - adjust based on your observations
    
    
    // Required rotation for calibration
    const float required_rotation = 360.0f; // degrees
    
    // Maximum time for calibration
    const int calibration_timeout = 60000; // 60 seconds timeout
    std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point last_update_time = start_time;
    
    // Minimum number of data points required
    const size_t min_data_points = 100;
    
    // Sampling interval for progress updates
    const int update_interval_ms = 200; // 1 second
    auto last_update = start_time;
    
    std::cerr << "Starting calibration - rotate robot slowly..." << std::endl;
    
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::system_clock::now() - start_time).count() < calibration_timeout) 
    {
        IMUData current;
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            current = current_data;
        }
        if (!current.valid) 
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        // Calculate time delta for gyro integration
        auto current_time = current.timestamp;
        float dt = std::chrono::duration<float>(current_time - last_update_time).count();
        last_update_time = current_time;
        
        // Check if rotation has started (using gyro)
        if (!calibration_started) 
        {
            if (std::abs(current.gz) > gyro_threshold) 
            {
                calibration_started = true;
                std::cerr << "Rotation detected! Beginning calibration..." << std::endl;
                mag_starting_angle = (std::atan2(current.my, current.mx) * 180 / M_PI);
            } 
            else 
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
        }
        
        // Add data to calibration set once rotation has started
        calibration_data.push_back(current);
        

        if (std::abs(current.gz) > gyro_threshold)
        {
            gyro_rotation += std::abs(current.gz) * dt;
        }
        

        float new_mag_angle = (std::atan2(current.my, current.mx) * 180 / M_PI);
        std::cerr << "New agnetometer angle: " <<  new_mag_angle << std::endl;
        if (new_mag_angle < 0)
        {
            new_mag_angle += 360;
        }
        
        float angle_diff = new_mag_angle - current_mag_angle;
        std::cerr << "Magnetometer angle diffrence: " <<  angle_diff << std::endl;

        if((angle_diff > mag_threshold))
        {
            current_mag_angle = new_mag_angle;
            mag_rotation +=  std::abs(angle_diff);

            //Update min/max values for magnetometer data
            min_mx = std::min(min_mx, current.mx);
            max_mx = std::max(max_mx, current.mx);
            min_my = std::min(min_my, current.my);
            max_my = std::max(max_my, current.my);
        }
        
        // Show progress updates at regular intervals
        auto now = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update).count() > update_interval_ms) 
        {
            std::cout << "Rotation progress (gyro): " << gyro_rotation
                      << "° / " << required_rotation << "°" << std::endl;
            std::cout << "Rotation progress (mag): " << (mag_rotation * RAD_TO_DEG) 
                      << "° / " << required_rotation << "°" << std::endl;
            std::cout << "Data points collected: " << calibration_data.size() << std::endl;
            
            // Update timestamp for next progress update
            last_update = now;
        }
        
        // Check if we've completed a full rotation using gyro measurement
        // Also verify we have enough data points
        if (gyro_rotation >= required_rotation && calibration_data.size() >= min_data_points) 
        {
            std::cout << "Full rotation detected! Gyro-based rotation: " << gyro_rotation
                      << "°, Mag-based rotation: " << (mag_rotation * RAD_TO_DEG) << "°" << std::endl;
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    
    // Check if we have enough data
    if (calibration_data.size() < min_data_points) 
    {
        std::cerr << "Insufficient data for calibration. Only " << calibration_data.size() 
                  << " data points collected." << std::endl;
        return false;
    }
    
    // Check if we completed a full rotation
    if (gyro_rotation < required_rotation) 
    {
        std::cerr << "Calibration incomplete. Only " << gyro_rotation 
                  << "° rotation detected (required: " << required_rotation << "°)" << std::endl;
        return false;
    }
    
    // Calculate center offsets (hard iron distortion)
    float offset_x = (min_mx + max_mx) / 2.0f;
    float offset_y = (min_my + max_my) / 2.0f;
    
    // For Z-axis, we use the average of all readings
    float sum_mz = 0.0f;
    int count = 0;
    for (const auto& data : calibration_data) {
        sum_mz += data.mz;
        count++;
    }
    float offset_z = (count > 0) ? (sum_mz / count) : 0.0f;
    
    std::cout << "Magnetometer calibration complete:" << std::endl;
    std::cout << "Hard iron offsets: X=" << offset_x << ", Y=" << offset_y << ", Z=" << offset_z << std::endl;
    
    // Convert offsets to register values
    int16_t raw_offset_x = static_cast<int16_t>(offset_x * mag_scale / 100.0f);
    int16_t raw_offset_y = static_cast<int16_t>(offset_y * mag_scale / 100.0f);
    int16_t raw_offset_z = static_cast<int16_t>(offset_z * mag_scale / 100.0f);
    
    // Write offsets to the magnetometer registers
    writeRegister(lis3mdl_fd, LIS3MDL::Reg::OFFSET_X_REG_L_M, raw_offset_x & 0xFF);
    writeRegister(lis3mdl_fd, LIS3MDL::Reg::OFFSET_X_REG_H_M, (raw_offset_x >> 8) & 0xFF);
    
    writeRegister(lis3mdl_fd, LIS3MDL::Reg::OFFSET_Y_REG_L_M, raw_offset_y & 0xFF);
    writeRegister(lis3mdl_fd, LIS3MDL::Reg::OFFSET_Y_REG_H_M, (raw_offset_y >> 8) & 0xFF);
    
    writeRegister(lis3mdl_fd, LIS3MDL::Reg::OFFSET_Z_REG_L_M, raw_offset_z & 0xFF);
    writeRegister(lis3mdl_fd, LIS3MDL::Reg::OFFSET_Z_REG_H_M, (raw_offset_z >> 8) & 0xFF);
    
    std::cout << "Magnetometer offset registers updated" << std::endl;
    
    // Calculate scaling factors (soft iron distortion)
    float radius_x = (max_mx - min_mx) / 2.0f;
    float radius_y = (max_my - min_my) / 2.0f;
    float avg_radius = (radius_x + radius_y) / 2.0f;
    
    float scale_factor_x = avg_radius / radius_x;
    float scale_factor_y = avg_radius / radius_y;
    
    std::cout << "Soft iron scaling: X=" << scale_factor_x << ", Y=" << scale_factor_y << std::endl;
    
    // Set scaling factors in Kalman filter (since hardware doesn't handle scaling)
    kalman_filter.setMagneticOffsets(0.0f, 0.0f, 0.0f); // Zero since hardware registers handle offsets
    kalman_filter.setMagneticScaling(scale_factor_x, scale_factor_y, 1.0f);
    
    // Calculate and report the fit quality (how close to a circle)
    float circle_error = std::abs(radius_x - radius_y) / avg_radius * 100.0f;
    std::cout << "Circle fit quality: " << (100.0f - circle_error) << "% (closer to 100% is better)" << std::endl;
    
    magCalibrated = true;
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool IMU::isStationary(float threshold) const 
{
    std::deque<IMUData> smaples; 
    std::lock_guard<std::mutex> lock(data_mutex);
    
    // Need a minimum number of samples
    if (data_history.size() < HISTORY_BUFFER_SIZE) 
    {
        std::cerr << "Not enough samples in deque to determine if device is stationary" << std::endl;
        return false;
    }
    else
    {
        smaples = data_history; // Copy the deque
    }

    // Calculate variance of angular velocity
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    float sum_gx2 = 0.0f, sum_gy2 = 0.0f, sum_gz2 = 0.0f;
    
    // Calculate means and squared values
    for (const auto& data : smaples) 
    {
        sum_gx += data.gx;
        sum_gy += data.gy;
        sum_gz += data.gz;
        
        sum_gx2 += data.gx * data.gx;
        sum_gy2 += data.gy * data.gy;
        sum_gz2 += data.gz * data.gz;
    }
    
    float n = static_cast<float>(data_history.size());
    float var_gx = (sum_gx2 / n) - (sum_gx / n) * (sum_gx / n);
    float var_gy = (sum_gy2 / n) - (sum_gy / n) * (sum_gy / n);
    float var_gz = (sum_gz2 / n) - (sum_gz / n) * (sum_gz / n);
    
    // Calculate the standard deviation for each axis
    float std_gx = std::sqrt(std::abs(var_gx));
    float std_gy = std::sqrt(std::abs(var_gy));
    float std_gz = std::sqrt(std::abs(var_gz));
    
    // Maximum standard deviation on any axis
    float max_std = std::max({std_gx, std_gy, std_gz});
    
    // For debugging
    std::cerr << "Movement check - Std Dev (x,y,z): " << std_gx << ", " 
              << std_gy << ", " << std_gz << " (threshold: " << threshold << ")" << std::endl;
    
    return max_std < threshold;
}






























































































//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


IMUData IMU::getSensorData() const 
{
    std::lock_guard<std::mutex> lock(data_mutex);
    return current_data;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


OrientationData IMU::getOrientationData() const 
{
    std::lock_guard<std::mutex> lock(data_mutex);
    return current_orientation;
}

void IMU::setHeading(float yaw_degrees)
{
    kalman_filter.setYaw(yaw_degrees);
}
