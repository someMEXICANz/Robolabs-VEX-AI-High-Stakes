#include "IMU.h"


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
      current_orientation{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, std::chrono::system_clock::now(),false}
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
        std::cerr << "Failed to start GPS read threads: " << e.what() << std::endl;
        running = false;
        return false;  
    }
    
    return true;
}

void IMU::stop() 
{
    running = false;
    
    if (read_thread&& read_thread->joinable()) {
        read_thread->join();
    }
    
    read_thread.reset();
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
    
    setAccelerometerRange(LSM6DS3::FS_XL::RANGE_2_G);
    setGyroscopeRange(LSM6DS3::FS_G::RANGE_125_DPS);

    // Enables First Low Pass Filter
    setBit(lsm6ds3_fd, LSM6DS3::Reg::CTRL1_XL, LSM6DS3::SB_MASK::CTRL1_LPF1_BW_SEL, true);
    // Enables second Low Pass Filter
    setBit(lsm6ds3_fd, LSM6DS3::Reg::CTRL8_XL, LSM6DS3::SB_MASK::CTRL8_LPF2_XL_EN, true);
    // Enables high performance mode for accel
    setBit(lsm6ds3_fd, LSM6DS3::Reg::CTRL6_C, LSM6DS3::SB_MASK::CTRL6_XL_HM_MODE, true);
    // Enables high performance mode for gyro
    setBit(lsm6ds3_fd, LSM6DS3::Reg::CTRL7_G, LSM6DS3::SB_MASK::CTRL7_G_HM_MODE, true);

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
         std::chrono::time_point start_time = std::chrono::steady_clock::now();
        
        if (!initialized) 
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            break;
        }
        
        readData();
        
        // Calculate time to sleep
        std::chrono::duration elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed < read_interval) 
        {
            std::this_thread::sleep_for(read_interval - elapsed);
        }
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
    
    float gx = (float)LSM6DS3_data[1] * gyro_scale * DEG_TO_RAD / 1000;
    float gy = (float)LSM6DS3_data[2] * gyro_scale * DEG_TO_RAD / 1000;
    float gz = (float)LSM6DS3_data[3] * gyro_scale * DEG_TO_RAD / 1000;
    
    float ax = (float)LSM6DS3_data[4] * accel_scale * GRAVITY_STD / 1000;
    float ay = (float)LSM6DS3_data[5] * accel_scale * GRAVITY_STD / 1000;
    float az = (float)LSM6DS3_data[6] * accel_scale * GRAVITY_STD / 1000;


    
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

    float mx = (float)LIS3MDL_data[0] / mag_scale * 100;
    float my = (float)LIS3MDL_data[1] / mag_scale * 100;
    float mz = (float)LIS3MDL_data[2] / mag_scale * 100;

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

        //updateOrientation();

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


bool IMU::isStationary(float threshold) const 
{
    std::lock_guard<std::mutex> lock(data_mutex);
    
    // Need a minimum number of samples
    if (data_history.size() < 10) {
        return false;
    }
    
    // Calculate variance of angular velocity
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    float sum_gx2 = 0.0f, sum_gy2 = 0.0f, sum_gz2 = 0.0f;
    
    // Calculate means and squared values
    for (const auto& data : data_history) {
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



// bool IMU::updateOrientation() 
// {
//     std::lock_guard<std::mutex> lock(data_mutex);
    
//     if (!current_data.valid) {
//         current_orientation.valid = false;
//         current_orientation.timestamp = std::chrono::system_clock::now();
//         return false;
//     }
    
//     // Static variables to track last update time and integrated angles
//     static std::chrono::steady_clock::time_point last_update_time = 
//         std::chrono::steady_clock::now();
//     static float integrated_roll = 0.0f;
//     static float integrated_pitch = 0.0f;
//     static float integrated_yaw = 0.0f;
    
//     // Calculate time delta
//     auto current_time = std::chrono::steady_clock::now();
//     float dt = std::chrono::duration<float>(current_time - last_update_time).count();
//     last_update_time = current_time;
    
//     // Skip if this is the first update or timing is off
//     if (dt > 0.5f || dt <= 0.0f) 
//     {
//         dt = 0.01f; // Default to 10ms if timing seems wrong
//     }
    
//     // Get current sensor data
//     float ax = current_data.ax;
//     float ay = current_data.ay;
//     float az = current_data.az;
//     float gx = current_data.gx;
//     float gy = current_data.gy;
//     float gz = current_data.gz;
//     float mx = current_data.mx;
//     float my = current_data.my;
//     float mz = current_data.mz;
    
//     // Calculate accel-based pitch and roll (in radians)
//     float accel_roll = atan2(ay, az);
//     float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az));
    
//     // Integrate gyro data (in degrees)
//     integrated_roll += gx * dt;
//     integrated_pitch += gy * dt;
//     integrated_yaw += gz * dt;
    
//     // Calculate tilt-compensated magnetometer heading
//     // Apply the tilt compensation to magnetometer readings
//     float tilt_comp_mx = mx * cos(accel_pitch) + mz * sin(accel_pitch);
//     float tilt_comp_my = mx * sin(accel_roll) * sin(accel_pitch) + 
//                          my * cos(accel_roll) - 
//                          mz * sin(accel_roll) * cos(accel_pitch);
    
//     // // Calculate the raw mag heading (0-360 degrees)
//     // float mag_heading = atan2(tilt_comp_my, tilt_comp_mx) * RAD_TO_DEG;
//     // if (mag_heading < 0) mag_heading += 360.0f;
    
//     // // Normalize to 0-360
//     // while (mag_heading < 0) mag_heading += 360.0f;
//     // while (mag_heading >= 360.0f) mag_heading -= 360.0f;
    
//     // Complementary filter to combine gyro and accel/mag data
//     const float alpha = 0.98f;
    
//     // Convert accel values to degrees
//     float roll_deg = accel_roll * RAD_TO_DEG;
//     float pitch_deg = accel_pitch * RAD_TO_DEG;
    
//     // Apply complementary filter for roll and pitch
//     float roll = alpha * integrated_roll + (1.0f - alpha) * roll_deg;
//     float pitch = alpha * integrated_pitch + (1.0f - alpha) * pitch_deg;
    
//     // For yaw/heading, use a complementary filter between gyro and magnetometer
//     // First calculate the difference between mag and gyro
//     float yaw_error = mag_heading - integrated_yaw;
//     // Normalize the error to -180 to +180
//     if (yaw_error > 180.0f) yaw_error -= 360.0f;
//     if (yaw_error < -180.0f) yaw_error += 360.0f;
    
//     // Apply the filter with a smaller alpha for heading (more magnetometer influence)
//     // Gyro provides short-term accuracy, mag provides long-term stability
//     float yaw = integrated_yaw + (1.0f - alpha) * yaw_error;
    
//     // Normalize yaw to 0-360
//     if (yaw < 0) yaw += 360.0f;
//     if (yaw >= 360.0f) yaw -= 360.0f;
    
//     // Save the new integrated values
//     integrated_roll = roll;
//     integrated_pitch = pitch;
//     integrated_yaw = yaw;
    
//     // The absolute heading is the same as the yaw for a robot
//     float heading = yaw;
    
//     // Convert to radians for quaternion calculation
//     float roll_rad = roll * DEG_TO_RAD;
//     float pitch_rad = pitch * DEG_TO_RAD;
//     float yaw_rad = yaw * DEG_TO_RAD;
    
//     // Convert Euler angles to quaternion
//     // Calculate half angles
//     float cr = cos(roll_rad * 0.5f);
//     float sr = sin(roll_rad * 0.5f);
//     float cp = cos(pitch_rad * 0.5f);
//     float sp = sin(pitch_rad * 0.5f);
//     float cy = cos(yaw_rad * 0.5f);
//     float sy = sin(yaw_rad * 0.5f);
    
//     // Quaternion
//     float qw = cr * cp * cy + sr * sp * sy;
//     float qx = sr * cp * cy - cr * sp * sy;
//     float qy = cr * sp * cy + sr * cp * sy;
//     float qz = cr * cp * sy - sr * sp * cy;
    
//     // Update orientation structure
//     current_orientation.roll = roll;
//     current_orientation.pitch = pitch;
//     current_orientation.yaw = yaw;
//     current_orientation.qw = qw;
//     current_orientation.qx = qx;
//     current_orientation.qy = qy;
//     current_orientation.qz = qz;
//     current_orientation.timestamp = std::chrono::system_clock::now();
//     current_orientation.valid = true;
    
//     return true;
// }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


// bool IMU::calibrateSensors() 
// {
//     if (!running || !initialized) 
//     {
//         last_error = "IMU must be running and initialized to calibrate";
//         std::cerr << last_error << std::endl;
//         return false;
//     }
    
//     // Wait for the sensor to be stationary
//     int attempts = 0;
//     while (!isStationary() && attempts < 30) 
//     {
//         std::cerr << "Waiting for device to be stationary..." << std::endl;
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         attempts++;
//     }
    
//     if (attempts >= 30) 
//     {
//         last_error = "Timed out waiting for device to be stationary";
//         std::cerr << last_error << std::endl;
//         return false;
//     }
    
//     // Once stationary, wait a bit longer to collect more stable data
//     std::cerr << "Device is stationary. Collecting stable data for calibration..." << std::endl;
//     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
//     // Make a local copy of the deque to reduce mutex lock time
//     std::deque<IMUData> calibration_data;
//     {
//         std::lock_guard<std::mutex> lock(data_mutex);
//         calibration_data = data_history; // Copy the deque
//     }
    
//     if (calibration_data.size() < 50) 
//     {
//         std::cerr << "Not enough data for calibration, need at least 50 samples" << std::endl;
//         return false;
//     }
    
//     float sumAx = 0.0f, sumAy = 0.0f, sumAz = 0.0f;
//     float sumMx = 0.0f, sumMy = 0.0f, sumMz = 0.0f;
//     size_t count = 0;
    
    
//     // Use copied data for calculations (no mutex needed)
//     for (int i = 0; i < calibration_data.size(); i++) 
//     {
//         sumAx += calibration_data[i].ax;
//         sumAy += calibration_data[i].ay;
//         sumAz += calibration_data[i].az;
//         sumMx += calibration_data[i].mx;
//         sumMy += calibration_data[i].my;
//         sumMz += calibration_data[i].mz;
//         count++;
//     }







    
    
//     float new_offset_x = sumX / count;
//     float new_offset_y = sumY / count;
//     float new_offset_z = (sumZ / count) - 1.0f;  // Remove gravity (1g)
    
//     // Lock mutex again just to update the offsets
//     {
//         std::lock_guard<std::mutex> lock(data_mutex);
//         accel_offset_x = new_offset_x;
//         accel_offset_y = new_offset_y;
//         accel_offset_z = new_offset_z;
//     }
    
//     std::cerr << "Accelerometer calibration: offsets = ("
//               << accel_offset_x << ", " << accel_offset_y << ", "
//               << accel_offset_z << ") using " << count << " samples" << std::endl;
    
//     return true;
// }


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

