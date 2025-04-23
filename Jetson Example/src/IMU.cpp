#include "IMU.h"


IMU::IMU(const std::string& i2cBus_)
    : i2c_bus(i2cBus_),
      i2c_fd(-1),
      accel_offset_x(0.0f), accel_offset_y(0.0f), accel_offset_z(0.0f),
      gyro_offset_x(0.0f), gyro_offset_y(0.0f), gyro_offset_z(0.0f),
      mag_offset_x(0.0f), mag_offset_y(0.0f), mag_offset_z(0.0f),
      heading_offset(0.0f),
      accel_scale(0.0f), 
      gyro_scale(0.0f),  
      mag_scale(0.0f),    
      running(false),
      connected(false),
      current_data{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f, std::chrono::system_clock::now(),false},
      current_orientation{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, std::chrono::system_clock::now(),false}
{
    if (!i2c_bus.empty()) {
        if(initialize()) {
            // Configure with default settings
            configureRanges(ACCEL_RAMGE::ACCEL_RANGE_4_G, 
                           GYRO_RANGE::GYRO_RANGE_2000_DPS, 
                           MAG_RANGE::LIS3MDL_RANGE_4_GAUSS);
            
            configureLSM6DS(LSM6DS_DATA_RATE::RATE_104_HZ,
                           LSM6DS_HPF_RAMGE::HPF_ODR_DIV_100);
            
            configureLIS3MDL(LIS3MDL_DATA_RATE::RATE_155_HZ,
                            LIS3MDL_PERF_MODE::LIS3MDL_ULTRAHIGHMODE,
                            LIS3MDL_OPER_MODE::LIS3MDL_CONTINUOUSMODE);
            
            start();
        }
        
        // Wait a moment to collect some data before calibrating
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        calibrateGyroscope();
        calibrateAccelerometer();
        calibrateMagnetometer();
    }
}

IMU::~IMU() 
{
    stop();
    
    if (i2c_fd>= 0) {
        close(i2c_fd);
        i2c_fd= -1;
    }
}



bool IMU::start() 
{
    if (running) return true;
    
    if (i2c_fd < 0 && !reconnect()) {
        return false;
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

void IMU::stop() {
    running = false;
    
    if (read_thread&& read_thread->joinable()) {
        read_thread->join();
    }
    
    read_thread.reset();
}

bool IMU::restart() {
    stop();
    return start();
}


bool IMU::reconnect() 
{
    if (i2c_fd>= 0) {
        close(i2c_fd);
        i2c_fd= -1;
    }
    return initialize();
}

// Main reading loop that runs in a separate thread
void IMU::readLoop() {
    std::cerr << "IMU read loop started" << std::endl;
    
    const std::chrono::milliseconds read_interval(10); // 100Hz update rate
    
    while (running) 
    {
         std::chrono::time_point start_time = std::chrono::steady_clock::now();
        
        if (!connected && !reconnect()) 
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        
        readData();
        
        // Calculate time to sleep
        std::chrono::duration elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed < read_interval) 
        {
            std::this_thread::sleep_for(read_interval - elapsed);
        }
    }
    
    std::cerr << "IMU read loop stopped" << std::endl;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

 
void IMU::configureRanges(ACCEL_RAMGE a_range, GYRO_RANGE g_range, MAG_RANGE m_range) {
    // Configure accelerometer range
    uint8_t accel_reg_value = 0;
    switch (a_range) {
        case ACCEL_RAMGE::ACCEL_RANGE_2_G:
            accel_scale = 0.061f; // mg/LSB
            accel_reg_value = 0x00;
            break;
        case ACCEL_RAMGE::ACCEL_RANGE_4_G:
            accel_scale = 0.122f; // mg/LSB
            accel_reg_value = 0x08;
            break;
        case ACCEL_RAMGE::ACCEL_RANGE_8_G:
            accel_scale = 0.244f; // mg/LSB
            accel_reg_value = 0x0C;
            break;
        case ACCEL_RAMGE::ACCEL_RANGE_16_G:
            accel_scale = 0.488f; // mg/LSB
            accel_reg_value = 0x04;
            break;
    }
    
    // Read current value to preserve data rate bits
    uint8_t current_ctrl1 = 0;
    readBytes(LSM6DS3_ADDR, LSM6DS_CTRL1_XL, &current_ctrl1, 1);
    // Clear range bits and set new range
    current_ctrl1 = (current_ctrl1 & 0xF3) | accel_reg_value;
    writeByte(LSM6DS3_ADDR, LSM6DS_CTRL1_XL, current_ctrl1);
    
    // Configure gyroscope range
    uint8_t gyro_reg_value = 0;
    switch (g_range) {
        case GYRO_RANGE::GYRO_RANGE_125_DPS:
            gyro_scale = 4.375f; // mdps/LSB
            gyro_reg_value = 0x02;
            break;
        case GYRO_RANGE::GYRO_RANGE_250_DPS:
            gyro_scale = 8.75f; // mdps/LSB
            gyro_reg_value = 0x00;
            break;
        case GYRO_RANGE::GYRO_RANGE_500_DPS:
            gyro_scale = 17.5f; // mdps/LSB
            gyro_reg_value = 0x04;
            break;
        case GYRO_RANGE::GYRO_RANGE_1000_DPS:
            gyro_scale = 35.0f; // mdps/LSB
            gyro_reg_value = 0x08;
            break;
        case GYRO_RANGE::GYRO_RANGE_2000_DPS:
            gyro_scale = 70.0f; // mdps/LSB
            gyro_reg_value = 0x0C;
            break;
    }
    
    // Read current value to preserve data rate bits
    uint8_t current_ctrl2 = 0;
    readBytes(LSM6DS3_ADDR, LSM6DS_CTRL2_G, &current_ctrl2, 1);
    // Clear range bits and set new range
    current_ctrl2 = (current_ctrl2 & 0xF0) | gyro_reg_value;
    writeByte(LSM6DS3_ADDR, LSM6DS_CTRL2_G, current_ctrl2);
    
    // Configure magnetometer range
    uint8_t mag_reg_value = 0;
    switch (m_range) {
        case MAG_RANGE::LIS3MDL_RANGE_4_GAUSS:
            mag_scale = 1.0f / 6842.0f; // gauss/LSB (from Adafruit driver)
            mag_reg_value = 0x00;
            break;
        case MAG_RANGE::LIS3MDL_RANGE_8_GAUSS:
            mag_scale = 1.0f / 3421.0f; // gauss/LSB
            mag_reg_value = 0x20;
            break;
        case MAG_RANGE::LIS3MDL_RANGE_12_GAUSS:
            mag_scale = 1.0f / 2281.0f; // gauss/LSB
            mag_reg_value = 0x40;
            break;
        case MAG_RANGE::LIS3MDL_RANGE_16_GAUSS:
            mag_scale = 1.0f / 1711.0f; // gauss/LSB
            mag_reg_value = 0x60;
            break;
    }
    
    writeByte(LIS3MDL_ADDR, LIS3MDL_REG_CTRL_REG2, mag_reg_value);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void IMU::configureLSM6DS(LSM6DS_DATA_RATE data_rate, LSM6DS_HPF_RAMGE hpf_range) {
    // Configure data rate for accelerometer and gyroscope
    uint8_t rate_bits = 0;
    switch (data_rate) {
        case LSM6DS_DATA_RATE::RATE_SHUTDOWN:
            rate_bits = 0x00;
            break;
        case LSM6DS_DATA_RATE::RATE_12_5_HZ:
            rate_bits = 0x10;
            break;
        case LSM6DS_DATA_RATE::RATE_26_HZ:
            rate_bits = 0x20;
            break;
        case LSM6DS_DATA_RATE::RATE_52_HZ:
            rate_bits = 0x30;
            break;
        case LSM6DS_DATA_RATE::RATE_104_HZ:
            rate_bits = 0x40;
            break;
        case LSM6DS_DATA_RATE::RATE_208_HZ:
            rate_bits = 0x50;
            break;
        case LSM6DS_DATA_RATE::RATE_416_HZ:
            rate_bits = 0x60;
            break;
        case LSM6DS_DATA_RATE::RATE_833_HZ:
            rate_bits = 0x70;
            break;
        case LSM6DS_DATA_RATE::RATE_1_66K_HZ:
            rate_bits = 0x80;
            break;
        case LSM6DS_DATA_RATE::RATE_3_33K_HZ:
            rate_bits = 0x90;
            break;
        case LSM6DS_DATA_RATE::RATE_6_66K_HZ:
            rate_bits = 0xA0;
            break;
    }
    
    // Set accelerometer data rate (preserve range bits)
    uint8_t current_ctrl1 = 0;
    readBytes(LSM6DS3_ADDR, LSM6DS_CTRL1_XL, &current_ctrl1, 1);
    current_ctrl1 = (current_ctrl1 & 0x0F) | rate_bits;
    writeByte(LSM6DS3_ADDR, LSM6DS_CTRL1_XL, current_ctrl1);
    
    // Set gyroscope data rate (preserve range bits)
    uint8_t current_ctrl2 = 0;
    readBytes(LSM6DS3_ADDR, LSM6DS_CTRL2_G, &current_ctrl2, 1);
    current_ctrl2 = (current_ctrl2 & 0x0F) | rate_bits;
    writeByte(LSM6DS3_ADDR, LSM6DS_CTRL2_G, current_ctrl2);
    
    // Configure high-pass filter for accelerometer
    uint8_t hpf_bits = 0;
    switch (hpf_range) {
        case LSM6DS_HPF_RAMGE::HPF_ODR_DIV_50:
            hpf_bits = 0x00;
            break;
        case LSM6DS_HPF_RAMGE::HPF_ODR_DIV_100:
            hpf_bits = 0x20;
            break;
        case LSM6DS_HPF_RAMGE::HPF_ODR_DIV_9:
            hpf_bits = 0x40;
            break;
        case LSM6DS_HPF_RAMGE::HPF_ODR_DIV_400:
            hpf_bits = 0x60;
            break;
    }
    
    // Enable high-pass filter
    writeByte(LSM6DS3_ADDR, LSM6DS_CTRL8_XL, 0x04 | hpf_bits);
    
    // Set Block Data Update bit to prevent register corruption during reading
    writeByte(LSM6DS3_ADDR, LSM6DS_CTRL3_C, 0x40);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void IMU::configureLIS3MDL(LIS3MDL_DATA_RATE data_rate, 
                           LIS3MDL_PERF_MODE p_mode,
                           LIS3MDL_OPER_MODE op_mode) {
    // Configure performance mode and data rate
    uint8_t ctrl_reg1_value = 0;
    uint8_t fast_odr = 0;
    
    // Set performance mode bits
    switch (p_mode) {
        case LIS3MDL_PERF_MODE::LIS3MDL_LOWPOWERMODE:
            ctrl_reg1_value = 0x00;
            break;
        case LIS3MDL_PERF_MODE::LIS3MDL_MEDIUMMODE:
            ctrl_reg1_value = 0x20;
            break;
        case LIS3MDL_PERF_MODE::LIS3MDL_HIGHMODE:
            ctrl_reg1_value = 0x40;
            break;
        case LIS3MDL_PERF_MODE::LIS3MDL_ULTRAHIGHMODE:
            ctrl_reg1_value = 0x60;
            break;
    }
    
    // Set data rate bits
    switch (data_rate) {
        case LIS3MDL_DATA_RATE::RATE_0_625_HZ:
            ctrl_reg1_value |= 0x00;
            break;
        case LIS3MDL_DATA_RATE::RATE_1_25_HZ:
            ctrl_reg1_value |= 0x04;
            break;
        case LIS3MDL_DATA_RATE::RATE_2_5_HZ:
            ctrl_reg1_value |= 0x08;
            break;
        case LIS3MDL_DATA_RATE::RATE_5_HZ:
            ctrl_reg1_value |= 0x0C;
            break;
        case LIS3MDL_DATA_RATE::RATE_10_HZ:
            ctrl_reg1_value |= 0x10;
            break;
        case LIS3MDL_DATA_RATE::RATE_20_HZ:
            ctrl_reg1_value |= 0x14;
            break;
        case LIS3MDL_DATA_RATE::RATE_40_HZ:
            ctrl_reg1_value |= 0x18;
            break;
        case LIS3MDL_DATA_RATE::RATE_80_HZ:
            ctrl_reg1_value |= 0x1C;
            break;
        case LIS3MDL_DATA_RATE::RATE_155_HZ:
            ctrl_reg1_value |= 0x02; // FAST_ODR = 1, DO = 00
            fast_odr = 1;
            break;
        case LIS3MDL_DATA_RATE::RATE_300_HZ:
            ctrl_reg1_value |= 0x02; // FAST_ODR = 1, DO = 00
            fast_odr = 1;
            break;
        case LIS3MDL_DATA_RATE::RATE_560_HZ:
            ctrl_reg1_value |= 0x02; // FAST_ODR = 1, DO = 00
            fast_odr = 1;
            break;
        case LIS3MDL_DATA_RATE::RATE_1000_HZ:
            ctrl_reg1_value |= 0x02; // FAST_ODR = 1, DO = 00
            fast_odr = 1;
            break;
    }
    
    // Set FAST_ODR bit
    if (fast_odr) {
        ctrl_reg1_value |= 0x02;
    }
    
    // Enable temperature sensor
    ctrl_reg1_value |= 0x80;
    
    // Write to CTRL_REG1
    writeByte(LIS3MDL_ADDR, LIS3MDL_REG_CTRL_REG1, ctrl_reg1_value);
    
    // Set operation mode
    uint8_t ctrl_reg3_value = 0;
    switch (op_mode) {
        case LIS3MDL_OPER_MODE::LIS3MDL_CONTINUOUSMODE:
            ctrl_reg3_value = 0x00;
            break;
        case LIS3MDL_OPER_MODE::LIS3MDL_SINGLEMODE:
            ctrl_reg3_value = 0x01;
            break;
        case LIS3MDL_OPER_MODE::LIS3MDL_POWERDOWNMODE:
            ctrl_reg3_value = 0x03;
            break;
    }
    
    // Write to CTRL_REG3
    writeByte(LIS3MDL_ADDR, LIS3MDL_REG_CTRL_REG3, ctrl_reg3_value);
    
    // Configure performance mode for Z axis (should match X and Y axes)
    uint8_t ctrl_reg4_value = 0;
    switch (p_mode) {
        case LIS3MDL_PERF_MODE::LIS3MDL_LOWPOWERMODE:
            ctrl_reg4_value = 0x00;
            break;
        case LIS3MDL_PERF_MODE::LIS3MDL_MEDIUMMODE:
            ctrl_reg4_value = 0x04;
            break;
        case LIS3MDL_PERF_MODE::LIS3MDL_HIGHMODE:
            ctrl_reg4_value = 0x08;
            break;
        case LIS3MDL_PERF_MODE::LIS3MDL_ULTRAHIGHMODE:
            ctrl_reg4_value = 0x0C;
            break;
    }
    
    // Write to CTRL_REG4
    writeByte(LIS3MDL_ADDR, LIS3MDL_REG_CTRL_REG4, ctrl_reg4_value);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool IMU::initialize() {
    // Open I2C device
    i2c_fd = open(i2c_bus.c_str(), O_RDWR);
    if (i2c_fd < 0) {
        last_error = "Failed to open I2C bus: " + i2c_bus;
        std::cerr << last_error << std::endl;
        return false;
    }

    // Check LSM6DS3 WHO_AM_I register (should be 0x6A)
    uint8_t whoami = 0;
    if (!readBytes(LSM6DS3_ADDR, LSM6DS_WHOAMI, &whoami, 1) || whoami != 0x6A) {
        last_error = "Failed to identify LSM6DS3 sensor";
        std::cerr << last_error << " (WHO_AM_I=" << static_cast<int>(whoami) << ")" << std::endl;
        return false;
    }

    // Check LIS3MDL WHO_AM_I register (should be 0x3D)
    if (!readBytes(LIS3MDL_ADDR, LIS3MDL_REG_WHO_AM_I, &whoami, 1) || whoami != 0x3D) {
        last_error = "Failed to identify LIS3MDL sensor";
        std::cerr << last_error << " (WHO_AM_I=" << static_cast<int>(whoami) << ")" << std::endl;
        return false;
    }

    // Reset sensors to ensure clean state
    // Reset LSM6DS3
    if (!writeByte(LSM6DS3_ADDR, LSM6DS_CTRL3_C, 0x01)) {
        last_error = "Failed to reset LSM6DS3";
        std::cerr << last_error << std::endl;
        return false;
    }
    
    // Wait for reset to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Reset LIS3MDL
    if (!writeByte(LIS3MDL_ADDR, LIS3MDL_REG_CTRL_REG2, 0x04)) {
        last_error = "Failed to reset LIS3MDL";
        std::cerr << last_error << std::endl;
        return false;
    }
    
    // Wait for reset to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    connected = true;
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool IMU::readData() 
{
    if (i2c_fd < 0) {
        return false;
    }

    bool error = false;

    // Check LSM6DS3 status register to see if data is ready
    uint8_t status = 0;
    if (!readBytes(LSM6DS3_ADDR, LSM6DS_STATUS_REG, &status, 1)) {
        std::cerr << "Failed to read LSM6DS3 status register" << std::endl;
        error = true;
    }
    
    // Only proceed with LSM6DS3 reading if data is ready
    bool accel_ready = (status & 0x01) != 0;
    bool gyro_ready = (status & 0x02) != 0;
    
    // Read accelerometer if data is ready
    uint8_t accel_buffer[6] = {0};
    if (accel_ready && !readBytes(LSM6DS3_ADDR, LSM6DS_OUTX_L_A, accel_buffer, 6)) {
        std::cerr << "Failed to read accelerometer data" << std::endl;
        error = true;
    }
    
    // Read gyroscope if data is ready
    uint8_t gyro_buffer[6] = {0};
    if (gyro_ready && !readBytes(LSM6DS3_ADDR, LSM6DS_OUTX_L_G, gyro_buffer, 6)) {
        std::cerr << "Failed to read gyroscope data" << std::endl;
        error = true;
    }
    
    // Read temperature
    uint8_t temp_buffer[2] = {0};
    if (!readBytes(LSM6DS3_ADDR, LSM6DS_OUT_TEMP_L, temp_buffer, 2)) {
        std::cerr << "Failed to read temperature data" << std::endl;
        error = true;
    }
    
    // Check LIS3MDL status register
    if (!readBytes(LIS3MDL_ADDR, LIS3MDL_REG_STATUS, &status, 1)) {
        std::cerr << "Failed to read magnetometer status register" << std::endl;
        error = true;
    }
    
    // Read magnetometer if data is ready
    uint8_t mag_buffer[6] = {0};
    bool mag_ready = (status & 0x08) != 0;
    if (mag_ready && !readBytes(LIS3MDL_ADDR, LIS3MDL_REG_OUT_X_L, mag_buffer, 6)) {
        std::cerr << "Failed to read magnetometer data" << std::endl;
        error = true;
    }

    if (error) {
        std::lock_guard<std::mutex> lock(data_mutex);
        current_data.timestamp = std::chrono::system_clock::now();
        current_data.valid = false;
        return false;
    } else {
        // Process all readings
        int16_t raw_ax = 0, raw_ay = 0, raw_az = 0;
        int16_t raw_gx = 0, raw_gy = 0, raw_gz = 0;
        int16_t raw_mx = 0, raw_my = 0, raw_mz = 0;
        int16_t raw_temp = 0;
        
        // Only update accel values if new data was available
        if (accel_ready) {
            raw_ax = (accel_buffer[1] << 8) | accel_buffer[0];
            raw_ay = (accel_buffer[3] << 8) | accel_buffer[2];
            raw_az = (accel_buffer[5] << 8) | accel_buffer[4];
        } else {
            // Use last values from current_data (already scaled)
            std::lock_guard<std::mutex> lock(data_mutex);
            raw_ax = current_data.ax / accel_scale + accel_offset_x;
            raw_ay = current_data.ay / accel_scale + accel_offset_y;
            raw_az = current_data.az / accel_scale + accel_offset_z;
        }
        
        // Only update gyro values if new data was available
        if (gyro_ready) {
            raw_gx = (gyro_buffer[1] << 8) | gyro_buffer[0];
            raw_gy = (gyro_buffer[3] << 8) | gyro_buffer[2];
            raw_gz = (gyro_buffer[5] << 8) | gyro_buffer[4];
        } else {
            // Use last values from current_data
            std::lock_guard<std::mutex> lock(data_mutex);
            raw_gx = current_data.gx / gyro_scale + gyro_offset_x;
            raw_gy = current_data.gy / gyro_scale + gyro_offset_y;
            raw_gz = current_data.gz / gyro_scale + gyro_offset_z;
        }
        
        // Only update magnetometer values if new data was available
        if (mag_ready) {
            raw_mx = (mag_buffer[1] << 8) | mag_buffer[0];
            raw_my = (mag_buffer[3] << 8) | mag_buffer[2];
            raw_mz = (mag_buffer[5] << 8) | mag_buffer[4];
        } else {
            // Use last values from current_data
            std::lock_guard<std::mutex> lock(data_mutex);
            raw_mx = current_data.mx / mag_scale + mag_offset_x;
            raw_my = current_data.my / mag_scale + mag_offset_y;
            raw_mz = current_data.mz / mag_scale + mag_offset_z;
        }
        
        raw_temp = (temp_buffer[1] << 8) | temp_buffer[0];
        
        // Apply scaling and offsets
        float ax = (raw_ax * accel_scale) - accel_offset_x;
        float ay = (raw_ay * accel_scale) - accel_offset_y;
        float az = (raw_az * accel_scale) - accel_offset_z;
        
        // Convert gyro data from mdps to dps
        float gx = ((raw_gx * gyro_scale) - gyro_offset_x) / 1000.0f;
        float gy = ((raw_gy * gyro_scale) - gyro_offset_y) / 1000.0f;
        float gz = ((raw_gz * gyro_scale) - gyro_offset_z) / 1000.0f;
        
        float mx = (raw_mx * mag_scale) - mag_offset_x;
        float my = (raw_my * mag_scale) - mag_offset_y;
        float mz = (raw_mz * mag_scale) - mag_offset_z;
        
        // Temperature calculation from Adafruit driver
        float temp = static_cast<float>(raw_temp) / 16.0f + 25.0f;
        
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
            if (data_history.size() > HISTORY_BUFFER_SIZE) {
                data_history.pop_front();  // Remove the oldest element
            }
        }
        updateOrientation();
    }
    
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool IMU::updateOrientation() 
{
    std::lock_guard<std::mutex> lock(data_mutex);
    
    if (!current_data.valid) {
        current_orientation.valid = false;
        current_orientation.timestamp = std::chrono::system_clock::now();
        return false;
    }
    
    // Static variables to track last update time and integrated angles
    static std::chrono::steady_clock::time_point last_update_time = 
        std::chrono::steady_clock::now();
    static float integrated_roll = 0.0f;
    static float integrated_pitch = 0.0f;
    static float integrated_yaw = 0.0f;
    
    // Calculate time delta
    auto current_time = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(current_time - last_update_time).count();
    last_update_time = current_time;
    
    // Skip if this is the first update or timing is off
    if (dt > 0.5f || dt <= 0.0f) {
        dt = 0.01f; // Default to 10ms if timing seems wrong
    }
    
    // Get current sensor data
    float ax = current_data.ax;
    float ay = current_data.ay;
    float az = current_data.az;
    float gx = current_data.gx;
    float gy = current_data.gy;
    float gz = current_data.gz;
    float mx = current_data.mx;
    float my = current_data.my;
    float mz = current_data.mz;
    
    // Calculate accel-based pitch and roll (in radians)
    float accel_roll = atan2(ay, az);
    float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az));
    
    // Integrate gyro data (in degrees)
    integrated_roll += gx * dt;
    integrated_pitch += gy * dt;
    integrated_yaw += gz * dt;
    
    // Calculate tilt-compensated magnetometer heading
    // Apply the tilt compensation to magnetometer readings
    float tilt_comp_mx = mx * cos(accel_pitch) + mz * sin(accel_pitch);
    float tilt_comp_my = mx * sin(accel_roll) * sin(accel_pitch) + 
                         my * cos(accel_roll) - 
                         mz * sin(accel_roll) * cos(accel_pitch);
    
    // Calculate the raw mag heading (0-360 degrees)
    float mag_heading = atan2(tilt_comp_my, tilt_comp_mx) * RAD_TO_DEG;
    if (mag_heading < 0) mag_heading += 360.0f;
    
    // Apply heading offset for calibration
    mag_heading += heading_offset;
    // Normalize to 0-360
    while (mag_heading < 0) mag_heading += 360.0f;
    while (mag_heading >= 360.0f) mag_heading -= 360.0f;
    
    // Complementary filter to combine gyro and accel/mag data
    const float alpha = 0.98f;
    
    // Convert accel values to degrees
    float roll_deg = accel_roll * RAD_TO_DEG;
    float pitch_deg = accel_pitch * RAD_TO_DEG;
    
    // Apply complementary filter for roll and pitch
    float roll = alpha * integrated_roll + (1.0f - alpha) * roll_deg;
    float pitch = alpha * integrated_pitch + (1.0f - alpha) * pitch_deg;
    
    // For yaw/heading, use a complementary filter between gyro and magnetometer
    // First calculate the difference between mag and gyro
    float yaw_error = mag_heading - integrated_yaw;
    // Normalize the error to -180 to +180
    if (yaw_error > 180.0f) yaw_error -= 360.0f;
    if (yaw_error < -180.0f) yaw_error += 360.0f;
    
    // Apply the filter with a smaller alpha for heading (more magnetometer influence)
    // Gyro provides short-term accuracy, mag provides long-term stability
    float yaw = integrated_yaw + (1.0f - alpha) * yaw_error;
    
    // Normalize yaw to 0-360
    if (yaw < 0) yaw += 360.0f;
    if (yaw >= 360.0f) yaw -= 360.0f;
    
    // Save the new integrated values
    integrated_roll = roll;
    integrated_pitch = pitch;
    integrated_yaw = yaw;
    
    // The absolute heading is the same as the yaw for a robot
    float heading = yaw;
    
    // Convert to radians for quaternion calculation
    float roll_rad = roll * DEG_TO_RAD;
    float pitch_rad = pitch * DEG_TO_RAD;
    float yaw_rad = yaw * DEG_TO_RAD;
    
    // Convert Euler angles to quaternion
    // Calculate half angles
    float cr = cos(roll_rad * 0.5f);
    float sr = sin(roll_rad * 0.5f);
    float cp = cos(pitch_rad * 0.5f);
    float sp = sin(pitch_rad * 0.5f);
    float cy = cos(yaw_rad * 0.5f);
    float sy = sin(yaw_rad * 0.5f);
    
    // Quaternion
    float qw = cr * cp * cy + sr * sp * sy;
    float qx = sr * cp * cy - cr * sp * sy;
    float qy = cr * sp * cy + sr * cp * sy;
    float qz = cr * cp * sy - sr * sp * cy;
    
    // Update orientation structure
    current_orientation.roll = roll;
    current_orientation.pitch = pitch;
    current_orientation.yaw = yaw;
    current_orientation.qw = qw;
    current_orientation.qx = qx;
    current_orientation.qy = qy;
    current_orientation.qz = qz;
    current_orientation.heading = heading;
    current_orientation.timestamp = std::chrono::system_clock::now();
    current_orientation.valid = true;
    
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void IMU::setHeading(float abs_angle) {
    std::lock_guard<std::mutex> lock(data_mutex);
    
    if (!current_orientation.valid) {
        return;
    }
    
    // Normalize input angle to 0-360
    while (abs_angle < 0.0f) abs_angle += 360.0f;
    while (abs_angle >= 360.0f) abs_angle -= 360.0f;
    
    // Calculate the difference between desired heading and current raw heading
    float current_mag_heading = current_orientation.heading;
    float offset_change = abs_angle - current_mag_heading;
    
    // Normalize the change to -180 to +180 for shortest path
    if (offset_change > 180.0f) offset_change -= 360.0f;
    if (offset_change < -180.0f) offset_change += 360.0f;
    
    // Update the heading offset
    heading_offset += offset_change;
    
    // Normalize the offset
    while (heading_offset < 0.0f) heading_offset += 360.0f;
    while (heading_offset >= 360.0f) heading_offset -= 360.0f;
    
    std::cerr << "Updated heading offset to " << heading_offset << " degrees" << std::endl;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool IMU::isStationary(float threshold) const {
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
    
    // Total variance
    float total_variance = var_gx + var_gy + var_gz;
    
    return total_variance < threshold;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool IMU::calibrateGyroscope() 
{
    if (!running || !connected) {
        last_error = "IMU must be running and connected to calibrate";
        std::cerr << last_error << std::endl;
        return false;
    }
    
    // Wait for the sensor to be stationary
    int attempts = 0;
    while (!isStationary() && attempts < 30) {
        std::cerr << "Waiting for device to be stationary..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        attempts++;
    }
    
    if (attempts >= 30) {
        last_error = "Timed out waiting for device to be stationary";
        std::cerr << last_error << std::endl;
        return false;
    }
    
    // Once stationary, wait a bit longer to collect more stable data
    std::cerr << "Device is stationary. Collecting stable data for calibration..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Make a local copy of the deque to reduce mutex lock time
    std::deque<IMUData> calibration_data;
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        calibration_data = data_history; // Copy the deque
    }
    
    if (calibration_data.size() < 50) {
        last_error = "Not enough data for calibration, need at least 50 samples";
        std::cerr << last_error << std::endl;
        return false;
    }
    
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    size_t count = 0;
    
    // Use copied data for calculations (no mutex needed)
    for (const auto& data : calibration_data) {
        sumX += data.gx;
        sumY += data.gy;
        sumZ += data.gz;
        count++;
    }
    
    float new_offset_x = sumX / count;
    float new_offset_y = sumY / count;
    float new_offset_z = sumZ / count;
    
    // Lock mutex again just to update the offsets
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        gyro_offset_x = new_offset_x;
        gyro_offset_y = new_offset_y;
        gyro_offset_z = new_offset_z;
    }
    
    std::cerr << "Gyroscope calibration: offsets = ("
              << gyro_offset_x << ", " << gyro_offset_y << ", "
              << gyro_offset_z << ") using " << count << " samples" << std::endl;
    
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool IMU::calibrateAccelerometer() {
    if (!running || !connected) {
        last_error = "IMU must be running and connected to calibrate";
        std::cerr << last_error << std::endl;
        return false;
    }
    
    // Wait for the sensor to be stationary
    int attempts = 0;
    while (!isStationary() && attempts < 30) {
        std::cerr << "Waiting for device to be stationary..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        attempts++;
    }
    
    if (attempts >= 30) {
        last_error = "Timed out waiting for device to be stationary";
        std::cerr << last_error << std::endl;
        return false;
    }
    
    // Once stationary, wait a bit longer to collect more stable data
    std::cerr << "Device is stationary. Collecting stable data for calibration..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Make a local copy of the deque to reduce mutex lock time
    std::deque<IMUData> calibration_data;
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        calibration_data = data_history; // Copy the deque
    }
    
    if (calibration_data.size() < 50) {
        last_error = "Not enough data for calibration, need at least 50 samples";
        std::cerr << last_error << std::endl;
        return false;
    }
    
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    size_t count = 0;
    
    // Use copied data for calculations (no mutex needed)
    for (const auto& data : calibration_data) {
        sumX += data.ax;
        sumY += data.ay;
        sumZ += data.az;
        count++;
    }
    
    float new_offset_x = sumX / count;
    float new_offset_y = sumY / count;
    float new_offset_z = (sumZ / count) - 1.0f;  // Remove gravity (1g)
    
    // Lock mutex again just to update the offsets
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        accel_offset_x = new_offset_x;
        accel_offset_y = new_offset_y;
        accel_offset_z = new_offset_z;
    }
    
    std::cerr << "Accelerometer calibration: offsets = ("
              << accel_offset_x << ", " << accel_offset_y << ", "
              << accel_offset_z << ") using " << count << " samples" << std::endl;
    
    return true;
}

bool IMU::calibrateMagnetometer() {
    if (!running || !connected) {
        last_error = "IMU must be running and connected to calibrate";
        std::cerr << last_error << std::endl;
        return false;
    }
    
    // For proper magnetometer calibration, the device should be rotated in figure-8 patterns
    // to collect samples across all orientations. Here we'll do a simpler calibration.
    
    // Make a local copy of the deque to reduce mutex lock time
    std::deque<IMUData> calibration_data;
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        calibration_data = data_history; // Copy the deque
    }
    
    if (calibration_data.size() < 50) {
        last_error = "Not enough data for calibration, need at least 50 samples";
        std::cerr << last_error << std::endl;
        return false;
    }
    
    // Find min and max values for each axis to determine the center of the sphere
    float min_x = FLT_MAX, max_x = -FLT_MAX;
    float min_y = FLT_MAX, max_y = -FLT_MAX;
    float min_z = FLT_MAX, max_z = -FLT_MAX;
    
    for (const auto& data : calibration_data) {
        min_x = std::min(min_x, data.mx);
        max_x = std::max(max_x, data.mx);
        min_y = std::min(min_y, data.my);
        max_y = std::max(max_y, data.my);
        min_z = std::min(min_z, data.mz);
        max_z = std::max(max_z, data.mz);
    }
    
    // Calculate offsets (center of the sphere)
    float new_offset_x = (min_x + max_x) / 2.0f;
    float new_offset_y = (min_y + max_y) / 2.0f;
    float new_offset_z = (min_z + max_z) / 2.0f;
    
    // Lock mutex again just to update the offsets
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        mag_offset_x = new_offset_x;
        mag_offset_y = new_offset_y;
        mag_offset_z = new_offset_z;
    }
    
    std::cerr << "Magnetometer calibration: offsets = ("
              << mag_offset_x << ", " << mag_offset_y << ", "
              << mag_offset_z << ") using " << calibration_data.size() << " samples" << std::endl;
    
    return true;
}

    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool IMU::writeByte(uint8_t devAddr, uint8_t reg, uint8_t data) 
{
    if (i2c_fd < 0) {
        last_error = "I2C device not open";
        std::cerr << last_error << std::endl;
        return false;
    }
    
    if (ioctl(i2c_fd, I2C_SLAVE, devAddr) < 0) {
        last_error = "Failed to set I2C slave address";
        std::cerr << last_error << std::endl;
        return false;
    }
    
    uint8_t buffer[2] = {reg, data};
    if (write(i2c_fd, buffer, 2) != 2) {
        last_error = "Failed to write to register";
        std::cerr << last_error << std::endl;
        return false;
    }
    
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool IMU::readBytes(uint8_t devAddr, uint8_t reg, uint8_t* buffer, size_t length) {
    if (i2c_fd < 0) {
        last_error = "I2C device not open";
        std::cerr << last_error << std::endl;
        return false;
    }
    
    if (ioctl(i2c_fd, I2C_SLAVE, devAddr) < 0) {
        last_error = "Failed to set I2C slave address";
        std::cerr << last_error << std::endl;
        return false;
    }
    
    if (write(i2c_fd, &reg, 1) != 1) {
        last_error = "Failed to write register address";
        std::cerr << last_error << std::endl;
        return false;
    }
    
    if (read(i2c_fd, buffer, length) != static_cast<ssize_t>(length)) {
        last_error = "Failed to read data";
        std::cerr << last_error << std::endl;
        return false;
    }
    
    return true;
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

