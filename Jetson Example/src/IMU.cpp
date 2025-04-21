#include "IMU.h"

IMU::IMU(const std::string& i2cBus_)
    : i2c_bus(i2cBus_),
      i2c_fd(-1),
      accel_offset_x(0.0f), accel_offset_y(0.0f), accel_offset_z(0.0f),
      gyro_offset_x(0.0f), gyro_offset_y(0.0f), gyro_offset_z(0.0f),
      mag_offset_x(0.0f), mag_offset_y(0.0f), mag_offset_z(0.0f),
      accel_scale(0.061f / 1000.0f),  // ±2g range: 0.061 mg/LSB
      gyro_scale(70.0f / 1000.0f),    // ±2000 dps range: 70 mdps/LSB
      mag_scale(0.58f / 1000.0f),     // ±16 gauss range: 0.58 mgauss/LSB
      running(false),
      connected(false)
{
    // Initialize sensor data
    current_data.ax = 0.0f;
    current_data.ay = 0.0f;
    current_data.az = 0.0f;
    current_data.gx = 0.0f;
    current_data.gy = 0.0f;
    current_data.gz = 0.0f;
    current_data.mx = 0.0f;
    current_data.my = 0.0f;
    current_data.mz = 0.0f;
    current_data.temperature = 0.0f;
    current_data.timestamp = std::chrono::high_resolution_clock::now();
    current_data.valid = false;
    
    // Try to initialize if bus is provided
    if (!i2c_bus.empty()) 
    {
        if(initialize())
        {
            start();
        }
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

bool IMU::initialize() {
    // Open I2C device
    i2c_fd = open(i2c_bus.c_str(), O_RDWR);
    if (i2c_fd< 0) {
        std::cerr << "Failed to open I2C bus: " << i2c_bus << std::endl;
        return false;
    }

    // Initialize accelerometer and gyroscope
    if (!writeByte(LSM6DS3_ADDR, LSM6DS3_CTRL1_XL, 0x60)) {  // 208Hz, ±2g
        std::cerr << "Failed to configure accelerometer" << std::endl;
        return false;
    }

    if (!writeByte(LSM6DS3_ADDR, LSM6DS3_CTRL2_G, 0x6C)) {  // 208Hz, ±2000dps
        std::cerr << "Failed to configure gyroscope" << std::endl;
        return false;
    }

    // Initialize magnetometer
    if (!writeByte(LIS3MDL_ADDR, LIS3MDL_CTRL_REG1, 0x7C)) {  // Ultra-high performance, 80Hz
        std::cerr << "Failed to configure magnetometer" << std::endl;
        return false;
    }

    if (!writeByte(LIS3MDL_ADDR, LIS3MDL_CTRL_REG2, 0x40)) {  // ±16 gauss
        std::cerr << "Failed to set magnetometer range" << std::endl;
        return false;
    }

    if (!writeByte(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3, 0x00)) {  // Continuous mode
        std::cerr << "Failed to set magnetometer mode" << std::endl;
        return false;
    }
    
    connected = true;
    return true;
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

bool IMU::reconnect() {
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

// Read all sensors and update internal data structure
bool IMU::readData() {
    if (i2c_fd< 0) 
    {
        return false;
    }

    bool error = false;

    // Read magnetometer
    uint8_t mag_buffer[6];
    if (!readBytes(LIS3MDL_ADDR, LIS3MDL_OUT_X_L, mag_buffer, 6)) 
    {
         std::cerr << "Failed to read magnetometer data" << std::endl;
         error = true;
    }
    // Read accelerometer
    uint8_t accel_buffer[6];
    if (!readBytes(LSM6DS3_ADDR, LSM6DS3_OUTX_L_XL, accel_buffer, 6)) 
    {
        std::cerr << "Failed to read accelerometer data" << std::endl;
        error = true;
    }
    // Read gyroscope
    uint8_t gyro_buffer[6];
    if (!readBytes(LSM6DS3_ADDR, LSM6DS3_OUTX_L_G, gyro_buffer, 6)) 
    {
        std::cerr << "Failed to read gyroscope data" << std::endl;
        error = true;
    }
    // Read temperature
    uint8_t temp_buffer[2];
    if (!readBytes(LSM6DS3_ADDR, LSM6DS3_OUT_TEMP_L, temp_buffer, 2)) 
    {
        std::cerr << "Failed to read temperature data" << std::endl;
        error = true;
    }

    if(error)
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        current_data.timestamp = std::chrono::high_resolution_clock::now();
        current_data.valid = false;
        return false;
    }
    else
    {
        
        // Process all readings
        int16_t raw_ax = (accel_buffer[1] << 8) | accel_buffer[0];
        int16_t raw_ay = (accel_buffer[3] << 8) | accel_buffer[2];
        int16_t raw_az = (accel_buffer[5] << 8) | accel_buffer[4];
        
        int16_t raw_gx = (gyro_buffer[1] << 8) | gyro_buffer[0];
        int16_t raw_gy = (gyro_buffer[3] << 8) | gyro_buffer[2];
        int16_t raw_gz = (gyro_buffer[5] << 8) | gyro_buffer[4];
        
        int16_t raw_mx = (mag_buffer[1] << 8) | mag_buffer[0];
        int16_t raw_my = (mag_buffer[3] << 8) | mag_buffer[2];
        int16_t raw_mz = (mag_buffer[5] << 8) | mag_buffer[4];
        
        int16_t raw_temp = (temp_buffer[1] << 8) | temp_buffer[0];
        
        // Apply scaling and offsets
        float ax = (raw_ax * accel_scale) - accel_offset_x;
        float ay = (raw_ay * accel_scale) - accel_offset_y;
        float az = (raw_az * accel_scale) - accel_offset_z;
        
        float gx = (raw_gx * gyro_scale) - gyro_offset_x;
        float gy = (raw_gy * gyro_scale) - gyro_offset_y;
        float gz = (raw_gz * gyro_scale) - gyro_offset_z;
        
        float mx = (raw_mx * mag_scale) - mag_offset_x;
        float my = (raw_my * mag_scale) - mag_offset_y;
        float mz = (raw_mz * mag_scale) - mag_offset_z;
        
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
            current_data.timestamp = std::chrono::high_resolution_clock::now();
            current_data.valid = true;
        }
    }
    
    return true;
}



bool IMU::calibrateAccelerometer() 
{
    if (!running || !connected) 
    {
        std::cerr << "IMU must be running and connected to calibrate" << std::endl;
        return false;
    }
    std::lock_guard<std::mutex> lock(data_mutex);
    // Check if device is stationary
    if (current_data.valid) 
    {
        float movement = std::abs(current_data.gx) + std::abs(current_data.gy) + std::abs(current_data.gz);
        if (movement > 1.0f) 
        {
            std::cerr << "IMU must be stationary for calibration" << std::endl;
            return false;
        }
    }
    
    const int numSamples = 100;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    
    for (int i = 0; i < numSamples; ++i) 
    {
        sumX += current_data.ax;
        sumY += current_data.ay;
        sumZ += current_data.az;
        usleep(10000);  // 10ms delay
    }
    
    accel_offset_x= sumX / numSamples;
    accel_offset_y= sumY / numSamples;
    accel_offset_z= (sumZ / numSamples) - 1.0f;  // Remove gravity (1g)
    
    std::cerr << "Accelerometer calibration: offsets = (" 
              << accel_offset_x<< ", " << accel_offset_y<< ", " 
              << accel_offset_z<< ")" << std::endl;
    
    return true;
}

bool IMU::calibrateGyroscope() 
{
    if (!running || !connected) 
    {
        std::cerr << "IMU must be running and connected to calibrate" << std::endl;
        return false;
    }
    
    std::lock_guard<std::mutex> lock(data_mutex);

    if (current_data.valid) 
    {
        float movement = std::abs(current_data.gx) + std::abs(current_data.gy) + std::abs(current_data.gz);
        if (movement > 1.0f) 
        {
            std::cerr << "Device must be stationary for calibration" << std::endl;
            return false;
        }
    }


    const int numSamples = 100;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    
    for (int i = 0; i < numSamples; ++i) 
    {
        sumX += current_data.gx;
        sumY += current_data.gy;
        sumZ += current_data.gz;
        usleep(10000);  // 10ms delay
    }
    
    gyro_offset_x = sumX / numSamples;
    gyro_offset_y = sumY / numSamples;
    gyro_offset_z = sumZ / numSamples;
    
    return true;
}

bool IMU::calibrateMagnetometer() 
{

    if (!running || !connected) 
    {
        std::cerr << "IMU must be running and connected to calibrate" << std::endl;
        return false;
    }
    
    // Check if device is stationary
    if (current_data.valid) 
    {
        float movement = std::abs(current_data.gx) + std::abs(current_data.gy) + std::abs(current_data.gz);
        if (movement > 1.0f) 
        {
            std::cerr << "Device must be stationary for calibration" << std::endl;
            return false;
        }
    }

    const int numSamples = 100;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    
    for (int i = 0; i < numSamples; ++i) 
    {   
        sumX += current_data.mx;
        sumY += current_data.my;
        sumZ += current_data.mz;
        usleep(10000);  // 10ms delay
    }
    
    
    mag_offset_x= sumX / numSamples;
    mag_offset_y= sumY / numSamples;
    mag_offset_z= sumZ / numSamples;
    
    return true;
}


bool IMU::writeByte(uint8_t devAddr, uint8_t reg, uint8_t data) {
    if (i2c_fd< 0) 
    {
        std::cerr << "I2C device not open" << std::endl;
        return false;
    }
    
    if (ioctl(i2c_fd, I2C_SLAVE, devAddr) < 0) 
    {
        std::cerr << "Failed to set I2C slave address" << std::endl;
        return false;
    }
    
    uint8_t buffer[2] = {reg, data};
    if (write(i2c_fd, buffer, 2) != 2) 
    {
        std::cerr << "Failed to write to register" << std::endl;
        return false;
    }
    
    return true;
}

bool IMU::readBytes(uint8_t devAddr, uint8_t reg, uint8_t* buffer, size_t length) {
    if (i2c_fd< 0) 
    {
        std::cerr << "I2C device not open" << std::endl;
        return false;
    }
    
    if (ioctl(i2c_fd, I2C_SLAVE, devAddr) < 0) 
    {
        std::cerr << "Failed to set I2C slave address" << std::endl;
        return false;
    }
    
    if (write(i2c_fd, &reg, 1) != 1) 
    {
        std::cerr << "Failed to write register address" << std::endl;
        return false;
    }
    
    if (read(i2c_fd, buffer, length) != static_cast<ssize_t>(length)) 
    {
        std::cerr << "Failed to read data" << std::endl;
        return false;
    }
    
    return true;
}

IMUData IMU::getIMUData() const 
{
    std::lock_guard<std::mutex> lock(data_mutex);
    return current_data;
}

