#include "UPS.h"



UPS::UPS(const char* i2c_device_) 
    : i2c_device(i2c_device_),
      ina219_fd(-1),
      cal_value(0),
      current_lsb(0),
      power_lsb(0),
      running(false),
      initialized(false),
      current_data{0, 0, 0, 0, 0, false, std::chrono::system_clock::now()}

{
    if(initialize())
    {
        start();
    }


}

UPS::~UPS() 
{
    stop();

    if (ina219_fd >= 0) 
    {
        close(ina219_fd);
        ina219_fd = -1;
    }
}

bool UPS::start() 
{
    if (running)
    {
        std::cerr << "UPS read thread is already running" << std::endl; 
        return true;
    }
    
    try{  

        running = true;
        read_thread = std::make_unique<std::thread>(&UPS::readLoop, this);
        return true;

    } catch (const std::exception& e) 
    {
        std::cerr << "Failed to start UPS read thread: " << e.what() << std::endl;
        running = false;
        return false;  
    }

    return true;
}


void UPS::stop() 
{
    running = false;
    
    if (read_thread && read_thread->joinable()) 
    {
        read_thread->join();
        read_thread.reset();
    }   
}

bool UPS::restart() 
{
    stop();
    return start();
}



bool UPS::initialize() 
{
    initialized = false;

    // Open I2C device
    ina219_fd = open(i2c_device, O_RDWR);
    if (ina219_fd < 0) 
    {
        std::cerr << "Failed to open " << i2c_device << " for LSM6DS3TRC" << std::endl;
        return false;
    }

    // Set I2C slave address
    if (ioctl(ina219_fd, I2C_SLAVE, UPS_ADDR) < 0) 
    {
        std::cerr << "Failed to set INA219 I2C slave address" << std::endl;
        close(ina219_fd);
        ina219_fd = -1;
        return false;
    }

    set_calibration_32V_2A();

    initialized = true;
    return true;
}


// Main reading loop that runs in a separate thread
void UPS::readLoop() 
{
    std::cerr << "UPS read loop started" << std::endl;
    
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
        
        // Calculate time to sleep
        std::chrono::duration elapsed = std::chrono::system_clock::now() - start_time;
        if (elapsed < read_interval) 
        {
            std::this_thread::sleep_for(read_interval - elapsed);
        }
    }
    running = false;
    
    std::cerr << "UPS read loop stopped" << std::endl;
}

float calculateBatteryLevel(float current_busV)
{

    // Calculate percentage using the formula in your Python code
    float p = (current_busV - 6.0f) / 2.4f * 100.0f;
    
    // Clamp between 0% and 100%
    if (p > 100.0f) p = 100.0f;
    if (p < 0.0f) p = 0.0f;

    return p;
}



bool UPS::readData()
{
    bool error = true;

    
    error = writeWord(REG_CALIBRATION, cal_value);
    int16_t shunt_value = readWord(REG_SHUNTVOLTAGE);
    float shunt_voltage =  shunt_value * 0.01f;  // LSB = 10uV
    
    error = writeWord(REG_CALIBRATION, cal_value);
    uint16_t busV_value = readWord(REG_BUSVOLTAGE);
    float bus_voltage = (busV_value >> 3) * 0.004f;

    error = writeWord(REG_CALIBRATION, cal_value);
    int16_t current_value = readWord(REG_CURRENT);
    float current = current_value * current_lsb;

    error = writeWord(REG_CALIBRATION, cal_value);
    int16_t power_value = readWord(REG_POWER);
    float power = power_value * power_lsb;

    float battery_level = calculateBatteryLevel(bus_voltage);

    if (!error) 
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        current_data.valid = false;
        current_data.timestamp = std::chrono::system_clock::now();
        return false;
    }

    // Update data structure with mutex protection
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        current_data.shuntVoltage = shunt_voltage;
        current_data.busVoltage = bus_voltage;
        current_data.current = current;
        current_data.power = power;
        current_data.batteryLevel = battery_level;
        current_data.valid = true;
        current_data.timestamp = std::chrono::system_clock::now();
    }
    return true;

}




void UPS::set_calibration_32V_2A() {
    // Current LSB = 100uA per bit
    current_lsb = 0.1;
    
    // Calibration value calculation
    cal_value = 4096;
    
    // Power LSB = 20 * Current LSB = 2mW per bit
    power_lsb = 0.002;
    
    // Set Calibration register
    writeWord(REG_CALIBRATION, cal_value);
    
    // Set Config register
    bus_voltage_range = RANGE_32V;
    gain = GAIN_8_320MV;
    bus_adc_resolution = ADCRES_12BIT_32S;
    shunt_adc_resolution = ADCRES_12BIT_32S;
    mode = MODE_SANDBVOLT_CONTINUOUS;
    
    config = bus_voltage_range << 13 |
             gain << 11 |
             bus_adc_resolution << 7 |
             shunt_adc_resolution << 3 |
             mode;
             
    writeWord(REG_CONFIG, config);
}

uint32_t UPS::getBatteryPercentage()
{
    std::lock_guard<std::mutex> lock(data_mutex);
    return current_data.batteryLevel;
}

bool UPS::writeByte(uint8_t reg, uint8_t data) 
{
    if (ioctl(ina219_fd, I2C_SLAVE, UPS_ADDR) < 0) 
    {
        //setError("Failed to set I2C slave address");
        return false;
    }
    
    uint8_t buffer[2] = {reg, data};
    if (write(ina219_fd, buffer, 2) != 2) 
    {
        //setError("Failed to write to register");
        return false;
    }
    
    return true;
}

bool UPS::writeWord(uint8_t reg, uint16_t data) 
{
    if (ioctl(ina219_fd, I2C_SLAVE, UPS_ADDR) < 0) 
    {
        //setError("Failed to set I2C slave address");
        return false;
    }
    
    uint8_t buffer[3];
    buffer[0] = reg;
    buffer[1] = (data >> 8) & 0xFF;  // MSB
    buffer[2] = data & 0xFF;         // LSB
    
    if (write(ina219_fd, buffer, 3) != 3) {
        //setError("Failed to write to register");
        return false;
    }
    
    return true;
}

int16_t UPS::readWord(uint8_t reg) 
{
    if (ioctl(ina219_fd, I2C_SLAVE, UPS_ADDR) < 0) 
    {
        //setError("Failed to set I2C slave address");
        return -1;
    }
    
    // Write the register address
    if (write(ina219_fd, &reg, 1) != 1) {
        //setError("Failed to write register address");
        return -1;
    }
    
    // Read the data (2 bytes)
    uint8_t buf[2] = {0};
    if (read(ina219_fd, buf, 2) != 2) {
        //setError("Failed to read from register");
        return -1;
    }
    
    return (buf[0] << 8) | buf[1];
}

// void UPS::setError(const std::string& error) {
//     last_error = error;
//     std::cerr << "UPS Error: " << error << std::endl;
// }


UPSData UPS::getUPSData() const
{
    std::lock_guard<std::mutex> lock(data_mutex);
    return current_data;
}