#ifndef UPS_H
#define UPS_H

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>


class UPS {



public:
    // Constructor
    UPS( const std::string& i2c_bus = "/dev/i2c-1");
    
    // Destructor
    ~UPS();
    
    // Initialize and calibrate
    bool initialize();
    bool start();
    void stop();
    bool restart();
    bool reconnect();

     // Check running state
    bool isRunning() const { return running; }
    bool isConnected() const { return i2c_fd >= 0; }
    
    // Read sensor data
    float getShuntVoltage_mV();
    float getBusVoltage_V();
    float getCurrent_mA();
    float getPower_W();
    float getBatteryPercentage();
    bool getAll(float& shuntVoltage, float& busVoltage, float& current,
                 float& power, float& batteryLevel);

    void set_calibration_32V_2A();
    
    // Error handling
    std::string getLastError() const { return last_error; }

private:
    
    void readLoop();
    bool readData();

    // I2C communication
    bool writeByte(uint8_t reg, uint8_t data);
    bool writeWord(uint8_t reg, uint16_t data);
    int16_t readWord(uint8_t reg);

    // I2C device properties
    std::string i2c_bus;
    int i2c_fd;
    std::string last_error;

     // Sensor address
    static constexpr uint8_t UPS_ADDR = 0x42;

    // Calibration values
    uint16_t cal_value;
    float current_lsb;
    float power_lsb;


     // Thread and running state
    mutable std::mutex data_mutex;
    std::unique_ptr<std::thread> read_thread;
    std::atomic<bool> running;
    std::atomic<bool> connected;
    
    // Configuration values
    uint8_t bus_voltage_range;
    uint8_t gain;
    uint8_t bus_adc_resolution;
    uint8_t shunt_adc_resolution;
    uint8_t mode;
    uint16_t config;


      
    struct UPSData {
        float shuntVoltage,  
              busVoltage,
              current,
              power,
              batteryLevel;
        bool valid;        
    } sensor_data;
    

    // Set error message
    void setError(const std::string& error);


};

#endif // UPS_H