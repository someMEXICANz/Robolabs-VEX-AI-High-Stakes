#ifndef BNO085_HAL_TESTER_H
#define BNO085_HAL_TESTER_H

#include "I2C_HAL.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include <memory>
#include <string>
#include <chrono>

class BNO085_HAL_Tester {
public:
    BNO085_HAL_Tester(const std::string& i2c_device_path, uint8_t i2c_address = 0x4A);
    ~BNO085_HAL_Tester();
    
    // Test functions
    bool runAllTests();
    bool testBasicInitialization();
    bool testProductIdRetrieval();
    bool testSensorConfiguration();
    bool testSensorDataReading();
    bool testTimingFunctions();
    
    // Event callbacks for SH2
    static void eventCallback(void* cookie, sh2_AsyncEvent_t* pEvent);
    static void sensorCallback(void* cookie, sh2_SensorEvent_t* pEvent);
    
private:
    std::unique_ptr<I2C_HAL> hal;
    bool initialization_successful;
    bool reset_occurred;
    bool sensor_data_received;
    sh2_SensorValue_t last_sensor_value;
    sh2_ProductIds_t product_ids;
    
    // Helper functions
    void printProductIds();
    void printSensorValue(const sh2_SensorValue_t& value);
    bool waitForReset(uint32_t timeout_ms = 5000);
    bool waitForSensorData(uint32_t timeout_ms = 5000);
    
    // Test individual components
    bool testHALFunctions();
    bool testSH2Protocol();
    void debugSH2State();
    void debugDataPattern();


};

#endif // BNO085_HAL_TESTER_H