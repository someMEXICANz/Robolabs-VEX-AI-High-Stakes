// #include "BNO085_HAL.h"
// #include "sh2.h"
// #include "sh2_SensorValue.h"
// #include <iostream>
// #include <thread>
// #include <chrono>

// class BNO085 {
// private:
//     BNO085_HAL hal;
//     bool initialized;
    
//     // Callback for SH2 events
//     static void eventCallback(void* cookie, sh2_AsyncEvent_t* pEvent) {
//         BNO085* sensor = static_cast<BNO085*>(cookie);
        
//         switch (pEvent->eventId) {
//             case SH2_RESET:
//                 std::cout << "BNO085: Reset event received" << std::endl;
//                 break;
//             case SH2_SHTP_EVENT:
//                 std::cout << "BNO085: SHTP event: " << pEvent->shtpEvent << std::endl;
//                 break;
//             default:
//                 std::cout << "BNO085: Unknown event: " << pEvent->eventId << std::endl;
//                 break;
//         }
//     }
    
//     // Callback for sensor data
//     static void sensorCallback(void* cookie, sh2_SensorEvent_t* pEvent) {
//         // This callback receives raw sensor events
//         // We'll process them in the main loop instead
//     }

// public:
//     BNO085(const std::string& i2c_device = "/dev/i2c-1", uint8_t address = 0x4A)
//         : hal(i2c_device, address), initialized(false) {
//     }
    
//     bool initialize() {
//         std::cout << "Initializing BNO085 Sensor..." << std::endl;
        
//         // Initialize HAL
//         if (!hal.initialize()) {
//             std::cerr << "Failed to initialize HAL" << std::endl;
//             return false;
//         }
        
//         // Open SH2 interface
//         int status = sh2_open(hal.getHalInterface(), eventCallback, this);
//         if (status != SH2_OK) {
//             std::cerr << "Failed to open SH2 interface, status: " << status << std::endl;
//             return false;
//         }
        
//         // Get product IDs to verify communication
//         sh2_ProductIds_t prodIds;
//         memset(&prodIds, 0, sizeof(prodIds));
//         status = sh2_getProdIds(&prodIds);
//         if (status != SH2_OK) {
//             std::cerr << "Failed to get product IDs, status: " << status << std::endl;
//             return false;
//         }
        
//         std::cout << "BNO085 Product Information:" << std::endl;
//         for (int i = 0; i < prodIds.numEntries; i++) {
//             const auto& entry = prodIds.entry[i];
//             std::cout << "  Entry " << i << ":" << std::endl;
//             std::cout << "    SW Version: " << static_cast<int>(entry.swVersionMajor) 
//                       << "." << static_cast<int>(entry.swVersionMinor) 
//                       << "." << entry.swVersionPatch << std::endl;
//             std::cout << "    Part Number: " << entry.swPartNumber << std::endl;
//             std::cout << "    Build: " << entry.swBuildNumber << std::endl;
//         }
        
//         // Register sensor callback
//         sh2_setSensorCallback(sensorCallback, this);
        
//         initialized = true;
//         std::cout << "BNO085 initialized successfully!" << std::endl;
//         return true;
//     }
    
//     bool enableRotationVector(uint32_t interval_us = 10000) { // 10ms default
//         if (!initialized) {
//             std::cerr << "Sensor not initialized" << std::endl;
//             return false;
//         }
        
//         int status = sh2_enableReport(SH2_ROTATION_VECTOR, interval_us);
//         if (status != SH2_OK) {
//             std::cerr << "Failed to enable rotation vector, status: " << status << std::endl;
//             return false;
//         }
        
//         std::cout << "Rotation vector enabled with " << interval_us << "us interval" << std::endl;
//         return true;
//     }
    
//     bool enableAccelerometer(uint32_t interval_us = 10000) {
//         if (!initialized) {
//             std::cerr << "Sensor not initialized" << std::endl;
//             return false;
//         }
        
//         int status = sh2_enableReport(SH2_ACCELEROMETER, interval_us);
//         if (status != SH2_OK) {
//             std::cerr << "Failed to enable accelerometer, status: " << status << std::endl;
//             return false;
//         }
        
//         std::cout << "Accelerometer enabled with " << interval_us << "us interval" << std::endl;
//         return true;
//     }
    
//     bool getSensorData(sh2_SensorValue_t& sensorValue) {
//         if (!initialized) {
//             return false;
//         }
        
//         // Service the SH2 library (handles communication)
//         sh2_service();
        
//         // Try to get a sensor event
//         sh2_SensorEvent_t event;
//         if (sh2_getSensorEvent(&event)) {
//             // Decode the event into a sensor value
//             int status = sh2_decodeSensorEvent(&sensorValue, &event);
//             return (status == SH2_OK);
//         }
        
//         return false;
//     }
    
//     void shutdown() {
//         if (initialized) {
//             sh2_close();
//             hal.shutdown();
//             initialized = false;
//         }
//     }
// };

