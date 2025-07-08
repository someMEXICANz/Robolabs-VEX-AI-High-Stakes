// example_main.cpp - Complete usage example for BNO085 class
#include "BNO085.h"
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <thread>
#include <chrono>

// Global flag for clean shutdown
volatile bool running = true;

void signalHandler(int signal) {
    std::cerr << "\nShutdown signal received (" << signal << ")" << std::endl;
    running = false;
}

// Callback functions for sensor data
void onIMUData(const BNO085::IMUData& data) {
    std::cerr << std::fixed << std::setprecision(3);
    std::cerr << "[IMU] Accel: " << data.acceleration.toString() 
              << " Gyro: " << data.gyroscope.toString()
              << " Mag: " << data.magnetometer.toString()
              << " (Status: " << static_cast<int>(data.status) << ")" << std::endl;
}

void onOrientationData(const BNO085::OrientationData& data) {
    std::cerr << std::fixed << std::setprecision(1);
    std::cerr << "[ORIENTATION] Yaw: " << data.euler.x 
              << "° Pitch: " << data.euler.y 
              << "° Roll: " << data.euler.z 
              << "° (Accuracy: " << data.rotation.accuracy << ")" << std::endl;
}

void onSensorEvent(sh2_SensorId_t sensorId, const sh2_SensorValue_t& value) {
    // This gets called for ALL sensor events
    // You can use this for detailed logging or custom processing
    static int eventCount = 0;
    if (++eventCount % 100 == 0) {  // Print every 100th event
        std::cerr << "[EVENT] Received " << eventCount << " sensor events" << std::endl;
    }
}

void onError(const std::string& error) {
    std::cerr << "[ERROR] " << error << std::endl;
}

void printCalibrationStatus(BNO085& sensor) {
    BNO085::CalibrationStatus accel, gyro, mag, system;
    sensor.getCalibrationStatus(accel, gyro, mag, system);
    
    auto statusToString = [](BNO085::CalibrationStatus status) -> std::string {
        switch (status) {
            case BNO085::CalibrationStatus::UNCALIBRATED:    return "UNCAL";
            case BNO085::CalibrationStatus::LOW_ACCURACY:    return "LOW";
            case BNO085::CalibrationStatus::MEDIUM_ACCURACY: return "MED";
            case BNO085::CalibrationStatus::HIGH_ACCURACY:   return "HIGH";
            default: return "UNK";
        }
    };
    
    std::cerr << "[CALIB] Accel: " << statusToString(accel)
              << " Gyro: " << statusToString(gyro)
              << " Mag: " << statusToString(mag)
              << " System: " << statusToString(system);
    
    if (sensor.isFullyCalibrated()) {
        std::cerr << " [FULLY CALIBRATED]";
    }
    std::cerr << std::endl;
}

int main() {
    std::cerr << "BNO085 Sensor Test Application" << std::endl;
    std::cerr << "==============================" << std::endl;
    
    // Set up signal handlers for clean shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        // Create BNO085 instance
        // Using default I2C settings (/dev/i2c-1, address 0x4A)
        BNO085 sensor;
        
        // Or create with custom settings:
        // BNO085 sensor("/dev/i2c-0", 0x4B);
        
        // Set up callbacks
        sensor.setIMUCallback(onIMUData);
        sensor.setOrientationCallback(onOrientationData);
        sensor.setSensorEventCallback(onSensorEvent);
        sensor.setErrorCallback(onError);
        
        // Initialize the sensor
        std::cerr << "\nInitializing sensor..." << std::endl;
        if (!sensor.initialize()) {
            std::cerr << "Failed to initialize sensor: " << sensor.getLastError() << std::endl;
            return -1;
        }
        
        // Print product information
        std::cerr << "\n" << sensor.getProductInfo() << std::endl;
        
        // Check connection
        if (!sensor.isConnected()) {
            std::cerr << "Sensor not responding!" << std::endl;
            return -1;
        }
        
        std::cerr << "Sensor connected and ready" << std::endl;
        
        // Enable sensors
        std::cerr << "\nConfiguring sensors..." << std::endl;
        
        // Option 1: Enable basic IMU sensors (accel, gyro, mag) at 100Hz
        if (!sensor.enableBasicIMU()) {
            std::cerr << "Failed to enable basic IMU" << std::endl;
            return -1;
        }
        
        // Option 2: Enable orientation sensing at 50Hz
        if (!sensor.enableOrientation()) {
            std::cerr << "Failed to enable orientation" << std::endl;
            return -1;
        }
        
        // Option 3: Enable individual sensors with custom settings
        BNO085::SensorConfig customConfig;
        customConfig.setFrequency(50.0f);  
        customConfig.wakeupEnabled = false;
        customConfig.alwaysOnEnabled = false;
        
        if (!sensor.enableSensor(BNO085::SensorType::LINEAR_ACCELERATION, customConfig)) {
            std::cerr << "Failed to enable linear acceleration" << std::endl;
        }
        
        // Start the service thread for automatic data processing
        std::cerr << "\nStarting sensor service..." << std::endl;
        sensor.startService();
        
        std::cerr << "Data collection started. Press Ctrl+C to exit.\n" << std::endl;
        
        // Main loop - print status periodically
        auto lastCalibPrint = std::chrono::steady_clock::now();
        auto lastDataPrint = std::chrono::steady_clock::now();
        
        while (running) {
            auto now = std::chrono::steady_clock::now();
            
            // Print calibration status every 5 seconds
            if (now - lastCalibPrint > std::chrono::seconds(5)) {
                printCalibrationStatus(sensor);
                lastCalibPrint = now;
            }
            
            // Print latest data every 2 seconds (alternative to callbacks)
            if (now - lastDataPrint > std::chrono::seconds(2)) {
                BNO085::Vector3 accel, gyro, mag;
                float yaw, pitch, roll;
                
                if (sensor.getAcceleration(accel) && 
                    sensor.getGyroscope(gyro) && 
                    sensor.getMagnetometer(mag) &&
                    sensor.getEulerAngles(yaw, pitch, roll)) {
                    
                    std::cerr << std::fixed << std::setprecision(2);
                    std::cerr << "[POLL] Accel: " << accel.toString()
                              << " Gyro: " << gyro.toString()
                              << " Orientation: (" << yaw << "°, " << pitch << "°, " << roll << "°)"
                              << std::endl;
                }
                lastDataPrint = now;
            }
            
            // Sleep for a short time
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cerr << "\nShutting down..." << std::endl;
        
        // Save calibration data if fully calibrated
        if (sensor.isFullyCalibrated()) {
            std::cerr << "Saving calibration data..." << std::endl;
            if (sensor.saveCalibration()) {
                std::cerr << "Calibration data saved successfully" << std::endl;
            }
        }
        
        // Cleanup is automatic via destructor
        std::cerr << "Shutdown complete" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}

// =============================================================================
// ADDITIONAL EXAMPLE FUNCTIONS
// =============================================================================

// Example: Simple orientation monitoring
void orientationMonitorExample() {
    BNO085 sensor;
    
    if (!sensor.initialize()) {
        return;
    }
    
    sensor.enableOrientation();
    sensor.startService();
    
    // Monitor orientation for specific conditions
    sensor.setOrientationCallback([](const BNO085::OrientationData& data) {
        // Check if device is approximately level
        if (std::abs(data.euler.y) < 5.0f && std::abs(data.euler.z) < 5.0f) {
            std::cerr << "Device is level!" << std::endl;
        }
        
        // Check for specific orientation
        if (data.euler.y > 45.0f) {
            std::cerr << "Device tilted forward significantly!" << std::endl;
        }
    });
    
    // Run for a while...
    std::this_thread::sleep_for(std::chrono::seconds(30));
}

// Example: Motion detection
void motionDetectionExample() {
    BNO085 sensor;
    
    if (!sensor.initialize()) {
        return;
    }
    
    sensor.enableBasicIMU();
    sensor.startService();
    
    // Detect motion based on acceleration magnitude
    sensor.setIMUCallback([](const BNO085::IMUData& data) {
        float accelMagnitude = data.acceleration.magnitude();
        
        // Earth's gravity is approximately 9.81 m/s²
        // Detect significant deviation from gravity
        if (std::abs(accelMagnitude - 9.81f) > 2.0f) {
            std::cerr << "Motion detected! Acceleration magnitude: " 
                      << accelMagnitude << " m/s²" << std::endl;
        }
    });
    
    // Run for a while...
    std::this_thread::sleep_for(std::chrono::seconds(30));
}

// Example: Calibration procedure
bool performCalibrationProcedure(BNO085& sensor) {
    std::cerr << "Starting calibration procedure..." << std::endl;
    std::cerr << "Please move the sensor in a figure-8 pattern for magnetometer calibration." << std::endl;
    std::cerr << "Place the sensor in different orientations for accelerometer calibration." << std::endl;
    std::cerr << "Keep the sensor stationary for gyroscope calibration." << std::endl;
    
    if (!sensor.startCalibration(20000)) {  // 50Hz during calibration
        return false;
    }
    
    // Monitor calibration progress
    auto startTime = std::chrono::steady_clock::now();
    auto timeout = std::chrono::minutes(2);  // 2 minute timeout
    
    while (std::chrono::steady_clock::now() - startTime < timeout) {
        if (sensor.isFullyCalibrated()) {
            std::cerr << "Calibration complete!" << std::endl;
            break;
        }
        
        // Print status every 5 seconds
        static auto lastPrint = std::chrono::steady_clock::now();
        if (std::chrono::steady_clock::now() - lastPrint > std::chrono::seconds(5)) {
            printCalibrationStatus(sensor);
            lastPrint = std::chrono::steady_clock::now();
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return sensor.finishCalibration();
}






















































































































































// #include <iostream>
// #include <random>
// #include <chrono>
// #include <thread>
// #include <cstring>

// // Your C++ includes
// #include <WS_UPS.h>
// #include <BrainComm.h>
// #include <Model.h>
// #include <ObjectDetection.h>
// #include <Camera.h>
// #include <I2C_HAL.h>



// #include "I2C_HAL.h"
// #include "sh2_c_interface.h"

// int main() {
//     // Create HAL with default settings
//     sh2_Hal_t* hal = createBNO085HAL();
    
//     // Or create with custom settings
//     // sh2_Hal_t* hal = createI2CHAL("/dev/i2c-0", 0x4B);
    
//     if (!hal) {
//         std::cerr << "Failed to create HAL" << std::endl;
//         return -1;
//     }
    
//     // Use with SH2 library
//     // int result = sh2_open(hal, eventCallback, nullptr);
//     // if (result != SH2_OK) {
//     //     std::cerr << "SH2 open failed: " << sh2ErrorToString(result) << std::endl;
//     //     destroyI2CHAL(hal);
//     //     return -1;
//     // }
    
//     // ... use sensor ...
    
//     sh2_close();
//     destroyI2CHAL(hal);
//     return 0;
// }


// // void printUPSdata(UPS &ups)
// // {
// //     UPSData ups_data = ups.getUPSData();

// //     std::cerr << "--------------------------------------------" << std::endl;
// //     // Display UPS data 
// //     std::cout << "WAVESHARE UPS DATA" << std::endl;

// //     std::cout << "Data Valid: " << (ups_data.valid ? "TRUE" : "FALSE") << std::endl;

// //     std::cerr << "Load Voltage:     " << ups_data.busVoltage  << " V" << std::endl;
// //     std::cerr << "Shunt Voltage:    " << ups_data.shuntVoltage  << " mV" << std::endl;
// //     std::cerr << "PSU Voltage:      " << (ups_data.busVoltage  + ups_data.shuntVoltage) << " V" << std::endl;
// //     std::cerr << "Current:          " << (ups_data.current) << " mA" << std::endl;
// //     std::cerr << "Power:            " << ups_data.power << " mW" << std::endl;
// //     std::cerr << "Battery Level:    " << ups_data.batteryLevel << "%" << std::endl;

// //     std::time_t time = std::chrono::system_clock::to_time_t(ups_data.timestamp);
// //     std::tm* tm_now = std::localtime(&time);
// //     // Print the formatted time                                              
// //     std::cout << "--------------------------------------------" << std::endl;
// //     std::cout << "Timestamp: " << std::put_time(tm_now, "%Y-%m-%d %H:%M:%S") << std::endl;
// //     std::cout << "--------------------------------------------" << std::endl;
// // }



// // // void printIMUData(BNO085 &imu)
// // {
// //     IMUData imu_data = imu.getIMUData();
// //     CalibrationStatus cal_status = imu.getCalibrationStatus();

// //     std::cout << "============================================" << std::endl;
// //     std::cout << "BNO085 IMU DATA" << std::endl;
// //     std::cout << "============================================" << std::endl;

// //     std::cout << "Last Error: " << imu.getLastError() << std::endl;
// //     std::cout << "Data Valid: " << (imu_data.valid ? "TRUE" : "FALSE") << std::endl;
// //     std::cout << "Accuracy: " << static_cast<int>(imu_data.accuracy) << "/3" << std::endl;
// //     std::cout << "Calibrated: " << (cal_status.is_calibrated ? "TRUE" : "FALSE") << std::endl;
    
// //     if (imu_data.valid) {
// //         std::cout << std::fixed << std::setprecision(3);
        
// //         // Raw Accelerometer (m/s²)
// //         std::cout << "Accel (m/s²):  X: " << std::setw(8) << imu_data.accel_x 
// //                   << "  Y: " << std::setw(8) << imu_data.accel_y 
// //                   << "  Z: " << std::setw(8) << imu_data.accel_z << std::endl;
        
// //         // Raw Gyroscope (rad/s)
// //         std::cout << "Gyro (rad/s):  X: " << std::setw(8) << imu_data.gyro_x 
// //                   << "  Y: " << std::setw(8) << imu_data.gyro_y 
// //                   << "  Z: " << std::setw(8) << imu_data.gyro_z << std::endl;
        
// //         // Raw Magnetometer (µTesla)
// //         std::cout << "Mag (µT):      X: " << std::setw(8) << imu_data.mag_x 
// //                   << "  Y: " << std::setw(8) << imu_data.mag_y 
// //                   << "  Z: " << std::setw(8) << imu_data.mag_z << std::endl;
        
// //         // Linear Acceleration (m/s²)
// //         std::cout << "Lin Accel:     X: " << std::setw(8) << imu_data.linear_accel_x 
// //                   << "  Y: " << std::setw(8) << imu_data.linear_accel_y 
// //                   << "  Z: " << std::setw(8) << imu_data.linear_accel_z << std::endl;
        
// //         // Gravity (m/s²)
// //         std::cout << "Gravity:       X: " << std::setw(8) << imu_data.gravity_x 
// //                   << "  Y: " << std::setw(8) << imu_data.gravity_y 
// //                   << "  Z: " << std::setw(8) << imu_data.gravity_z << std::endl;
        
// //         // Euler Angles (degrees)
// //         std::cout << std::fixed << std::setprecision(1);
// //         std::cout << "Euler (deg):   Roll: " << std::setw(7) << imu_data.euler_roll 
// //                   << "  Pitch: " << std::setw(7) << imu_data.euler_pitch 
// //                   << "  Yaw: " << std::setw(7) << imu_data.euler_yaw << std::endl;
        
// //         // Quaternion
// //         std::cout << std::fixed << std::setprecision(4);
// //         std::cout << "Quaternion:    I: " << std::setw(7) << imu_data.quat_i 
// //                   << "  J: " << std::setw(7) << imu_data.quat_j 
// //                   << "  K: " << std::setw(7) << imu_data.quat_k 
// //                   << "  Real: " << std::setw(7) << imu_data.quat_real << std::endl;
// //     } else {
// //         std::cout << "No valid IMU data available" << std::endl;
// //         std::cout << "Last Error: " << imu.getLastError() << std::endl;
// //     }

// //     std::time_t time = std::chrono::system_clock::to_time_t(imu_data.timestamp);
// //     std::tm* tm_now = std::localtime(&time);
// //     std::cout << "Timestamp: " << std::put_time(tm_now, "%Y-%m-%d %H:%M:%S") << std::endl;
// //     std::cout << "============================================" << std::endl;
// // }


























// // void printIMUData(IMU &imu)
// // {
// //     IMUData raw_data = imu.getSensorData();
// //     OrientationData orient_data = imu.getOrientationData();
 
// //     std::cout << "--------------------------------------------" << std::endl;
// //     // Display Raw IMU Data
// //     std::cout << "IMU RAW DATA" << std::endl;
// //     std::cout << "Accel (g):  X: " << raw_data.ax << "  Y: " << raw_data.ay << "  Z: " << raw_data.az << std::endl;     // Accelerometer data in g
// //     std::cout << "Gyro (dps): X: " << raw_data.gx << "  Y: " << raw_data.gy << "  Z: " << raw_data.gz << std::endl;     // Gyroscope data in degrees per second
// //     std::cout << "Mag (gauss): X: " << raw_data.mx << "  Y: " << raw_data.my << "  Z: " << raw_data.mz << std::endl;    // Magnetometer data in gauss
// //     // std::cout << "Temperature: " << raw_data.temperature << " °C" << std::endl;                                         // Temperature in Celsius
    
// //     // Display Raw IMU Data
// //     std::cout << "IMU ORIENTATION DATA" << std::endl;
// //     std::cout << "Euler (deg):  Roll: " << orient_data.roll << "  Pitch: " << orient_data.pitch << "  Yaw: " << orient_data.yaw << std::endl;      
// //     // std::cout << "Quaternion (deg): W: " << orient_data.qw << " X: " << orient_data.qx << "  Y: " << orient_data.qy << "  Z: " << orient_data.qz << std::endl;                  
                         
// //     std::time_t time = std::chrono::system_clock::to_time_t(raw_data.timestamp);
// //     std::tm* tm_now = std::localtime(&time);
// //     // // Print the formatted time                                              
// //     // std::cout << "--------------------------------------------" << std::endl;
// //     std::cout << "Timestamp: " << std::put_time(tm_now, "%Y-%m-%d %H:%M:%S") << std::endl;
// //     // std::cout << "Valid Data: " ;
// //     // if(raw_data.valid)
// //     //     std::cout << "TRUE " << std::endl ;
// //     // else
// //     //     std::cout << "FALSE " << std::endl ;
// //     std::cout << "--------------------------------------------" << std::endl;
// // }





// // int main() 
// // {
// //     //boost::asio::io_service myService;

// //     UPS ups; // (threaded)
// //     // std::this_thread::sleep_for(std::chrono::seconds(1));
// //     // IMU imu; // (threaded)
// //     // std::this_thread::sleep_for(std::chrono::seconds(1));
    
// //     // Camera camera; // (threaded)
// //     std::this_thread::sleep_for(std::chrono::seconds(1));
// //     // Brain::BrainComm brain(myService); // (threaded)
// //     // std::this_thread::sleep_for(std::chrono::seconds(1));

// //     // RobotPosition robotPosition(brain, imu, myService); // (threaded) 
// //     // std::this_thread::sleep_for(std::chrono::seconds(1));

// //     // Model model;
// //     // ObjectDetection objdet;
// //     // std::vector<DetectedObject> Detections;
// //     // std::this_thread::sleep_for(std::chrono::seconds(1));
    
// //     // //FieldMapper mapper(camera); //, robotPosition); // (threaded)
// //     // std::this_thread::sleep_for(std::chrono::seconds(1));


// //     // imu.calibrateAccelerometer();
    
 


    

// //     while (true) 
// //     {
// //         // std::cerr << "Camera FPS: " << camera.getFPS() << std::endl;
// //         // //std::cerr << "Mapper PPS: " << mapper.getPPS() << std::endl;

// //         // if(camera.getInferFrame(model.inferInput))
// //         // {
// //         //     model.runInference();
// //         //     Detections = objdet.decodeOutputs(model.inferOutput1, model.inferOutput2);
// //         //     std::cerr << "Found " << Detections.size() << " detected objects" << std::endl;;
// //         // }
// //         // if(imu.isRunning())
// //         //     printIMUData(imu);
// //         if(ups.isRunning())
// //             printUPSdata(ups);
            
// //         // if(brain.isConnected() && brain.isRunning())
// //         // {
// //         //     brain.setJetsonBattery(ups.getBatteryPercentage());
// //         // }

// //         std::this_thread::sleep_for(std::chrono::milliseconds(250));    
// //     }
// //     //mapper.stop();
// //     // camera.stop();
// //     // robotPosition.stop();
// //     // brain.stop();
// //     // imu.stop();
// //     ups.stop();
    
    
// //     return 0;
// // }


