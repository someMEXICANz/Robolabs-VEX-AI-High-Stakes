// example_main.cpp - Complete usage example for BNO085 class
#include "BNO085.h"
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <thread>
#include <chrono>
#include "UPS.h"

// Global flag for clean shutdown
volatile bool running = true;

void printUPSdata(UPS &ups)
{
    UPSData ups_data = ups.getUPSData();

    std::cerr << "--------------------------------------------" << std::endl;
    // Display UPS data 
    std::cout << "WAVESHARE UPS DATA" << std::endl;

    std::cout << "Data Valid: " << (ups_data.valid ? "TRUE" : "FALSE") << std::endl;

    std::cerr << "Load Voltage:     " << ups_data.busVoltage  << " V" << std::endl;
    std::cerr << "Shunt Voltage:    " << ups_data.shuntVoltage  << " mV" << std::endl;
    std::cerr << "PSU Voltage:      " << (ups_data.busVoltage  + ups_data.shuntVoltage) << " V" << std::endl;
    std::cerr << "Current:          " << (ups_data.current) << " mA" << std::endl;
    std::cerr << "Power:            " << ups_data.power << " mW" << std::endl;
    std::cerr << "Battery Level:    " << ups_data.batteryLevel << "%" << std::endl;

    std::time_t time = std::chrono::system_clock::to_time_t(ups_data.timestamp);
    std::tm* tm_now = std::localtime(&time);
    // Print the formatted time                                              
    std::cout << "--------------------------------------------" << std::endl;
    std::cout << "Timestamp: " << std::put_time(tm_now, "%Y-%m-%d %H:%M:%S") << std::endl;
    std::cout << "--------------------------------------------" << std::endl;
}


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
    
        UPS ups;
        BNO085 sensor;

        
        // Set up callbacks
        // sensor.setIMUCallback(onIMUData);
        sensor.setOrientationCallback(onOrientationData);
        // sensor.setSensorEventCallback(onSensorEvent);
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
        auto lastDataPrint = std::chrono::steady_clock::now();
        
        while (running) 
        {
            auto now = std::chrono::steady_clock::now();
            
            // Print calibration status every 5 seconds
            if (now - lastDataPrint > std::chrono::seconds(5)) 
                if(ups.isRunning())
                {
                    printUPSdata(ups);
                    lastDataPrint = now;
                }

            // Sleep for a short time
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Cleanup is automatic via destructor
        std::cerr << "Shutdown complete" << std::endl;
        
    
    return 0;
}
















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
// //         
            
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


