
#include <iostream>
#include <random>
#include <chrono>

#include <WS_UPS.h>
#include <BrainComm.h>
#include <Model.h>
#include <ObjectDetection.h>
#include <Camera.h>





using namespace std;

// void printUPSdata(UPS &ups);
// // void printIMUData(BNO085 &imu);






int main() 
{
    //UPS ups; // (threaded)
    // BNO085 imu; // (threaded)
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Give IMU more time to initialize

    // Enable sensors with 100Hz rate (10000 microseconds = 10ms)
    // if (imu.isInitialized()) {
    //     std::cout << "Enabling IMU sensors..." << std::endl;
    //     imu.enableAccelerometer(10000);
    //     imu.enableGyroscope(10000);
    //     imu.enableMagnetometer(10000);
    //     imu.enableRotationVector(10000);
    //     imu.enableLinearAcceleration(10000);
    //     imu.enableGravity(10000);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // }

    while (true) 
    {
   
        // if(ups.isRunning())
        //     printUPSdata(ups);
        // if(imu.isRunning())
        //     printIMUData(imu);
        std::this_thread::sleep_for(std::chrono::milliseconds(250));    
    }
    //ups.stop();
    // imu.stop();
    return 0;
}




// void printUPSdata(UPS &ups)
// {
//     UPSData ups_data = ups.getUPSData();

//     std::cerr << "--------------------------------------------" << std::endl;
//     // Display UPS data 
//     std::cout << "WAVESHARE UPS DATA" << std::endl;

//     std::cout << "Data Valid: " << (ups_data.valid ? "TRUE" : "FALSE") << std::endl;

//     std::cerr << "Load Voltage:     " << ups_data.busVoltage  << " V" << std::endl;
//     std::cerr << "Shunt Voltage:    " << ups_data.shuntVoltage  << " mV" << std::endl;
//     std::cerr << "PSU Voltage:      " << (ups_data.busVoltage  + ups_data.shuntVoltage) << " V" << std::endl;
//     std::cerr << "Current:          " << (ups_data.current) << " mA" << std::endl;
//     std::cerr << "Power:            " << ups_data.power << " mW" << std::endl;
//     std::cerr << "Battery Level:    " << ups_data.batteryLevel << "%" << std::endl;

//     std::time_t time = std::chrono::system_clock::to_time_t(ups_data.timestamp);
//     std::tm* tm_now = std::localtime(&time);
//     // Print the formatted time                                              
//     std::cout << "--------------------------------------------" << std::endl;
//     std::cout << "Timestamp: " << std::put_time(tm_now, "%Y-%m-%d %H:%M:%S") << std::endl;
//     std::cout << "--------------------------------------------" << std::endl;
// }



// // void printIMUData(BNO085 &imu)
// {
//     IMUData imu_data = imu.getIMUData();
//     CalibrationStatus cal_status = imu.getCalibrationStatus();

//     std::cout << "============================================" << std::endl;
//     std::cout << "BNO085 IMU DATA" << std::endl;
//     std::cout << "============================================" << std::endl;

//     std::cout << "Last Error: " << imu.getLastError() << std::endl;
//     std::cout << "Data Valid: " << (imu_data.valid ? "TRUE" : "FALSE") << std::endl;
//     std::cout << "Accuracy: " << static_cast<int>(imu_data.accuracy) << "/3" << std::endl;
//     std::cout << "Calibrated: " << (cal_status.is_calibrated ? "TRUE" : "FALSE") << std::endl;
    
//     if (imu_data.valid) {
//         std::cout << std::fixed << std::setprecision(3);
        
//         // Raw Accelerometer (m/s²)
//         std::cout << "Accel (m/s²):  X: " << std::setw(8) << imu_data.accel_x 
//                   << "  Y: " << std::setw(8) << imu_data.accel_y 
//                   << "  Z: " << std::setw(8) << imu_data.accel_z << std::endl;
        
//         // Raw Gyroscope (rad/s)
//         std::cout << "Gyro (rad/s):  X: " << std::setw(8) << imu_data.gyro_x 
//                   << "  Y: " << std::setw(8) << imu_data.gyro_y 
//                   << "  Z: " << std::setw(8) << imu_data.gyro_z << std::endl;
        
//         // Raw Magnetometer (µTesla)
//         std::cout << "Mag (µT):      X: " << std::setw(8) << imu_data.mag_x 
//                   << "  Y: " << std::setw(8) << imu_data.mag_y 
//                   << "  Z: " << std::setw(8) << imu_data.mag_z << std::endl;
        
//         // Linear Acceleration (m/s²)
//         std::cout << "Lin Accel:     X: " << std::setw(8) << imu_data.linear_accel_x 
//                   << "  Y: " << std::setw(8) << imu_data.linear_accel_y 
//                   << "  Z: " << std::setw(8) << imu_data.linear_accel_z << std::endl;
        
//         // Gravity (m/s²)
//         std::cout << "Gravity:       X: " << std::setw(8) << imu_data.gravity_x 
//                   << "  Y: " << std::setw(8) << imu_data.gravity_y 
//                   << "  Z: " << std::setw(8) << imu_data.gravity_z << std::endl;
        
//         // Euler Angles (degrees)
//         std::cout << std::fixed << std::setprecision(1);
//         std::cout << "Euler (deg):   Roll: " << std::setw(7) << imu_data.euler_roll 
//                   << "  Pitch: " << std::setw(7) << imu_data.euler_pitch 
//                   << "  Yaw: " << std::setw(7) << imu_data.euler_yaw << std::endl;
        
//         // Quaternion
//         std::cout << std::fixed << std::setprecision(4);
//         std::cout << "Quaternion:    I: " << std::setw(7) << imu_data.quat_i 
//                   << "  J: " << std::setw(7) << imu_data.quat_j 
//                   << "  K: " << std::setw(7) << imu_data.quat_k 
//                   << "  Real: " << std::setw(7) << imu_data.quat_real << std::endl;
//     } else {
//         std::cout << "No valid IMU data available" << std::endl;
//         std::cout << "Last Error: " << imu.getLastError() << std::endl;
//     }

//     std::time_t time = std::chrono::system_clock::to_time_t(imu_data.timestamp);
//     std::tm* tm_now = std::localtime(&time);
//     std::cout << "Timestamp: " << std::put_time(tm_now, "%Y-%m-%d %H:%M:%S") << std::endl;
//     std::cout << "============================================" << std::endl;
// }


























// void printIMUData(IMU &imu)
// {
//     IMUData raw_data = imu.getSensorData();
//     OrientationData orient_data = imu.getOrientationData();
 
//     std::cout << "--------------------------------------------" << std::endl;
//     // Display Raw IMU Data
//     std::cout << "IMU RAW DATA" << std::endl;
//     std::cout << "Accel (g):  X: " << raw_data.ax << "  Y: " << raw_data.ay << "  Z: " << raw_data.az << std::endl;     // Accelerometer data in g
//     std::cout << "Gyro (dps): X: " << raw_data.gx << "  Y: " << raw_data.gy << "  Z: " << raw_data.gz << std::endl;     // Gyroscope data in degrees per second
//     std::cout << "Mag (gauss): X: " << raw_data.mx << "  Y: " << raw_data.my << "  Z: " << raw_data.mz << std::endl;    // Magnetometer data in gauss
//     // std::cout << "Temperature: " << raw_data.temperature << " °C" << std::endl;                                         // Temperature in Celsius
    
//     // Display Raw IMU Data
//     std::cout << "IMU ORIENTATION DATA" << std::endl;
//     std::cout << "Euler (deg):  Roll: " << orient_data.roll << "  Pitch: " << orient_data.pitch << "  Yaw: " << orient_data.yaw << std::endl;      
//     // std::cout << "Quaternion (deg): W: " << orient_data.qw << " X: " << orient_data.qx << "  Y: " << orient_data.qy << "  Z: " << orient_data.qz << std::endl;                  
                         
//     std::time_t time = std::chrono::system_clock::to_time_t(raw_data.timestamp);
//     std::tm* tm_now = std::localtime(&time);
//     // // Print the formatted time                                              
//     // std::cout << "--------------------------------------------" << std::endl;
//     std::cout << "Timestamp: " << std::put_time(tm_now, "%Y-%m-%d %H:%M:%S") << std::endl;
//     // std::cout << "Valid Data: " ;
//     // if(raw_data.valid)
//     //     std::cout << "TRUE " << std::endl ;
//     // else
//     //     std::cout << "FALSE " << std::endl ;
//     std::cout << "--------------------------------------------" << std::endl;
// }





// int main() 
// {
//     boost::asio::io_service myService;

//     UPS ups; // (threaded)
//     std::this_thread::sleep_for(std::chrono::seconds(1));
//     // IMU imu; // (threaded)
//     // std::this_thread::sleep_for(std::chrono::seconds(1));
    
//     Camera camera; // (threaded)
//     std::this_thread::sleep_for(std::chrono::seconds(1));
//     // Brain::BrainComm brain(myService); // (threaded)
//     // std::this_thread::sleep_for(std::chrono::seconds(1));

//     // RobotPosition robotPosition(brain, imu, myService); // (threaded) 
//     // std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Model model;
//     // ObjectDetection objdet;
//     // std::vector<DetectedObject> Detections;
//     // std::this_thread::sleep_for(std::chrono::seconds(1));
    
//     // //FieldMapper mapper(camera); //, robotPosition); // (threaded)
//     // std::this_thread::sleep_for(std::chrono::seconds(1));

   
//     std::cerr << endl;

//     // imu.calibrateAccelerometer();
    
 


    

//     while (true) 
//     {
//         // std::cerr << "Camera FPS: " << camera.getFPS() << std::endl;
//         // //std::cerr << "Mapper PPS: " << mapper.getPPS() << std::endl;

//         // if(camera.getInferFrame(model.inferInput))
//         // {
//         //     model.runInference();
//         //     Detections = objdet.decodeOutputs(model.inferOutput1, model.inferOutput2);
//         //     std::cerr << "Found " << Detections.size() << " detected objects" << std::endl;;
//         // }
//         // if(imu.isRunning())
//         //     printIMUData(imu);
//         if(ups.isRunning())
//             printUPSdata(ups);
            
//         // if(brain.isConnected() && brain.isRunning())
//         // {
//         //     brain.setJetsonBattery(ups.getBatteryPercentage());
//         // }

//         std::this_thread::sleep_for(std::chrono::milliseconds(250));    
//     }
//     //mapper.stop();
//     // camera.stop();
//     // robotPosition.stop();
//     // brain.stop();
//     // imu.stop();
//     ups.stop();
    
    
//     return 0;
// }


