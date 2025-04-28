#include <iostream>
#include <random>
#include <chrono>

#include <UPS.h>
#include <IMU.h>
#include <BrainComm.h>
#include <RobotPosition.h>
#include <Model.h>
#include <ObjectDetection.h>
#include <Camera.h>
#include <FieldMapper.h>



using namespace std;

void printIMUData(IMU &imu)
{
    IMUData raw_data = imu.getSensorData();
    OrientationData orient_data = imu.getOrientationData();
 
    std::cout << "--------------------------------------------" << std::endl;
    // Display Raw IMU Data
    std::cout << "IMU RAW DATA" << std::endl;
    std::cout << "Accel (g):  X: " << raw_data.ax << "  Y: " << raw_data.ay << "  Z: " << raw_data.az << std::endl;     // Accelerometer data in g
    std::cout << "Gyro (dps): X: " << raw_data.gx << "  Y: " << raw_data.gy << "  Z: " << raw_data.gz << std::endl;     // Gyroscope data in degrees per second
    std::cout << "Mag (gauss): X: " << raw_data.mx << "  Y: " << raw_data.my << "  Z: " << raw_data.mz << std::endl;    // Magnetometer data in gauss
    std::cout << "Temperature: " << raw_data.temperature << " °C" << std::endl;                                         // Temperature in Celsius
    
    // Display Raw IMU Data
    std::cout << "IMU ORIENTATION DATA" << std::endl;
    std::cout << "Euler (deg):  Roll: " << orient_data.roll << "  Pitch: " << orient_data.pitch << "  Yaw: " << orient_data.yaw << std::endl;      
    std::cout << "Quaternion (deg): W: " << orient_data.qw << " X: " << orient_data.qx << "  Y: " << orient_data.qy << "  Z: " << orient_data.qz << std::endl;                  
                         
    std::time_t time = std::chrono::system_clock::to_time_t(raw_data.timestamp);
    std::tm* tm_now = std::localtime(&time);
    // Print the formatted time                                              
    std::cout << "--------------------------------------------" << std::endl;
    std::cout << "Timestamp: " << std::put_time(tm_now, "%Y-%m-%d %H:%M:%S") << std::endl;
    std::cout << "Valid Data: " ;
    if(raw_data.valid)
        std::cout << "TRUE " << std::endl ;
    else
        std::cout << "FALSE " << std::endl ;
    std::cout << "--------------------------------------------" << std::endl;
}


void printUPSdata(UPS &ups)
{
    UPSData ups_data = ups.getUPSData();

    std::cerr << "--------------------------------------------" << std::endl;
    // Display UPS data 
    std::cout << "WAVESHARE UPS DATA" << std::endl;
    std::cerr << "PSU Voltage:   " << (ups_data.busVoltage  + ups_data.shuntVoltage) << " V" << std::endl;
    std::cerr << "Load Voltage:  " << ups_data.busVoltage  << " V" << std::endl;
    std::cerr << "Current:       " << (ups_data.current / 1000.0f) << " A" << std::endl;
    std::cerr << "Power:         " << ups_data.power << " W" << std::endl;
    std::cerr << "Percent:       " << ups_data.batteryLevel << "%" << std::endl;

    std::time_t time = std::chrono::system_clock::to_time_t(ups_data.timestamp);
    std::tm* tm_now = std::localtime(&time);
    // Print the formatted time                                              
    std::cout << "--------------------------------------------" << std::endl;
    std::cout << "Timestamp: " << std::put_time(tm_now, "%Y-%m-%d %H:%M:%S") << std::endl;
    std::cout << "Valid Data: " ;
    if(ups_data.valid)
        std::cout << "TRUE " << std::endl ;
    else
        std::cout << "FALSE " << std::endl ;
    std::cout << "--------------------------------------------" << std::endl;
}


int main() 
{
    boost::asio::io_service myService;

    UPS ups; // (threaded)
    IMU imu; // (threaded)
    

    // Camera camera; // (threaded)
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // Brain::BrainComm brain(myService); // (threaded)
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // // RobotPosition robotPosition(brain, imu, myService); // (threaded) 
    // // std::this_thread::sleep_for(std::chrono::seconds(1));

    // Model model;
    // ObjectDetection objdet;
    // std::vector<DetectedObject> Detections;
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // //FieldMapper mapper(camera); //, robotPosition); // (threaded)
    // std::this_thread::sleep_for(std::chrono::seconds(1));

    while (true) 
    {
        // std::cerr << "Camera FPS: " << camera.getFPS() << std::endl;
        // //std::cerr << "Mapper PPS: " << mapper.getPPS() << std::endl;

        // if(camera.getInferFrame(model.inferInput))
        // {
        //     model.runInference();
        //     Detections = objdet.decodeOutputs(model.inferOutput1, model.inferOutput2);
        //     std::cerr << "Found " << Detections.size() << " detected objects" << std::endl;;
        // }
        if(imu.isRunning())
            printIMUData(imu);
        if(ups.isRunning())
            printUPSdata(ups);
            
        // if(brain.isConnected() && brain.isRunning())
        // {
        //     brain.setJetsonBattery(ups.getBatteryPercentage());
        // }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));    
    }
    //mapper.stop();
    // camera.stop();
    // robotPosition.stop();
    // brain.stop();
    imu.stop();
    ups.stop();
    
    
    return 0;
}








