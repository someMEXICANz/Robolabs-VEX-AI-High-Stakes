#include <iostream>
#include <random>
#include <chrono>

#include <UPS.h>
#include <IMU.h>
#include <BrainComm.h>
// #include <Position.h>
#include <RobotPosition.h>
#include <Model.h>
#include <ObjectDetection.h>
#include <Camera.h>
#include <FieldMapper.h>
#include <RobotVisualizer.h>


#include <open3d/visualization/gui/Application.h>
#include <open3d/visualization/gui/Window.h>
#include <open3d/visualization/rendering/Scene.h>
#include <open3d/visualization/visualizer/O3DVisualizer.h>

using namespace std;


int main() 
{
            
    

    // UPS ups; // (threaded)
    // IMU imu; // (threaded)
    // boost::asio::io_service myService;
    // Brain::BrainComm brain(myService); // (threaded)
    // RobotPosition robotPosition(brain, imu, myService); // (threaded) 

    Camera camera; // (threaded)

    while(!camera.isInitialized() || !camera.isRunning())
    {
        std::cerr << "Waiting for camera, standy..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    camera.setExtrinsic(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    Model model;
    ObjectDetection objdet;
    
    FieldMapper mapper(camera); //, robotPosition); // (threaded)
    std::vector<DetectedObject> Detections;

    while (true) 
    {

        std::cout << "Camera FPS: " << camera.getFPS() << std::endl;
       
        // if(camera.getInferFrame(model.inferInput))
        // {
        //     model.runInference();
        //     Detections = objdet.decodeOutputs(model.inferOutput1, model.inferOutput2);
        // }
            
        // if(brain.isConnected() && brain.isRunning())
        // {
        //     brain.setJetsonBattery(ups.getBatteryPercentage());
        // }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));    
    }
    
    camera.stop();
    // robotPosition.stop();
    // brain.stop();
    // imu.stop();
    // ups.stop();
    
    
    return 0;
}



void printUPSdata(UPS &ups)
{
    //Get voltage and current measurements
    float shunt_voltage,            // voltage between V+ and V- across the shunt (in V)
          bus_voltage,              // voltage on V- (load side)           
          current,                  // current in mA
          power,                    // power in W
          percentage;               // battery percentage

    ups.getAll(shunt_voltage, bus_voltage, current, power, percentage);

    std::cout << "--------------------------------------------" << std::endl;
    // Display UPS data 
    std::cout << "PSU Voltage:   " << (bus_voltage + shunt_voltage) << " V" << std::endl;
    std::cout << "Load Voltage:  " << bus_voltage << " V" << std::endl;
    std::cout << "Current:       " << (current / 1000.0f) << " A" << std::endl;
    std::cout << "Power:         " << power << " W" << std::endl;
    std::cout << "Percent:       " << percentage << "%" << std::endl;
    std::cout << "--------------------------------------------" << std::endl;
}
