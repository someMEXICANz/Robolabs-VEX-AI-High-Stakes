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

//camera.setExtrinsic(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);



int main() 
{
    //open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);

    boost::asio::io_service myService;

    // UPS ups; // (threaded)
    // IMU imu; // (threaded)

    Camera camera; // (threaded)
    std::this_thread::sleep_for(std::chrono::seconds(1));
    Brain::BrainComm brain(myService); // (threaded)
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // RobotPosition robotPosition(brain, imu, myService); // (threaded) 
    // std::this_thread::sleep_for(std::chrono::seconds(1));

    Model model;
    ObjectDetection objdet;
    std::vector<DetectedObject> Detections;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    FieldMapper mapper(camera); //, robotPosition); // (threaded)
    std::this_thread::sleep_for(std::chrono::seconds(1));

    while (true) 
    {
        std::cerr << "Camera FPS: " << camera.getFPS() << std::endl;
        std::cerr << "Mapper PPS: " << mapper.getPPS() << std::endl;

        if(camera.getInferFrame(model.inferInput))
        {
            model.runInference();
            Detections = objdet.decodeOutputs(model.inferOutput1, model.inferOutput2);
            std::cerr << "Found " << Detections.size() << " detected objects" << std::endl;;
        }
            
        // if(brain.isConnected() && brain.isRunning())
        // {
        //     brain.setJetsonBattery(ups.getBatteryPercentage());
        // }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));    
    }
    //mapper.stop();
    camera.stop();
    // robotPosition.stop();
    brain.stop();
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

    std::cerr << "--------------------------------------------" << std::endl;
    // Display UPS data 
    std::cerr << "PSU Voltage:   " << (bus_voltage + shunt_voltage) << " V" << std::endl;
    std::cerr << "Load Voltage:  " << bus_voltage << " V" << std::endl;
    std::cerr << "Current:       " << (current / 1000.0f) << " A" << std::endl;
    std::cerr << "Power:         " << power << " W" << std::endl;
    std::cerr << "Percent:       " << percentage << "%" << std::endl;
    std::cerr << "--------------------------------------------" << std::endl;
}
