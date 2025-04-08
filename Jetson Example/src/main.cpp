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

using namespace std;

int main() {
            
            
    Camera camera; // (threaded)

    boost::asio::io_service myService;

    // UPS ups; // (threaded)
    IMU imu; // (threaded)
    Brain::BrainComm brain(myService); // (threaded)
    RobotPosition robotPosition(brain, imu, myService); // (threaded) 
   
    Model model;
    ObjectDetection objdet;

    FieldMapper mapper(camera, robotPosition); // (threaded)


    while (true) 
    {

        
        //std::cout << "Camera is connected and running" << std::endl;
        camera.preprocessFrames(model.inferInput);
        model.runInference();
        std::vector<DetectedObject> Det = objdet.decodeOutputs(model.inferOutput1, model.inferOutput2);
        camera.displayDetections(Det);

        if (cv::waitKey(1) == 27)
        {
            break;
        }
      
    
    
        // if(brain.isConnected() && brain.isRunning())
        // {
        //     brain.setJetsonBattery(ups.getBatteryPercentage());
        // }

        //std::this_thread::sleep_for(std::chrono::milliseconds(100));    
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



