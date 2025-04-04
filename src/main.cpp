#include <Camera.h>
#include <Model.h>
#include <ObjectDetection.h>
#include <GPS.h>
#include <PortDetector.h>
#include <IMU.h>
#include <iostream>
#include <BrainComm.h>
#include <RobotPosition.h>
#include <Position.h>
#include <UPS.h>
#include <random>
#include <chrono>
#include <FieldMapper.h>

using namespace std;


// void printUPSdata(UPS &ups)
// {
//     //Get voltage and current measurements
//     float bus_voltage = ups.getBusVoltage_V();              // voltage on V- (load side)
//     float shunt_voltage = ups.getShuntVoltage_mV() / 1000;  // voltage between V+ and V- across the shunt (in V)
//     float current = ups.getCurrent_mA();                    // current in mA
//     float power = ups.getPower_W();                         // power in W
//     float percentage = ups.getBatteryPercentage();          // battery percentage


//      std::cout << "--------------------------------------------" << std::endl;
//     // Display UPS data 
//     std::cout << "PSU Voltage:   " << (bus_voltage + shunt_voltage) << " V" << std::endl;
//     std::cout << "Load Voltage:  " << bus_voltage << " V" << std::endl;
//     std::cout << "Current:       " << (current / 1000.0f) << " A" << std::endl;
//     std::cout << "Power:         " << power << " W" << std::endl;
//     std::cout << "Percent:       " << percentage << "%" << std::endl;
//     std::cout << "--------------------------------------------" << std::endl;
// }




int main() {
    boost::asio::io_service myService;

    UPS ups;
    IMU imu;
    Brain::BrainComm brain(myService);
    RobotPosition robotPosition(brain, imu, myService);
   

    Model model;
    ObjectDetection objdet;

    Camera camera;

    FieldMapper mapper(camera, robotPosition);


    while (true) 
    {
        if(camera.isConnected() && camera.isRunning())
        {
            camera.preprocessFrames(model.inferInput);
            model.runInference();
            std::vector<DetectedObject> Det = objdet.decodeOutputs(model.inferOutput1, model.inferOutput2);
        }
        if(brain.isConnected() && brain.isRunning())
        {
            brain.setJetsonBattery(ups.getBatteryPercentage());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));    
    }
    camera.stop();
    brain.stop();
    imu.stop();
    
    
    return 0;
}




