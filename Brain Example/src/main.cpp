#include "robot-config.h"

using namespace vex;

// A global instance of competition
competition Competition;


void pre_auton(void) 
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // initiateComms();
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}



void usercontrol(void) {

  // MotorCommand Control;
  Controller1.ButtonY.pressed(requestVoltages);
  Controller1.ButtonB.pressed(requestMacro);
  Controller1.ButtonX.pressed(requestLvl);

  Controller1.ButtonUp.pressed(requestAll);
  Controller1.ButtonDown.pressed(requestNothing);
  



  while (1) 
  {

   

    // Control = jetson.getMotorCommand();
    // LeftDriveSmart.spin(fwd,Control.left_voltage,vex::voltageUnits::volt); 
    // RightDriveSmart.spin(reverse,Control.right_voltage,vex::voltageUnits::volt); 

    wait(100, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  thread t1(dashboardTask);

  // Run the pre-autonomous function.
  pre_auton();
  uint32_t jetsonbatt = 0;


    while (true) 
    {

      // jetsonbatt = 0;
      // jetsonbatt = jetson.getJetsonBattery();
      // if (jetsonbatt != 0)
      //   fprintf(fp, "Jetson Battery: %lu  percent\n\r" ,jetson.getJetsonBattery());
      
 
      // if(jetson.isTransmitting())
      // {
      //   fprintf(fp, "Brain is sending Data\n\r");
      // }
      // else
      // {
      //   fprintf(fp, "Brain is not sending Data\n\r");
      // }
 


    wait(100, msec);
    

  }
}
