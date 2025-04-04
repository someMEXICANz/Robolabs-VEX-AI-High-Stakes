#include "robot-config.h"

using namespace vex;
using namespace Jetson;

/*----------------------------------------------------------------------------*/
// Create a robot link on a PORT using a unique name.
// The unique name should probably incorporate the team number
// and be at least 12 characters so as to generate a good hash
//
// The Demo is symetrical, we send the same data and display the same status on both
// manager and worker robots
// Comment out the following definition to build for the worker robot
#define  MANAGER_ROBOT    1

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
RobotLink       link( PORT10, "3303X_24", linkType::manager );
#else
#pragma message("building for the worker")
RobotLink       link( PORT10, "3303X_15", linkType::worker );
#endif


FILE *fp = fopen("/dev/serial2","wb");

JetsonComm jetson;
brain  Brain;
// VEXcode device constructors
Position2D LeftOffset = {.x = -.5, .y = 0, .heading = 90};
Position2D RightOffset = {.x = .5, .y = 0, .heading = 270};

gps Left_GPS = gps(PORT14, LeftOffset.x, LeftOffset.y, mm, LeftOffset.heading);
gps Right_GPS = gps(PORT15, RightOffset.x, RightOffset.y, mm, RightOffset.heading);

controller Controller1 = controller(primary);
motor leftMotorA = motor(PORT1, ratio6_1, true);
motor leftMotorB = motor(PORT2, ratio6_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT3, ratio6_1, false);
motor rightMotorB = motor(PORT4, ratio6_1, false);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
inertial DrivetrainInertial = inertial(PORT5);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial, 299.24, 320, 40, mm, 0.6);



// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

void requestNothing()
{
  jetson.updateRequests(static_cast<uint16_t>(RequestFlag::NoData));
  wait(250,msec);
}

void requestVoltages()
{
  jetson.updateRequests(static_cast<uint16_t>(RequestFlag::MotorVoltages));
  wait(250,msec);

}

void requestMacro()
{
  jetson.updateRequests(static_cast<uint16_t>(RequestFlag::MacroControls));
  wait(250,msec);

}

void requestAll()
{
  jetson.updateRequests(static_cast<uint16_t>(RequestFlag::MacroControls) | static_cast<uint16_t>(RequestFlag::MotorVoltages) | static_cast<uint16_t>(RequestFlag::BatteryLevel));
  wait(250,msec);

}

// void initiateComms()
// {
//   jetson.initCommunication();
//   wait(250,msec);
// }


void requestLvl()
{

  jetson.updateRequests(static_cast<uint16_t>(RequestFlag::BatteryLevel));
  wait(250,msec);
}








int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3 + Axis1
      // right = Axis3 - Axis1
      int drivetrainLeftSideSpeed = Controller1.Axis3.position() + Controller1.Axis1.position();
      int drivetrainRightSideSpeed = Controller1.Axis3.position() - Controller1.Axis1.position();


     
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(3, 1);
  wait(200, msec);
  DrivetrainInertial.calibrate();
  Brain.Screen.print("Calibrating Inertial for Drivetrain");
  // wait for the Inertial calibration process to finish
  while (DrivetrainInertial.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
  wait(50, msec);
  Brain.Screen.clearScreen();
}