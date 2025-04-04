#include "vex.h"

using namespace vex;
using namespace Jetson;

extern brain Brain;


// VEXcode devices

// extern smartdrive Drivetrain;
extern controller Controller1;
extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;

extern Position2D LeftOffset;
extern Position2D RightOffset;

extern gps Left_GPS;
extern gps Right_GPS;

extern RobotLink link;
extern JetsonComm jetson;
extern FILE *fp;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void );
void requestNothing();
void requestVoltages();
void requestMacro();
void requestLvl();
void requestAll();

// void initiateComms();
