#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LeftBack = motor(PORT13, ratio6_1, true);
motor RightBack = motor(PORT8, ratio6_1, false);
motor RightForward = motor(PORT10, ratio6_1, false);
motor LeftForward = motor(PORT11, ratio6_1, true);
motor RightBackTop = motor(PORT9, ratio6_1, true);
motor LeftBackTop = motor(PORT12, ratio6_1, false);
controller Controller1 = controller(primary);
inertial Inertial = inertial(PORT3);
digital_out Pneumatic1 = digital_out(Brain.ThreeWirePort.A);
digital_out Pneumatic2 = digital_out(Brain.ThreeWirePort.F);
motor Catapult = motor(PORT20, ratio36_1, false);
motor Intake = motor(PORT1, ratio6_1, false);
digital_out Pneumatic3 = digital_out(Brain.ThreeWirePort.C);
pot Potentiometer = pot(Brain.ThreeWirePort.B);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}