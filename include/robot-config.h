using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LeftBack;
extern motor RightBack;
extern motor RightForward;
extern motor LeftForward;
extern motor RightBackTop;
extern motor LeftBackTop;
extern controller Controller1;
extern inertial Inertial;
extern digital_out Pneumatic1;
extern digital_out Pneumatic2;
extern motor Catapult;
extern motor Intake;
extern digital_out Pneumatic3;
extern pot Potentiometer;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );