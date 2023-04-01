/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftBack             motor         13              
// RightBack            motor         8               
// RightForward         motor         10              
// LeftForward          motor         11              
// RightBackTop         motor         9               
// LeftBackTop          motor         12              
// Controller1          controller                    
// Inertial             inertial      3               
// Pneumatic1           digital_out   A               
// Pneumatic2           digital_out   F               
// Catapult             motor         20              
// Intake               motor         1               
// Pneumatic3           digital_out   C               
// Potentiometer        pot           B               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>

using namespace vex;

// A global instance of competition
competition Competition;

// Drive
bool defaultDrive = true;
int t = 16;
float driveMultiplier = 1;


float whdia= 3.25;
float whcir= M_PI* whdia;
float in_to_deg=360.0/whcir;
//Controller 
int J2 = 0;
int J3 = 0;
int L2 = 0;
int L1 = 0;
int Bdown;

// Drive Curve
bool alreadydown = false;
bool expand = false;

// PID Values
// --Catapult--
float kP = 1.5;
float kI = 0.2;
float kD = 1.2;

//--Catapult--
double cata_error;
float cata_integral;
double cata_prevError;
float cata_derivative;
long int powercata;
double cata_target;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  RightBack.setStopping(coast);
  RightBackTop.setStopping(coast);
  RightForward.setStopping(coast);

  LeftBack.setStopping(coast);
  LeftBackTop.setStopping(coast);
  LeftForward.setStopping(coast);

  Catapult.setStopping(hold);
  Inertial.calibrate();
}
// Auton Functions
int catarotate(){
  while(Catapult.isSpinning()){
    task::sleep(20);
  }
  while(true){
    cata_error = (cata_target - Potentiometer.angle(degrees));
    if (fabs(cata_error) < 5 && cata_error != 0){
     cata_integral += cata_error;
    }
    else{
      cata_integral = 0;
    }

    cata_derivative = cata_error - cata_prevError;
    cata_prevError = cata_error; 
    powercata = cata_error * kP + cata_integral * kI + cata_derivative * kD;
    Catapult.spin(forward,powercata,voltageUnits::volt);
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print(Potentiometer.angle(degrees));
    Brain.Screen.setCursor(2,2);
    Brain.Screen.print(cata_error);
    task::sleep(20);
    if(fabs(cata_error) < 10){
      break;
    }
  }
  Catapult.stop(hold);
  return 1;
}
void Pneumaticshoot(){
  Pneumatic1.set(true);
  Pneumatic2.set(true);
  Pneumatic3.set(true);
}
void SpinRoller(){
  Intake.rotateFor(-980,rotationUnits::deg,100,velocityUnits::pct);
}
void StopDriveTrain(){
  LeftBack.stop();
  RightBackTop.stop ();
  RightBack.stop ();
  LeftBackTop.stop ();
  RightForward.stop();
  LeftForward.stop();
}
void ResetDrive(){
  LeftBack.setPosition(0,degrees);
  RightBack.setPosition(0,degrees);
  LeftBackTop.setPosition(0,degrees);
  RightBackTop.setPosition(0,degrees);
  LeftForward.setPosition(0,degrees);
  RightForward.setPosition(0,degrees);
  Inertial.setRotation(0,degrees);
}
void TurnPID(double gyroRequest, double maxspeed, int TimeExit){// + spin right, - spin left
  float gyrocurrent;
  float gyroerror;
  float gyrodrive;
  float lastgyroError;
  float gyroP;
  float gyroD;
  
  const float gyro_kP = 0.573;
  const float gyro_kI = 0.4;
  const float gyro_kD = 0.18;

  double Threshold = 1.5;

  while(1){
    gyrocurrent = Inertial.rotation();

    Brain.Screen.setCursor(3,1);
    // calculate error
    gyroerror = gyroRequest - gyrocurrent;
    // Exit Loop
    if(gyroerror < Threshold && gyroerror > -Threshold){
      break;
    }
    else if(TimeExit == 10000){
      StopDriveTrain();
      ResetDrive();
      break;
    }
    else{
      TimeExit = 0;
    }
    
    // drivePID
    gyroP = (gyro_kP * gyroerror);
    static float gyrol = 0;
    gyrol += gyroerror * gyro_kI;
    if(gyrol > 1){
      gyrol = 1;
    }
    else if (gyrol < -1){
      gyrol = -1;
    }
    gyroD = (gyroerror - lastgyroError) * gyro_kD;
    gyrodrive = gyroP + gyrol + gyroD;

    if(gyrodrive > maxspeed){
      gyrodrive = maxspeed;
    }
    else if (gyrodrive < -maxspeed){
      gyrodrive = -maxspeed;
    }
    int powerValue = gyrodrive;

    RightBack.spin(reverse, powerValue, velocityUnits::pct);
    LeftBack.spin(forward, powerValue, velocityUnits::pct);
    LeftBackTop.spin(forward, powerValue, velocityUnits::pct);
    RightBackTop.spin(reverse, powerValue, velocityUnits::pct);
    LeftForward.spin(forward, powerValue, velocityUnits::pct);
    RightForward.spin(reverse, powerValue, velocityUnits::pct);
    
    //Update Value
    lastgyroError = gyroerror;
    wait(50,msec);
  }
  ResetDrive();
  StopDriveTrain();
}
void driveStraight(int dist){
 double distkP = 5;
 double distkI = 0.05;
 double distkD = 0.025;

 double diffkP = 2;
 double diffkI = 0;
 double diffkD = 0;
 

 double distSpeed;
 double diffSpeed;
 
 double distError; 
 double diffError;
 
 double prevDistError;
 double prevDiffError;
 
 double diffIntegral;
 double distIntegral;
 
 double distDerivative; 
 double diffDerivative;

  RightBack.setPosition(0,degrees);
  LeftBack.setPosition(0,degrees);
  LeftBackTop.setPosition(0,degrees);
  RightBackTop.setPosition(0,degrees);
  LeftForward.setPosition(0,degrees);
  RightForward.setPosition(0,degrees);
 
  while (distError != -1){
    Brain.Screen.printAt( 10, 50, "distError %6.2f", distError);
    Brain.Screen.printAt( 10, 80, "diffError %6.2f", diffError);
    double leftFEncoder = (LeftForward.position(degrees) / 360) * 31.92;
    double rightFEncoder = (RightForward.position(degrees) / 360) * 31.92;
    double leftREncoder = (LeftBack.position(degrees) / 360) * 31.92;
    double rightREncoder = (RightBack.position(degrees) / 360) * 31.92;
    double leftRTopEncoder = (LeftBackTop.position(degrees) / 360) * 31.92;
    double rightRTopEncoder = (RightBackTop.position(degrees) / 360) * 31.92;
    double leftEncoder = (leftFEncoder + leftREncoder + leftRTopEncoder) /3;
    double rightEncoder = (rightFEncoder + rightREncoder + rightRTopEncoder) /3;


    distError = dist - ((leftREncoder + rightREncoder + leftFEncoder + rightFEncoder + leftRTopEncoder + rightRTopEncoder)/6); //Calculate distance error
    diffError = leftEncoder - rightEncoder; //Calculate difference error
 
    // Find the integral ONLY if within controllable range AND if the distance error is not equal to zero
    if( std::abs(distError) < 2 && distError != 0)
    {
      distIntegral = distIntegral + distError;
    }
    else
    {
      distIntegral = 0; //Otherwise, reset the integral
    }
 
    // Find the integral ONLY if within controllable range AND if the difference error is not equal to zero
    if( std::abs(diffError) < 60 && diffError != 0)
    {
      diffIntegral = diffIntegral + diffError;
    }
    else
    {
      diffIntegral = 0; //Otherwise, reset the integral
    }
 
    distDerivative = distError - prevDistError; //Calculate distance derivative
    diffDerivative = diffError - prevDiffError; //Calculate difference derivative
 
    prevDistError = distError; //Update previous distance error
    prevDiffError = diffError; //Update previous difference error
 
    distSpeed = (distError * distkP)  + (distIntegral * distkI) + (distDerivative * distkD); //Calculate distance speed
    diffSpeed = (diffError * diffkP)  + (distIntegral * distkI)+ (diffDerivative* diffkD); //Calculate difference (turn) speed
 

    
    LeftBack.spin(forward, distSpeed - diffSpeed, voltageUnits::volt); //Set motor values
    RightBack.spin(forward, distSpeed + diffSpeed, voltageUnits::volt);
    LeftBackTop.spin(forward, distSpeed - diffSpeed, voltageUnits::volt); //Set motor values
    RightBackTop.spin(forward, distSpeed + diffSpeed, voltageUnits::volt); //Set motor values
    LeftForward.spin(forward, distSpeed - diffSpeed, voltageUnits::volt);
    RightForward.spin(forward, distSpeed + diffSpeed, voltageUnits::volt); //Set motor values
    
      {
    }
    if(distError < 1 && distError > -1){
      StopDriveTrain();
      break;
      }
  }
}
void Shoot(){
 Catapult.rotateFor(-2000,rotationUnits::deg,100,velocityUnits::pct);
}
void intakespin(){
  Intake.spin(forward,100,pct);
}
void intakestop(){
  Intake.stop();
}
void Driver(){

}
void NonDriver(){

}
void DoubleRoller(){

}
void Skills(){

}
void SpinRoller_Shoot(){

}
void autonomous(void) {
  while(Inertial.isCalibrating()){};// comes back false when done
  TurnPID(90,100,0);
  wait(2,sec);
  TurnPID(-90,100,0);
  //Driver();
  //NonDriver();
  //DoubleRoller();
  //Skills();
  //SpinRoller_Shot();
}


//User Control Functions
void cata(){
  if(alreadydown == true){
  Catapult.startRotateFor(-70,rotationUnits::deg,100,velocityUnits::pct);
  }
  cata_target = 159; // target number
  vex::task catapultPID(catarotate);
  alreadydown = true;
}
void release(){
  if (expand == true){
  Pneumatic1.set(true);
  Pneumatic2.set(true);
  Pneumatic3.set(true);
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("normal release");
  }
}
void spin_opposite_catapult(){
  Catapult.startRotateFor(200,rotationUnits::deg,100,velocityUnits::pct);
}
int controls(){
  while(1){
  J2 = Controller1.Axis2.position(pct);
  J3 = Controller1.Axis3.position(pct);

  Controller1.ButtonR1.pressed(cata);
  Controller1.ButtonR2.pressed(release);
  L1 = Controller1.ButtonL1.pressing();
  L2 = Controller1.ButtonL2.pressing();
  Controller1.ButtonDown.pressed(spin_opposite_catapult);
  }
  return 1;
}
float getDriveOutput(int x) {
   return (exp(-t/10.0) + exp((abs(x) - 100)/10.0)) * (1 - exp(-t/10.0)) * x;
  }
void driveFunction() {
  // Right Group
  RightBack.spin(fwd,defaultDrive? getDriveOutput(driveMultiplier*J2):-getDriveOutput(driveMultiplier*J3),pct);
  RightBackTop.spin(fwd,defaultDrive? getDriveOutput(driveMultiplier*J2):-getDriveOutput(driveMultiplier*J3),pct);
  RightForward.spin(fwd,defaultDrive? getDriveOutput(driveMultiplier*J2):-getDriveOutput(driveMultiplier*J3),pct);
  //Left Group
  LeftBack.spin(fwd,defaultDrive? getDriveOutput(driveMultiplier*J3):-getDriveOutput(driveMultiplier*J2),pct);
  LeftBackTop.spin(fwd,defaultDrive? getDriveOutput(driveMultiplier*J3):-getDriveOutput(driveMultiplier*J2),pct);
  LeftForward.spin(fwd,defaultDrive? getDriveOutput(driveMultiplier*J3):-getDriveOutput(driveMultiplier*J2),pct);
}

int expansionTimeLimit = 90000; // one min 30 seconds
int timerVal;

//Controller 
void usercontrol(void) {
  Brain.Timer.clear(); // Resets the Time
  while (1) {
  timerVal = Brain.Timer.time(msec); 
  if(Controller1.ButtonUp.pressing() && Controller1.ButtonX.pressing()){
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("failsafe activate!");
    Pneumatic1.set(true);
    Pneumatic2.set(true);
    Pneumatic3.set(true);
  }
  if (timerVal >= expansionTimeLimit){
    Controller1.Screen.clearScreen();
    Controller1.rumble(rumbleShort);
    expand = true;
  }
  // Update Controller Values 
  vex::task drivecontrol(controls);
  // Drive Controls
  driveFunction();
  // Intake && Roller
  if (L1){
    Intake.spin(forward,100,pct);
  }
  else if (L2){
    Intake.spin(reverse,100,pct);;
  }
  else{
    Intake.stop();
  }
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
