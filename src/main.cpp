/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       mwedd                                                     */
/*    Created:      4/28/2025, 6:19:12 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <cmath>
#include <robot-config.h>

using namespace vex;

vex::competition Competition;

// A global instance of vex::brain used for printing to the V5 brain screen



/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

// Makes so if button is pressed once, the Tongue mechanism is enabled, but when button is pressed again, it is disabled
void flipTongue(){ Tongue.set(!Tongue.value()); }

// LEAVE THIS!!! IF THE OTHER EQUATIONS WORK, THEN YOU CAN SCRAP THIS!!

    constexpr double gear_ratio = ((double)1/1);
    constexpr double encoder_wheel_radius = 1.375;
    constexpr double encoder_wheel_circumference = 2 * M_PI * encoder_wheel_radius;
    constexpr double start_heading = 90;

    // defines x and y for later
    float x = 0;
    float y = 0;

/*void odometry(double , double ) {

    LeftEncoder.resetPosition();
    RightEncoder.resetPosition();

    float previous_distance_traveled = 0;

    while(1) {
        // Using std::fmod to preserve the "wraparound effect" of 0-360 degrees when considering the offset of start heading.
        float heading = std::fmod((360 - InertialSensor.rotation()) + start_heading, 360); // finds the angle
        float average_encoder_position = (LeftEncoder.position(vex::degrees) + RightEncoder.position(vex::degrees)) / 2; // finds the encoder position on the robot
        float distance_traveled = (average_encoder_position / 360) * encoder_wheel_circumference; // pretty self explanatory
        float change_in_distance = distance_traveled - previous_distance_traveled; // finds the distance traveled

        x += change_in_distance * std::cos(heading * (M_PI / 180));
        y += change_in_distance * std::sin(heading * (M_PI / 180));
        
        // At the end of the loop, reset previous_distance_traveled for the next loop
        previous_distance_traveled = distance_traveled;
        
        vex::this_thread::sleep_for(10);
    }
}*/

/*void Teo_Odometry () {

  int side_encoder_distance; // The distance between the center of the robot and the left/right encoders
  int back_encoder_distance; // The distance between the center of the robot and the back encoder

  // side_encoder_distance = 
  // back_encoder_distance = 

  double Theta = (RightEncoder.angle() - LeftEncoder.angle()) / (2 * side_encoder_distance); // The angle of the arc
  // The radius of the movement in relation to the center of the robot
  double R1 = (side_encoder_distance * (RightEncoder.angle() + LeftEncoder.angle())) / (RightEncoder.angle() - LeftEncoder.angle());
  double R2 = (BackEncoder.angle() / Theta) - back_encoder_distance; // The radius of the movement as read by the back encoder
  double R3 = R1 - R2; // The difference between the two radii (radiuses for dumbies), represents the amount the robot drifted during the turn
  
  // The estimated position of the robot only taking into account the left and right encoders
  double X1 = R1 * (std::cos(Theta) - 1);
  double Y1 = R1 * (std::sin(Theta));
  // The estimated position of the robot taking into account all encoders, this is offset by theta/2
  double X2 = X1 + R3 * std::sin(Theta);
  double Y2 = Y1 + R3 * (1 - std::cos(Theta));
  // The estimated position of the robot fixing the theta/2 offset
  double X3 = (((X2 - X1) * (std::cos(Theta / 2))) - ((Y2 - Y1) * (std::sin(Theta / 2)) + X1));
  double Y3 = (((X2 - X1) * (std::sin(Theta / 2))) + ((Y2 - Y1) * (std::cos(Theta / 2)) + Y1));

  // Maybe reset the rotation sensors if it's necessary (i don't know yet)

  vex::this_thread::sleep_for(10);
} */

void moveToPoint(double ptX, double ptY){
  //define variables
  double xDiff=x-ptX;
  double yDiff=y-ptY;
  double targetAngle;
  double targetDist;

  //get angle to point
  //targetAngle=std::atan(xDiff/yDiff);

  //turn to point
  

  //get distance to point
  //targetDist=std::sqrt(std::pow(xDiff,2)+std::pow(yDiff,2));

  //drive there


  //find arc-distance to point


  std::printf("distance to point: %.3f   angle to point: %.3f rad %.3f deg \n", targetDist, targetAngle, (targetAngle*180/M_PI));


}

void StopDriveTrain (){ // stop motors
  LeftFront.stop ();
  LeftMiddle.stop ();
  LeftRear.stop ();
  RightFront.stop ();
  RightMiddle.stop ();
  RightRear.stop ();
  wait(10,msec);
}

void Drive (int dist, int speed){ // Drive function
  float rotations;
  rotations = 360.*dist/(encoder_wheel_circumference*gear_ratio);
  LeftFront.spinFor (forward, rotations, degrees, speed, velocityUnits::pct, false);
  LeftMiddle.spinFor (forward, rotations, degrees, speed, velocityUnits::pct, false);
  LeftRear.spinFor (forward, rotations, degrees, speed, velocityUnits::pct, false);
  RightFront.spinFor (forward, rotations, degrees, speed, velocityUnits::pct, false);
  RightMiddle.spinFor (forward, rotations, degrees, speed, velocityUnits::pct, false);
  RightRear.spinFor (forward, rotations, degrees, speed, velocityUnits::pct, true);
}

void Turn (int angle){ // Turn function
  int top_speed = 50;
  float speed;
  float sumError = 0;
  float error = 67;
  float Kp = 0.3;
  double Ki = 0.03;

  double startingRot=InertialSensor.rotation();

  while (fabs (error) > 0.5){
    
    error = angle - (InertialSensor.rotation()-startingRot);
    if(fabs(error) < 0.2*angle) sumError += error; // Lists range over which sum is used
    speed = Kp*error + Ki*sumError; // slows down as it approaches destination

    if(speed > top_speed) speed = top_speed; // doesn't get too fast
    if(speed < -top_speed) speed = -top_speed; // doesn't get too slow

    LeftFront.spin (forward, speed, pct); // motors on
    LeftMiddle.spin (forward, speed, pct);
    LeftRear.spin (forward, speed, pct);
    RightFront.spin (forward, -speed, pct);
    RightMiddle.spin (forward, -speed, pct);
    RightRear.spin (forward, -speed, pct);
    wait(20,msec);
  }
  StopDriveTrain(); // stop motors
  // InertialSensor.resetRotation();
}
void TurnToHeading (int angle){ // Turn function
  int top_speed = 50;
  float speed;
  float sumError = 0;
  float error = 67;
  float Kp = 0.3;
  double Ki = 0.03;
  double startingRot=InertialSensor.rotation();

  while (fabs (error) > 0.5){
    error = angle - InertialSensor.rotation();
    printf("Error: %f \n", error);
    if(fabs(error) < 0.2*angle) sumError += error; // Lists range over which sum is used
    speed = Kp*error + Ki*sumError; // slows down as it approaches destination

    if(speed > top_speed) speed = top_speed; // doesn't get too fast
    if(speed < -top_speed) speed = -top_speed; // doesn't get too slow

    LeftFront.spin (forward, speed, pct); // motors on
    LeftMiddle.spin (forward, speed, pct);
    LeftRear.spin (forward, speed, pct);
    RightFront.spin (forward, -speed, pct);
    RightMiddle.spin (forward, -speed, pct);
    RightRear.spin (forward, -speed, pct);
    wait(20,msec);
  }
  StopDriveTrain(); // stop motors
  // InertialSensor.resetRotation();
}

  void pre_auton(void) {

  InertialSensor.calibrate();

  while (InertialSensor.isCalibrating()){
    vex::wait(25,msec);
  }
  
}

  
  /*---------------------------------------------------------------------------*/
  /*                                                                           */
  /*                              Autonomous Task                              */
  /*                                                                           */
  /*  This task is used to control your robot during the autonomous phase of   */
  /*  a VEX Competition.                                                       */
  /*                                                                           */
  /*  You must modify the code to add your own robot specific commands here.   */
  /*---------------------------------------------------------------------------*/
  
void autonomous(void) {

  vex::thread debug_thread([](){
    std::ofstream outFile;
    outFile.open("record-1.txt");

    outFile.close();
    while(1){
      printf("Inertial: %.4f \n", InertialSensor.rotation());
      vex::wait(25,msec);
    }
  });

  /*vex::thread unstuckThread([](){
    float oldMotorCommandMid=0;
    float oldMotorCommandTop=0;
    float oldMotorCommandBot=0;
    while(1){
      if((LowerIntake.torque()>=0.75 && LowerIntake.velocity(rpm)==0)||
        (UpperIntake.torque()>=0.5 && UpperIntake.velocity(rpm)==0)||
        (MiddleIntake.torque()>=0.75 && MiddleIntake.velocity(rpm)==0)){
          UpperIntake.spinFor(reverse,300,degrees,600,rpm,false);
          MiddleIntake.spinFor(reverse,300,degrees,600,rpm,false);
          LowerIntake.spinFor(reverse,100,degrees,50,rpm,true);
          vex::task::sleep(50);
          UpperIntake.spin(forward,oldMotorCommandTop,rpm);
          LowerIntake.spin(forward,oldMotorCommandBot,rpm);
          MiddleIntake.spin(forward,oldMotorCommandMid,rpm);
      }
      else{
        oldMotorCommandMid=MiddleIntake.velocity(rpm);
        oldMotorCommandTop=UpperIntake.velocity(rpm);
        oldMotorCommandBot=LowerIntake.velocity(rpm);
      }
      vex::task::sleep(100);
    }
  });*/

  

  autonSelection="LeftSide";
  
  if(autonSelection=="RightSide"){
  LowerIntake.spin(forward,80,pct);
  MiddleIntake.spin(forward,65,pct);
  UpperIntake.spin(forward,15,pct);
  Drive(30,25);
  wait(50,msec);
  MiddleIntake.stop();
  UpperIntake.stop();
  Turn(-77.65);
  wait(25,msec);
  Tongue.set(true);
  wait(1,sec);
  Drive(11,40);
  wait(25,msec);
  LowerIntake.spin(reverse,20,pct);
  MiddleIntake.spin(reverse,20,pct);
  UpperIntake.spin(reverse,30,pct);
  wait(2,sec);
  LowerIntake.stop();
  MiddleIntake.stop();
  UpperIntake.stop();
  Drive(-45.75,40);
  Tongue.set(false);
  vex::wait(1,sec);
  Turn(-133);
  wait(25,msec);
  Drive(-16,40);
  wait(25,msec);
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,100,pct);
  UpperIntake.spin(forward,100,pct);
  wait(1,sec);
  Tongue.set(true);
  wait(25,msec);
  LowerIntake.spin(forward,90,pct);
  MiddleIntake.spin(forward,65,pct);
  UpperIntake.spin(forward,15,pct);
  Drive(32,40);
  wait(1.5,sec);
  MiddleIntake.stop();
  UpperIntake.stop();
  Drive(-32,40);
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,100,pct);
  UpperIntake.spin(forward,100,pct);
  }
  else if(autonSelection=="LeftSide"){
  LowerIntake.spin(forward,85,pct);
  MiddleIntake.spin(forward,65,pct);
  UpperIntake.spin(forward,10,pct);
  Drive(30,25);
  wait(50,msec);
  Turn(75);
  MiddleIntake.stop();
  UpperIntake.stop();
  wait(25,msec);
  Tongue.set(true);
  wait(1,sec);
  Drive(12,30);
  wait(25,msec);
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,55,pct);
  UpperIntake.spin(reverse,30,pct);
  wait(1.5,sec);
  LowerIntake.stop();
  MiddleIntake.stop();
  UpperIntake.stop();
  Drive(-47,40);
  Tongue.set(false);
  vex::wait(25,msec);
  //TurnToHeading(200);
  Turn(129);
  wait(50,msec);
  //vex::task::sleep(1000);
  Drive(-17,40);
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,100,pct);
  UpperIntake.spin(forward,100,pct);
  wait(1,sec);
  Tongue.set(true);
  wait(25,msec);
  LowerIntake.spin(forward,90,pct);
  MiddleIntake.spin(forward,65,pct);
  UpperIntake.spin(forward,15,pct);
  Drive(22.5,40);
  wait(50,msec);
  // Turn(-2);
  // wait(25,msec);
  Drive(10,40);
  wait(1,sec);
  MiddleIntake.stop();
  UpperIntake.stop();
  Drive(-10,40);
  wait(50,msec);
  // Turn(2);
  // wait(25,msec);
  Drive(-22,40);
  wait(25,msec);
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,100,pct);
  UpperIntake.spin(forward,100,pct);
  }
  else{
  //Skills
      
  }

}
  
  /*---------------------------------------------------------------------------*/
  /*                                                                           */
  /*                              User Control Task                            */
  /*                                                                           */
  /*  This task is used to control your robot during the user control phase of */
  /*  a VEX Competition.                                                       */
  /*                                                                           */
  /*  You must modify the code to add your own robot specific commands here.   */
  /*---------------------------------------------------------------------------*/


  
  void RELEASE () {
  MiddleIntake.stop();
  UpperIntake.stop();
  }
  
  void usercontrol(void) {
    // User control code here, inside the loop
    Controller1.ButtonB.pressed(flipTongue);
    Brain.Screen.clearScreen();

    int J1;
    int J3;
  
    while (1){

      wait (20, msec);

      
  
      // Comms
      J1 = 0.5*Controller1.Axis1.position (percent); //slow down turns
      J3 = Controller1.Axis3.position (percent);
  
      // RC Control
      //if(J3 < 0) J1 = -J1; //flip turn direction when backing up
  
      LeftFront.spin (forward, J3 + J1, pct);
      LeftMiddle.spin (forward, J3 + J1, pct);
      LeftRear.spin (forward, J3 + J1, pct);
      RightFront.spin (forward, J3 - J1, pct);
      RightMiddle.spin (forward, J3 - J1, pct);
      RightRear.spin (forward, J3 - J1, pct);

      // Storage
      if(Controller1.ButtonR1.pressing()){
        LowerIntake.spin(forward,75,pct);
      }

      if(Controller1.ButtonR1.pressing()){
        MiddleIntake.spin(forward,50,pct);
        UpperIntake.spin(forward,50,pct);
      }
      Controller1.ButtonR1.released(RELEASE);

      // Move Blocks Up
      if(Controller1.ButtonL1.pressing()){
        LowerIntake.spin(forward,75,pct);
        MiddleIntake.spin(forward,100,pct);
        UpperIntake.spin(forward,100,pct);
      }

      // Move Blocks Down
      if(Controller1.ButtonR2.pressing()){
        LowerIntake.spin(reverse,25,pct);
        MiddleIntake.spin(reverse,50,pct);
        UpperIntake.spin(reverse,50,pct);
      }

      // Middle Goal
      if(Controller1.ButtonL2.pressing()){
        LowerIntake.spin(forward,50,pct);
        MiddleIntake.spin(forward,75,pct);
        UpperIntake.spin(reverse,85,pct);
      }

      // Un-Middle Goal
      if(Controller1.ButtonDown.pressing()){
        LowerIntake.spin(reverse,50,pct);
        MiddleIntake.spin(reverse,75,pct);
        UpperIntake.spin(forward,85,pct);
      }

      // Unstucky
      if(Controller1.ButtonY.pressing()){
        LowerIntake.spin(reverse,100,pct);
        MiddleIntake.spin(forward,100,pct);
      }

      // Stop All
      if(Controller1.ButtonRight.pressing()){
        LowerIntake.stop();
        MiddleIntake.stop();
        UpperIntake.stop();
      }

      LeftFront.setStopping(brake);
      LeftMiddle.setStopping(brake);
      LeftRear.setStopping(brake);
      RightFront.setStopping(brake);
      RightMiddle.setStopping(brake);
      RightRear.setStopping(brake);
    }
}


int main() {
  if(Brain.SDcard.isInserted()){
    setPortsFromSD();
  }
  Brain.Screen.released(switchScreen);
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  vex::thread touchscreen_thread([](){
      while(1){
        drawAllUi();
      }
  });
  // Run the pre-autonomous function.

  std::ofstream outFile;

  outFile.open("hello-world.txt");

  outFile << 'Hello!';

  outFile.close();




  pre_auton();

    while (1) {
      wait(100, msec);
    }
}
