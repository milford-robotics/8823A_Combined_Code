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
extern digital_out Tongue;

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

void odometry(double , double ) {

    LeftEncoder.resetPosition();
    RightEncoder.resetPosition();

    float previous_distance_traveled = 0;

    while(1) {
        // Using std::fmod to preserve the "wraparound effect" of 0-360 degrees when considering the offset of start heading.
        float heading = std::fmod((360 - InertialSensor()) + start_heading, 360); // finds the angle
        float average_encoder_position = (LeftEncoder.position(vex::degrees) + RightEncoder.position(vex::degrees)) / 2; // finds the encoder position on the robot
        float distance_traveled = (average_encoder_position / 360) * encoder_wheel_circumference; // pretty self explanatory
        float change_in_distance = distance_traveled - previous_distance_traveled; // finds the distance traveled

        x += change_in_distance * std::cos(heading * (M_PI / 180));
        y += change_in_distance * std::sin(heading * (M_PI / 180));
        
        // At the end of the loop, reset previous_distance_traveled for the next loop
        previous_distance_traveled = distance_traveled;
        
        vex::this_thread::sleep_for(10);
    }
}

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
  targetAngle=std::atan(xDiff/yDiff);
  //turn to point
  

  //get distance to point
  targetDist=std::sqrt(std::pow(xDiff,2)+std::pow(yDiff,2));
  //drive there

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
  RightRear.spinFor (forward, rotations, degrees, speed, velocityUnits::pct, false);
}



void Turn (int angle){ // Turn function
  int top_speed = 50;
  float speed;
  float sumError = 0;
  float error = 999;
  float Kp = 0.4;
  double Ki = 0.038;

  InertialSensor1.resetRotation ();
  InertialSensor2.resetRotation ();
  while (fabs (error) > 0.5){
    
    error = angle - InertialSensor();
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
  StopDriveTrain (); // stop motors
  InertialSensor1.resetRotation ();
  InertialSensor2.resetRotation ();
}

  void pre_auton(void) {

  InertialSensor1.calibrate();
  InertialSensor2.calibrate();
  
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

    // Start our odometry thread.
    // The odometry loop will run in the background while we move.

    /*vex::thread Teo_Odometry_thread([](){
      Teo_Odometry(25,45);
    });*/

    vex::thread odometry_thread([](){
      odometry(25,45);
    });

    // Print where we ended up on the coordinate plane onto the brain screen.
    // Brain.Screen.print("(%f, %f)", x, y);

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
  Controller1.ButtonX.pressed(flipTongue);
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
      if(Controller1.ButtonB.pressing()){
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
      if(Controller1.ButtonA.pressing()){
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

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  vex::thread odometry_thread([](){
      odometry(25,45);
    });
  // Run the pre-autonomous function.
  pre_auton();

    while (1) {
      wait(100, msec);
    }
}
