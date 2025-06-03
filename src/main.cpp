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

using namespace vex;

vex::competition Competition;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;
controller Controller1 = controller(primary);
motor LeftFront = motor(PORT11, ratio6_1, true);
motor LeftMiddle = motor(PORT12, ratio6_1, true);
motor LeftRear = motor(PORT13, ratio6_1, true);
motor RightFront = motor(PORT18, ratio6_1, false);
motor RightMiddle = motor(PORT19, ratio6_1, false);
motor RightRear = motor(PORT20, ratio6_1, false);
motor UpperIntake = motor(PORT10, ratio6_1, false);
motor MiddleIntake = motor(PORT1, ratio6_1, true);
motor LowerIntakeL = motor(PORT14, ratio6_1, false);
motor LowerIntakeR = motor(PORT17, ratio6_1, true);
rotation LeftEncoder = rotation(PORT8, true);
rotation RightEncoder = rotation(PORT9, true);
inertial InertialSensor = inertial(PORT15);


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/


    constexpr double gear_ratio = ((double)1/1);
    constexpr double encoder_wheel_radius = 1.375;
    constexpr double encoder_wheel_circumference = 2 * M_PI * encoder_wheel_radius;
    constexpr double start_heading = 90;

    // defines x and y for later
    double x = 0;
    double y = 0;

void odometry(double distance_traveled, double heading) {

    LeftFront.resetPosition();
    LeftMiddle.resetPosition();
    LeftRear.resetPosition();
    RightFront.resetPosition();
    RightMiddle.resetPosition();
    RightRear.resetPosition();

    double previous_distance_traveled = 0;

    while(1) {
        // Using std::fmod to preserve the "wraparound effect" of 0-360 degrees when considering the offset of start heading.
        double heading = std::fmod((360 - InertialSensor.heading(vex::degrees)) + start_heading, 360); // finds the angle
        double average_encoder_position = (LeftEncoder.position(vex::degrees) + RightEncoder.position(vex::degrees)) / 2; // finds the encoder position on the robot
        double distance_traveled = (average_encoder_position / 360) * encoder_wheel_circumference; // pretty self explanatory
        double change_in_distance = distance_traveled - previous_distance_traveled; // finds the distance traveled

        x += change_in_distance * std::cos(heading * (M_PI / 180));
        y += change_in_distance * std::sin(heading * (M_PI / 180));
        
        // At the end of the loop, reset previous_distance_traveled for the next loop
        previous_distance_traveled = distance_traveled;
        
        vex::this_thread::sleep_for(10);
    }
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
  LeftFront.spinFor (forward,rotations, degrees, speed, velocityUnits::pct, false);
  LeftMiddle.spinFor (forward,rotations, degrees, speed, velocityUnits::pct, false);
  LeftRear.spinFor (forward,rotations, degrees, speed, velocityUnits::pct, false);
  RightFront.spinFor (forward,rotations, degrees, speed, velocityUnits::pct, false);
  RightMiddle.spinFor (forward,rotations, degrees, speed, velocityUnits::pct, false);
  RightRear.spinFor (forward,rotations, degrees, speed, velocityUnits::pct, false);
}

void Turn (int angle){ // Turn function
  int top_speed = 50;
  float speed;
  float sumError = 0;
  float error = 999;
  float Kp = 0.4;
  double Ki = 0.038;

  InertialSensor.resetRotation ();
  while (fabs (error) > 0.5){
    
    error = angle - InertialSensor.rotation (degrees);
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
  InertialSensor.resetRotation ();
}

  void pre_auton(void) {

  InertialSensor.calibrate();
  
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
    vex::thread odometry_thread([](){
      odometry(25,45);
    });

    // Print where we ended up on the coordinate plane onto the brain screen.
    Brain.Screen.print("(%f, %f)", x, y);

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
  
  void usercontrol(void) {
    // User control code here, inside the loop
  
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
        LowerIntakeR.spin(forward,50,pct);
        LowerIntakeL.spin(forward,50,pct);
        MiddleIntake.spin(forward,35,pct);
        UpperIntake.spin(forward,5,pct);
        // wait(1,sec);
        // MiddleIntake.stop();
        // UpperIntake.stop();
        // wait(1,sec);
      }

      // Move Blocks Up
      if(Controller1.ButtonL1.pressing()){
        LowerIntakeR.spin(forward,50,pct);
        LowerIntakeL.spin(forward,50,pct);
        MiddleIntake.spin(forward,100,pct);
        UpperIntake.spin(forward,100,pct);
      }

      // Move Blocks Down
      if(Controller1.ButtonR2.pressing()){
        LowerIntakeR.spin(reverse,50,pct);
        LowerIntakeL.spin(reverse,50,pct);
        MiddleIntake.spin(reverse,100,pct);
        UpperIntake.spin(reverse,100,pct);
      }

      // Middle Goal
        if(Controller1.ButtonL2.pressing()){
        LowerIntakeR.spin(forward,50,pct);
        LowerIntakeL.spin(forward,50,pct);
        MiddleIntake.spin(forward,65,pct);
        UpperIntake.spin(reverse,75,pct);
      }

      // Stop All
      if(Controller1.ButtonA.pressing()){
        LowerIntakeR.stop();
        LowerIntakeL.stop();
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

  // Run the pre-autonomous function.
  pre_auton();

    while (1) {
      wait(100, msec);
    }
}
