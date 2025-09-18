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

#define hawktuah 1

// A global instance of vex::brain used for printing to the V5 brain screen

    constexpr double gear_ratio = ((double)1/1);
    constexpr double encoder_wheel_radius = 1.375;
    constexpr double encoder_wheel_circumference = 2 * M_PI * encoder_wheel_radius;
    constexpr double start_heading = 90;


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


enum sc{
  blue,
  red
};
sc wrongColor= sc::blue;

void flipSpitColor(){
  if(wrongColor==sc::blue){
    wrongColor=sc::red;
  }
  else if(wrongColor==sc::red){
    wrongColor=sc::blue;
  }
}

bool colorSort=false;


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
  double Ki = 0.031;

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
  double Ki = 0.04;
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
    while(1){
      printf("%.4f \n",InertialSensor.rotation());
      vex::task::sleep(50);
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
  //autonSelection="LeftSide";    
  if(autonSelection=="RightSide"){
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,75,pct);
  UpperIntake.spin(forward,15,pct);
  Drive(28,25);
  wait(50,msec);
  MiddleIntake.stop();
  UpperIntake.stop();
  Turn(-75);
  wait(25,msec);
  Drive(13,30);
  wait(25,msec);
  LowerIntake.spin(reverse,65,pct);
  MiddleIntake.spin(reverse,55,pct);
  UpperIntake.spin(reverse,30,pct);
  wait(1.5,sec);
  LowerIntake.stop();
  MiddleIntake.stop();
  UpperIntake.stop();
  Drive(-47,40);
  vex::wait(25,msec);
  TurnToHeading(-200);
  vex::task::sleep(75);
  // Turn(131);
  // wait(25,msec);
  Drive(-18,30);
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,100,pct);
  UpperIntake.spin(forward,100,pct);
  wait(1,sec);
  Tongue.set(true);
  wait(25,msec);
  LowerIntake.spin(forward,90,pct);
  MiddleIntake.spin(forward,65,pct);
  UpperIntake.spin(forward,15,pct);
  Drive(30,50);
  wait(2,sec);
  MiddleIntake.stop();
  UpperIntake.stop();
  Drive(-32,30);
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,100,pct);
  UpperIntake.spin(forward,100,pct);
  }
  else if(autonSelection=="LeftSide"){
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,75,pct);
  UpperIntake.spin(forward,15,pct);
  Drive(28,25);
  wait(50,msec);
  MiddleIntake.stop();
  UpperIntake.stop();
  Turn(73);
  wait(25,msec);
  Tongue.set(true);
  wait(1,sec);
  Drive(16,30);
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
  TurnToHeading(200);
  vex::task::sleep(75);
  // Turn(131);
  // wait(25,msec);
  Drive(-18,30);
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,100,pct);
  UpperIntake.spin(forward,100,pct);
  wait(1,sec);
  Tongue.set(true);
  wait(25,msec);
  LowerIntake.spin(forward,90,pct);
  MiddleIntake.spin(forward,65,pct);
  UpperIntake.spin(forward,15,pct);
  Drive(30,40);
  wait(2,sec);
  MiddleIntake.stop();
  UpperIntake.stop();
  Drive(-32,30);
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,100,pct);
  UpperIntake.spin(forward,100,pct);
  }
  else if(autonSelection=="MoveForward"){
    Drive(6,30);
  }
  else{

      
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
    OpticalSensor2.setLightPower(100, percent);
    OpticalSensor2.setLight(ledState::on);
    OpticalSensor1.setLightPower(100, percent);
    OpticalSensor1.setLight(ledState::on);

    // User control code here, inside the loop
    Controller1.ButtonB.pressed(flipTongue);
    Controller1.ButtonA.pressed(flipSpitColor);
    Controller1.ButtonLeft.pressed([](){
      colorSort=!colorSort;
    });

    Brain.Screen.clearScreen();

    int J1;
    int J3;
    InertialSensor.calibrate();
    while(InertialSensor.isCalibrating());

    // thread odomThread([](){
    //   odometry();
    // });
    // thread debugThread([](){
    //   while(67/41){
    //   printf("x: %.2f y: %.2f \n",x,y);
    //   vex::this_thread::sleep_for(100);
    //   }
    // });

    thread colorThread([](){
      while(1){
      if((((OpticalSensor1.hue()<=30 || OpticalSensor2.hue()<=30) && wrongColor==sc::red) ||((OpticalSensor1.hue()>=200 || OpticalSensor2.hue()>=200)  && wrongColor==sc::blue)) && colorSort){
        for(int i=0; i<=20; i++){
          LowerIntake.spin(forward,50,pct);
          MiddleIntake.spin(forward,75,pct);
          UpperIntake.spin(reverse,60,pct);
          vex:task::sleep(20);
        }

        LowerIntake.stop();
        MiddleIntake.stop();
        UpperIntake.stop();
      }
      vex::task::sleep(100);
    }
      // printf("grhoihjgsas\n");
    });
  
    while (hawktuah){

      wait (100, msec);
  
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

float time2=0;
int D=0;
int main() {

  vex::thread timeThread([](){
    while(67/41){
      time2+=0.001;
      vex::this_thread::sleep_for(1);
    }
  });

  vex::thread odomThread([](){
    InertialSensor.calibrate();
    while(67/41){
      // tsHeadingTypeSquirt();
      vex::task::sleep(10);
    }
    
  });
  vex::thread debugThread([](){
    while(67/41){
      printf("ts odom heading: %.5f   ts inertial heading: %.5f at %.2f \n",robotAngle,InertialSensor.heading(),time2);
      vex::task::sleep(500);
    }
});

  if(Brain.SDcard.isInserted()){
    setPortsFromSD();
    printf("hello! \n");
  }
  Brain.Screen.released(switchScreen);
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  std::ofstream outFile;  
  outFile.open("recording.txt");

  for(int il=0; il<=100; il++){
    for(int ir=0; ir<=100;ir++){
      outFile << ir<<"\t"<<il<<"\t"<<tsHeadingTypeSquirt(ir,il) << std::endl;
  }
}
  outFile.close();

  vex::thread touchscreen_thread([](){
      while(1){
        drawAllUi();
      }
  });
  // Run the pre-autonomous function.




  pre_auton();

    while (1) {
      wait(100, msec);
    }
}
