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
  vex::task::sleep(10);
}

void Drive (float dist, int speed){ // Drive function
  float rotations;
  rotations = 360.*dist/(encoder_wheel_circumference*gear_ratio);
  LeftFront.spinFor (forward, rotations, degrees, speed, velocityUnits::pct, false);
  LeftMiddle.spinFor (forward, rotations, degrees, speed, velocityUnits::pct, false);
  LeftRear.spinFor (forward, rotations, degrees, speed, velocityUnits::pct, false);
  RightFront.spinFor (forward, rotations, degrees, speed, velocityUnits::pct, false);
  RightMiddle.spinFor (forward, rotations, degrees, speed, velocityUnits::pct, false);
  RightRear.spinFor (forward, rotations, degrees, speed, velocityUnits::pct, true);
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



void DriveP (int dist, int speed){ // Drive function
  auto clamp=[](float val,float bottom,float top){return std::min(std::max(val,bottom),top);};
  float rotations = dist/(encoder_wheel_circumference*gear_ratio);
  float mod=1;
  float error=67;
  
  while(error>=0){
    error=rotations-(getTheMotorPositionTsInRotationsTypeSquirtOnGod(left)+getTheMotorPositionTsInRotationsTypeSquirtOnGod(right))/2.;
    if(error<=rotations*0.5) mod=clamp(2.*(error/rotations),0.1,1);
    if(error>=rotations*0.5) mod=clamp(2.*(1-error/rotations),0.1,1);
    printf("e: %.2f m: %.2f s: %.2f r: %.2f \n",error,mod,speed*mod,rotations);
    LeftFront.spin (forward, speed*mod, velocityUnits::pct);
    LeftMiddle.spin (forward, speed*mod, velocityUnits::pct);
    LeftRear.spin (forward, speed*mod, velocityUnits::pct);
    RightFront.spin (forward, speed*mod, velocityUnits::pct);
    RightMiddle.spin (forward, speed*mod, velocityUnits::pct);
    RightRear.spin (forward, speed*mod, velocityUnits::pct);
  }
}


void Turn (int angle){ // Turn function
  auto clamp=[](float val,float bottom,float top){return std::min(std::max(val,bottom),top);};
  float error=0, tol=0.2,start=InertialSensor.rotation(), speed=0, ki=0.2,kp=0.1,integral=0,oldError;

  do{
    oldError=error;
    
    vex::task::sleep(10);

    printf("e: %.2f h: %.2f i: %.2f s: %.2f \n",error,InertialSensor.rotation(),integral, speed);
    error=angle-(InertialSensor.rotation()-start);
    speed=ki*integral+kp*error;
    speed=clamp(speed,-80,80);
    if(fabs(error)<=fabs(angle*0.3)) integral+=error*0.005;

    LeftFront.spin(forward,speed,percent);
    LeftMiddle.spin(forward,speed,percent);
    LeftRear.spin(forward,speed,percent);
    RightFront.spin(reverse,speed,percent);
    RightMiddle.spin(reverse,speed,percent);
    RightRear.spin(reverse,speed,percent);

  }while(fabs(error)+fabs(oldError)>tol);
  StopDriveTrain();
  printf("done \n\n\n");
  vex::task::sleep(1000);

}
void TurnP (int angle){ // Turn function but it mansion
  auto clamp=[](float val,float bottom,float top){return std::min(std::max(val,bottom),top);};
  auto sign=[](float val){if(val<0)return -1; else if(val>0)return 1;else return 0;};
  auto matchsign=[sign](float val,float match){if(sign(val)!=sign(match))return val*-1;else return val;};
  float error=0, tol=0.02,start=InertialSensor.rotation(), speed=0, ki=0.2,kp=0.1,integral=0,oldError;

  do{
    oldError=error;
    
    vex::task::sleep(10);

    printf("e: %.2f h: %.2f i: %.2f s: %.2f \n",error,InertialSensor.rotation(),integral, speed);
    error=angle-(InertialSensor.rotation()-start);
    speed=ki*integral+kp*error;
    speed=clamp(speed,-80,80);
    if(fabs(error)<=fabs(angle*0.3)) integral+=error*0.005;
    if(fabs(error)<=fabs(angle*0.02)) integral=matchsign(clamp(integral,-2,2),error);

    LeftFront.spin(forward,speed,percent);
    LeftMiddle.spin(forward,speed,percent);
    LeftRear.spin(forward,speed,percent);
    RightFront.spin(reverse,speed,percent);
    RightMiddle.spin(reverse,speed,percent);
    RightRear.spin(reverse,speed,percent);

  }while(fabs(error)+fabs(oldError)>tol);
  StopDriveTrain();
  printf("done \n\n\n");
  vex::task::sleep(1000);

}


void pre_auton(void) {

  InertialSensor.calibrate();

  while (InertialSensor.isCalibrating()){
    vex::task::sleep(25);
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
  // Turn(90);
  // vex::task::sleep(1000);
  // Turn(90);
  // vex::task::sleep(1000);
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
  // autonSelection="hawk tuah!";    
  if(autonSelection=="RightSide"){
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,75,pct);
  UpperIntake.spin(forward,15,pct);
  Drive(28,25);
  vex::task::sleep(50);
  MiddleIntake.spin(forward,25,pct);
  UpperIntake.stop();
  Turn(-75);
  vex::task::sleep(25);
  Drive(13,30);
  vex::task::sleep(25);
  LowerIntake.spin(reverse,70,pct);
  MiddleIntake.spin(reverse,50,pct);
  UpperIntake.spin(reverse,30,pct);
  vex::task::sleep(1500);
  LowerIntake.stop();
  MiddleIntake.stop();
  UpperIntake.stop();
  Drive(-47,40);
  vex::task::sleep(25);
  // TurnToHeading(-123);
  // vex::task::sleep(75);
  Turn(-131);
  vex::task::sleep(50);
  Drive(-18,30);
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,100,pct);
  UpperIntake.spin(forward,100,pct);
  vex::task::sleep(1000);
  Tongue.set(true);
  vex::task::sleep(25);
  LowerIntake.spin(forward,90,pct);
  MiddleIntake.spin(forward,65,pct);
  UpperIntake.spin(forward,15,pct);
  Drive(29,50);
  vex::task::sleep(2000);
  MiddleIntake.stop();
  UpperIntake.stop();
  Drive(-32,30);
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,100,pct);
  UpperIntake.spin(forward,100,pct);
  }
  else if(autonSelection=="LeftSide"){
    LowerIntake.spin(forward,100,pct);
    MiddleIntake.spin(forward,45,pct);
    UpperIntake.spin(forward,15,pct);
    Drive(27,25);
    vex::task::sleep(50);
    MiddleIntake.stop();
    UpperIntake.stop();
    Turn(75);
    vex::task::sleep(25);
    Tongue.set(true);
    vex::task::sleep(1000);
    Drive(15,30);
    vex::task::sleep(25);
    LowerIntake.spin(forward,100,pct);
    MiddleIntake.spin(forward,75,pct);
    UpperIntake.spin(reverse,85,pct);
    vex::task::sleep(1500);
    LowerIntake.stop();
    MiddleIntake.stop();
    UpperIntake.stop();
    Drive(-49,40);
    Tongue.set(false);
    vex::task::sleep(25);
    // TurnToHeading(124);
    // vex::task::sleep(75);
    Turn(131);
    vex::task::sleep(25);
    Drive(-18,30);
    LowerIntake.spin(forward,100,pct);
    MiddleIntake.spin(forward,100,pct);
    UpperIntake.spin(forward,100,pct);
    vex::task::sleep(1000);
    Tongue.set(true);
    vex::task::sleep(25);
    LowerIntake.spin(forward,90,pct);
    MiddleIntake.spin(forward,65,pct);
    UpperIntake.spin(forward,15,pct);
    Drive(30,40);
    vex::task::sleep(2000);
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
  else{ // Skills
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,75,pct);
  UpperIntake.spin(forward,15,pct);
  //go to cluster
  Drive(33.2f,25);
  vex::task::sleep(150);
  //reverse a little
  LowerIntake.spin(forward,75,pct);
  MiddleIntake.spin(forward,30,pct);
  UpperIntake.spin(forward,5,pct);
  Drive(-7.6f,20);
  UpperIntake.stop();
  vex::task::sleep(100);
  
  TurnP(-77);
  vex::task::sleep(25);
  //go to mid goal
  Drive(18,20);
  vex::task::sleep(25);
  //score
  LowerIntake.spin(reverse,40,pct);
  MiddleIntake.spin(reverse,40,pct);
  UpperIntake.spin(reverse,30,pct);
  vex::task::sleep(1500);
  //stop that
  LowerIntake.stop();
  MiddleIntake.stop();
  UpperIntake.stop();
  //come bacck from mid goal
  Drive(-16,40);
  vex::task::sleep(25);
  TurnP(-45);
  vex::task::sleep(25);
  //intake
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,75,pct);
  UpperIntake.spin(forward,15,pct);
  //drive across field
  Drive(51,30);
  vex::task::sleep(1000);
  MiddleIntake.spin(forward,15,pct);
  UpperIntake.stop();
  //turn 2 middle goal
  TurnP(137);
  vex::task::sleep(25);
  Tongue.set(true);
  vex::task::sleep(1000);
  //go to mid goal
  Drive(20,40);
  vex::task::sleep(25);
  //score the ts
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,65,pct);
  UpperIntake.spin(reverse,85,pct);
  vex::task::sleep(1500);
  LowerIntake.stop();
  MiddleIntake.stop();
  UpperIntake.stop();
  //high goal time
  Drive(-48,40);
  Tongue.set(false);
  vex::task::sleep(25);
  //go to the high goal to align
  TurnP(131);
  vex::task::sleep(75);
  Drive(-18,30);
  //Score ts
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,100,pct);
  UpperIntake.spin(forward,100,pct);
  vex::task::sleep(1000);
  //freaky tongye comes out for the loader
  Tongue.set(true);
  vex::task::sleep(25);
  LowerIntake.spin(forward,90,pct);
  MiddleIntake.spin(forward,65,pct);
  UpperIntake.spin(forward,15,pct);
  //go 2 loader
  Drive(30,40);
  vex::task::sleep(3500);
  MiddleIntake.stop();
  UpperIntake.stop();
  //back to high goal
  Drive(-32,30);
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,100,pct);
  UpperIntake.spin(forward,100,pct);
  vex::task::sleep(1500);
  Tongue.set(false);
  //go to x coord of park
  Drive(10,25);
  vex::task::sleep(50);
  //turn to start park
  TurnP(-90);
  vex::task::sleep(25);
  //
  Drive(24,40);
  vex::task::sleep(1000);
  TurnP(90);
  vex::task::sleep(75);
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,100,pct);
  UpperIntake.spin(forward,100,pct);
  Drive(20,30);
  vex::task::sleep(100);
  TurnP(90);
  vex::task::sleep(75);
  Drive(30,100);
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

    //   while(1){
    //     printf("\t%.2f\t\n\n",InertialSensor.rotation(deg));
    //     vex::task::sleep(100);
    //   }
    // });

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

      vex::task::sleep (100);
  
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
  LeftEncoder.resetPosition();
  RightEncoder.resetPosition();

  vex::thread timeThread([](){
    while(67/41){
      time2+=0.001;
      vex::this_thread::sleep_for(1);
    }
  });

  vex::thread odomThread([](){
    InertialSensor.calibrate();
    while(InertialSensor.isCalibrating());
    while(67/41){
      tsHeadingTypeSquirt();
      vex::task::sleep(50);
    }
    
  });
  vex::thread debugThread([](){
    std::ofstream outFile;
    outFile.open("recording.txt");
    printf("open \n");
    
    while(!Controller1.ButtonA.pressing()){
      outFile << InertialSensor.rotation() << "\t" << robotAngle << "\t" << InertialSensor.rotation()-robotAngle << "\t" << rawLeftDist << "\t" << rawRightDist << "\n" ;
      // outFile << robotX << "\t" << robotY << "\t" << "\n";
      vex::task::sleep(5);
    }
    printf("closed\n");
    outFile.close();
});

vex::thread debugPrint([](){
  while(1){
    // printf("x %.2f\ty %.2f\th %.2f\th2 %.2f %f %f\n",robotX,robotY,robotAngle,InertialSensor.rotation(),LeftEncoder.position(rev),RightEncoder.position(rev));
    vex::task::sleep(100);
  }
});
vex::thread setMotors([](){
  for(int i=0; i<100; i++){
  if(Brain.SDcard.isInserted()){
    setPortsFromSD();
    printf("hello! \n");
    vex::task::sleep(20);
  }
}
});
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




  pre_auton();

    while (1) {
      vex::task::sleep(100);
    }
}
