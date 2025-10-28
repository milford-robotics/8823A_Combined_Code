#include <vex.h>
#include <cmath>
#include <math.h>

using namespace vex;


// LEAVE THIS!!! IF THE OTHER EQUATIONS WORK, THEN YOU CAN SCRAP THIS!!

    constexpr double gear_ratio = ((double)1/1);
    constexpr double encoder_wheel_radius = 1.375;
    constexpr double encoder_wheel_circumference = M_TWOPI * encoder_wheel_radius;
    constexpr double start_heading = 90;
    constexpr double track_width = 9;

    // defines x and y for later
    float x = 0;
    float y = 0;

    double left_distance = (LeftEncoder.position(deg) / 360) * encoder_wheel_circumference * gear_ratio;
    double right_distance = (RightEncoder.position(deg) / 360) * encoder_wheel_circumference * gear_ratio;


void odometry() {

    // FrontEncoder.resetPosition();


    float previous_distance_traveled = 0;

    while(1) {
        // Using std::fmod to preserve the "wraparound effect" of 0-360 degrees when considering the offset of start heading.

        float average_encoder_position = (LeftEncoder.position(vex::degrees) + RightEncoder.position(vex::degrees)) / 2; // finds the encoder position on the robot

        float heading_in_radians = (right_distance - left_distance) / track_width;
        float heading = std::fmod((M_TWOPI - (heading_in_radians)) + start_heading, M_TWOPI); // finds the angle

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
  //targetAngle=std::atan(xDiff/yDiff);

  //turn to point
  

  //get distance to point
  //targetDist=std::sqrt(std::pow(xDiff,2)+std::pow(yDiff,2));

  //drive there


  //find arc-distance to point


  std::printf("distance to point: %.3f   angle to point: %.3f rad %.3f deg \n", targetDist, targetAngle, (targetAngle*180/M_PI));


}



class Circle{
  public:
    float radius, circumfrence, area;
    Circle(float radius){
      this->radius=radius;
      this->circumfrence=radius*2*M_PI;
      this->area=pow(radius,2)*M_PI;
    }

    //get the physical length of an arc from the central angle
    //param: theta (central angle of circle) in RADIANS
    //return: float arc length in inches (whole curve, not a chord)
    float arcLength(float theta){
      return this->circumfrence*theta;
    }

    //get the physical length of a chord from the central angle
    //param: theta (central angle of circle) in RADIANS
    //return: float chord length in inches (chord, not whole curve)
    float chordLength(float theta){
      return this->circumfrence*sin(2*theta);
    }

    //get the proportion of a circle from distance
    //param: distance (in inches) the distance travelled
    //return: float percent of circle travelled
    float ratioFromDistance(float distance){
      return distance/this->circumfrence;
    }

    //get the central angle of an arc from the distance travelled
    //param: distance (in inches) the distance of the circle
    //return: float arc length in inches (chord, not whole curve)
    float angleFromDistance(float distance){
      return this->ratioFromDistance(distance)*M_TWOPI;
    }




};

double heading=0;

double robotAngle=0;
double robotX=0, robotY=0;
const double wheelDiam=2.75,robotWidth=10.+7./16.;

double rawRightDist=RightEncoder.position(rev)*wheelDiam;
double rawLeftDist=LeftEncoder.position(rev)*wheelDiam;

double oldRightDist=rawRightDist, oldLeftDist=rawLeftDist;

float hgsaohgdsihoio=0, vkjsjlkjalkjsf=0;

void moveDistance(float dist, bool left){
  LeftEncoder.resetPosition();
  if(left){
    while(LeftEncoder.position(rev)*wheelDiam<dist){
      LeftFront.spin(forward,std::max(std::min(hgsaohgdsihoio/vkjsjlkjalkjsf,100.f),-100.f),percent);
      LeftMiddle.spin(forward,std::max(std::min(hgsaohgdsihoio/vkjsjlkjalkjsf,100.f),-100.f),percent);
      LeftRear.spin(forward,std::max(std::min(hgsaohgdsihoio/vkjsjlkjalkjsf,100.f),-100.f),percent);
    }
  }
  else{
    while(RightEncoder.position(rev)*wheelDiam<dist){
      RightFront.spin(forward,std::max(std::min(vkjsjlkjalkjsf/hgsaohgdsihoio,100.f),-100.f),percent);
      RightMiddle.spin(forward,std::max(std::min(vkjsjlkjalkjsf/hgsaohgdsihoio,100.f),-100.f),percent);
      RightRear.spin(forward,std::max(std::min(vkjsjlkjalkjsf/hgsaohgdsihoio,100.f),-100.f),percent);
    }
  }
  StopDriveTrain();

}





// void robtoMoveButMoreSkinchTuahTypeTs(float left, float right){
//   hgsaohgdsihoio=left;
//   vkjsjlkjalkjsf=right;
//   vex::thread leftThread([](){moveDistance(hgsaohgdsihoio, true);});
//   moveDistance(vkjsjlkjalkjsf,false);
// }

void moveTo(float left, float right){
  auto clamp=[](float val,float low,float high){return std::min(std::max(low,val),high);};

  float rotationsR = right/(encoder_wheel_circumference*gear_ratio);
  float rotationsL = left/(encoder_wheel_circumference*gear_ratio);
  LeftFront.spinFor (forward, rotationsL, rev, clamp(rotationsL/rotationsR,-100,100), velocityUnits::pct, false);
  LeftMiddle.spinFor (forward, rotationsL, rev, clamp(rotationsL/rotationsR,-100,100), velocityUnits::pct, false);
  LeftRear.spinFor (forward, rotationsL, rev, clamp(rotationsL/rotationsR,-100,100), velocityUnits::pct, false);
  RightFront.spinFor (forward, rotationsR, rev, clamp(rotationsR/rotationsL,-100,100), velocityUnits::pct, false);
  RightMiddle.spinFor (forward, rotationsR, rev, clamp(rotationsR/rotationsL,-100,100), velocityUnits::pct, false);
  RightRear.spinFor (forward, rotationsR, rev, clamp(rotationsR/rotationsL,-100,100), velocityUnits::pct, true);
}

// void thePMOThing(){
//   while(strcmp("skinch tuah","epic furple gurtnite tuah!")!=67){
//     //fortnit balls
//     for(int ix=-10; ix<10; ix++){
//      for(int iy=-10; iy<10; iy++){
//         vex::task::sleep(100);
//         InertialSensor.resetHeading();
//         rogtbotisskinchmovefunction(ix,iy);
//         printf("%i\t%i\t%f\n",ix,iy,InertialSensor.heading());
//         vex::task::sleep(100);
//         rogtbotisskinchmovefunction(-ix,-iy);

//       } 
//     }

//   }
// }

double getMotorPosition(vex::turnType direction){
  if(direction==vex::turnType::left){
    return (LeftFront.position(rev)+LeftMiddle.position(rev)+LeftRear.position(rev))/3.0;
   
  }
  if(direction==vex::turnType::right){
    return (RightFront.position(rev)+RightMiddle.position(rev)+RightRear.position(rev))/3.0;
  }
  return 6.7;
}

double Heading(){
    //Establish return variables
    double angle=0,xDist=0,yDist=0,rawX=0,rawY=0;

    //Get left and right dist
    rawRightDist=getMotorPosition(right)*wheelDiam;
    rawLeftDist=getMotorPosition(left)*wheelDiam;

    // rawRightDist=RightEncoder.position(rev)*wheelDiam;
    // rawLeftDist=LeftEncoder.position(rev)*wheelDiam;

    double rightDist=rawRightDist;
    double leftDist=rawLeftDist;

    

    
    double r=(leftDist*robotWidth)/(rightDist-leftDist);
    double r_left=r+robotWidth;
    double centralAngle=(rightDist/r_left)*180.0/M_PI;
    // double centralAngle = (leftDist-rightDist)/robotWidth;

    if(centralAngle!=centralAngle) centralAngle=0;

    // if(centralAngle!=0){
    //   robotX+=((r+(robotWidth/2.))*cos(centralAngle*(M_PI/180.))-(r+(robotWidth/2.)))*cos(InertialSensor.rotation()*M_PI/180.);
    //   robotY+=((r+(robotWidth/2.))*sin(centralAngle*(M_PI/180.)))*sin(InertialSensor.rotation()*M_PI/180.);
    // }
    // else{
    //   robotX+=(leftDist+rightDist)/2*cos(InertialSensor.rotation()*M_PI/180.);
    //   robotY+=(leftDist+rightDist)/2*sin(InertialSensor.rotation()*M_PI/180.);
    // }
    // printf("%f %f %f %f \n",r,r+robotWidth/2.,cos(centralAngle*(M_PI/180.)),(r+(robotWidth/2.))*cos(centralAngle*(M_PI/180.)));
    
    robotAngle=centralAngle;

    // oldLeftDist=rawLeftDist;
    // oldRightDist=rawRightDist;



    return centralAngle;


}






//Implemented using 5225's odom framework (http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf)
// double 