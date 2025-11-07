#include <vex.h>
// pick cmath or math.h, don't use both
#include <cmath>
#include <math.h>

// The vex namespace is being brought into the current one but vex::*** is still being used, pick one or the other
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




class Circle{
  public:
    float radius, circumfrence, area;
    Circle(float radius){
      this->radius=radius;
      this->circumfrence=radius*M_TWOPI;
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

// Completely get rid of all globals that are not constants
// If shared state is needed refactor into a class or reorganise the functions to feed the correct data into each other


// Not used
double heading=0;

// Assigned in the Heading function but never read
double robotAngle=0;
double robotX=0, robotY=0;
// Get rid of magic numbers
// These can also be constexpr
const double wheelDiam=2.75,robotWidth=10.+1./2.;
const double wheelCirc=wheelDiam*M_PI;

// This does not get a new value every time it is read, instead it just reads a (most likley garbage) value at the start of the program only.
// I am not sure if the encoder objects technically have to exist at this point either so it might randomly fail to link one day
double rawRightDist=RightEncoder.position(rev)*wheelCirc;
double rawLeftDist=LeftEncoder.position(rev)*wheelCirc;

// These are also not used
double oldRightDist=rawRightDist, oldLeftDist=rawLeftDist;

// What???? When is this set to something other than zero??? Causes 0/x in the move functions
float rotationsLeft=0, rotationsRight=0;

// Change arguments to (float left, float right) for clarity
// This is also usless because it has an overshoot problem that is fixed by the pid drive function
// Was the intended function spinBase(float leftSpeed, float rightSpeed);?
void moveDistance(float dist, bool left){
  LeftEncoder.resetPosition();
  if(left){
    while (LeftEncoder.position(rev) * wheelDiam < dist){
      // max(min(x)) can be replaced by clamp
      LeftFront.spin(forward,std::max(std::min(rotationsLeft/rotationsRight,100.f),-100.f),percent);
      LeftMiddle.spin(forward,std::max(std::min(rotationsLeft/rotationsRight,100.f),-100.f),percent);
      LeftRear.spin(forward,std::max(std::min(rotationsLeft/rotationsRight,100.f),-100.f),percent);
    }
  }
  else{
    while(RightEncoder.position(rev)*wheelDiam<dist){
      // max(min(x)) can be replaced by clamp
      RightFront.spin(forward,std::max(std::min(rotationsRight/rotationsLeft,100.f),-100.f),percent);
      RightMiddle.spin(forward,std::max(std::min(rotationsRight/rotationsLeft,100.f),-100.f),percent);
      RightRear.spin(forward,std::max(std::min(rotationsRight/rotationsLeft,100.f),-100.f),percent);
    }
  }
  StopDriveTrain();

}

// Same feedback as moveDistance
void moveTo(float left, float right){
  // Move this to a normal function so it can be used in other places, maybe put it in a helper/utils/math namespace?
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

// I would make a seprate enum for the side of the base, as the turntype enum is supposed to be used for turns (not a big issue)
double getMotorPosition(vex::turnType direction){
  if(direction==vex::turnType::left){
    return (LeftFront.position(rev)+LeftMiddle.position(rev)+LeftRear.position(rev))/3.0;
   
  }
  else{
    return (RightFront.position(rev)+RightMiddle.position(rev)+RightRear.position(rev))/3.0;
  }
}

// From what I remember if this function is not ran continuously with the results summed up it will diverge from the actual value
// If there is no odom thread to do that + a mechinism to get the value out of it just use an inertial sensor
// Otherwise this math looks correct from what I remember, the name of the function is a bit misleading though (see above ^)
double Heading(){
  
    //Establish return variables
    double angle=0,xDist=0,yDist=0,rawX=0,rawY=0; // These are never used

    //Get left and right dist
    rawRightDist=getMotorPosition(right)*wheelCirc;
    rawLeftDist=getMotorPosition(left)*wheelCirc;

    double rightDist=rawRightDist;
    double leftDist=rawLeftDist;

    double r=(leftDist*robotWidth)/(rightDist-leftDist);
    double r_left=r+robotWidth;
    double centralAngle=(rightDist/r_left)*180.0/M_PI;

    if(centralAngle!=centralAngle){
      centralAngle=0;
    }
    centralAngle*=2./3.;
    
    robotAngle=-centralAngle;

    return centralAngle;

}

// If this is not read (and UNDERSTOOD!!!!) by this friday your gurt will be skinched