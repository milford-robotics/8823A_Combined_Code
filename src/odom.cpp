#include <vex.h>
#include <cmath>

using namespace vex;


// LEAVE THIS!!! IF THE OTHER EQUATIONS WORK, THEN YOU CAN SCRAP THIS!!

    constexpr double gear_ratio = ((double)1/1);
    constexpr double encoder_wheel_radius = 1.375;
    constexpr double encoder_wheel_circumference = 2 * M_PI * encoder_wheel_radius;
    constexpr double start_heading = 90;

    // defines x and y for later
    float x = 0;
    float y = 0;



void odometry(double , double ) {

    FrontEncoder.resetPosition();


    float previous_distance_traveled = 0;

    while(1) {
        // Using std::fmod to preserve the "wraparound effect" of 0-360 degrees when considering the offset of start heading.
        float heading = std::fmod((360 - InertialSensor.rotation()) + start_heading, 360); // finds the angle

        // float average_encoder_position = (LeftEncoder.position(vex::degrees) + RightEncoder.position(vex::degrees)) / 2; // finds the encoder position on the robot
        float average_encoder_position = (FrontEncoder.position(vex::degrees)); // finds the encoder position on the robot

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

