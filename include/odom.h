// extern  constexpr  double gear_ratio;
// extern constexpr      double encoder_wheel_radius;
// extern  constexpr double encoder_wheel_circumference;
// extern    constexpr      double start_heading;
#pragma once


double Heading();
extern double robotAngle;
void odometry();
void thePMOThing();
double getTheMotorPositionTsInRotationsTypeSquirtOnGod(vex::turnType direction);

extern double robotX;
extern    double robotY;
extern double rawRightDist;
extern double rawLeftDist;

extern double oldRightDist, oldLeftDist;
