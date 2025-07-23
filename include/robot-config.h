
#pragma once

using namespace vex;

extern brain       Brain;
extern controller Controller1;
extern motor LeftFront;
extern motor LeftMiddle;
extern motor LeftRear;
extern motor RightFront;
extern motor RightMiddle;
extern motor RightRear;
extern motor UpperIntake;
extern motor MiddleIntake;
extern motor LowerIntake;
extern rotation LeftEncoder;
extern rotation RightEncoder;
extern rotation BackEncoder;
extern inertial InertialSensor;
extern digital_out Tongue;

int getMotorPort(std::string motorName);
void setMotorPort(std::string motorName, int port);
void setAllMotorPorts();

void Drive (int dist, int speed);

void Turn (int angle);

void StopDriveTrain ();