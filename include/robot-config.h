
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
extern inertial InertialSensor1;
extern inertial InertialSensor2;
extern digital_out Tongue;

float InertialSensor();