#include <vex.h>

using namespace vex;

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
rotation LeftEncoder = rotation(PORT7, true);
rotation RightEncoder = rotation(PORT8, true);
rotation BackEncoder = rotation(PORT9, true);
inertial InertialSensor = inertial(PORT15);