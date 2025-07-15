#include <vex.h>

using namespace vex;

vex::gearSetting gearRatioList[] = {ratio6_1, ratio18_1, ratio36_1};
int32_t portList[] = {PORT1,PORT2,PORT3,PORT4,PORT5,PORT6,PORT7,PORT8,PORT9,PORT10,PORT11,PORT12,PORT13,PORT14,PORT15,PORT16,PORT17,PORT18,PORT19,PORT20,PORT21};


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

bool isMotorReversed(vex::motor motor){
    return true;
}

vex::gearSetting getMotorCartridge(vex::motor motor){
    return ratio6_1;
}


int getMotorPort(std::string motorName){
    std::ifstream inFile;
    inFile.open(motorName+".txt");

    int valToReturn=0;

    inFile >> valToReturn;

    return valToReturn;

}