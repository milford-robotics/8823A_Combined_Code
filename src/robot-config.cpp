#include <vex.h>

using namespace vex;

vex::gearSetting gearRatioList[] = {ratio6_1, ratio18_1, ratio36_1};
int32_t portList[] = {PORT1,PORT2,PORT3,PORT4,PORT5,PORT6,PORT7,PORT8,PORT9,PORT10,PORT11,PORT12,PORT13,PORT14,PORT15,PORT16,PORT17,PORT18,PORT19,PORT20,PORT21};

int getMotorPort(std::string motorName){
    std::ifstream inFile;

    inFile.open(motorName+".txt");

    int valToReturn=0;

    inFile >> valToReturn;

    if(inFile.fail()){
        valToReturn=0;
    }

    inFile.close();
    
    return portList[valToReturn];

}

void setMotorPort(std::string motorName, int port){
    std::ofstream outFile;
    outFile.open(motorName+".txt");
    
    outFile << port;

    outFile.close();
}

void setAllMotorPorts(){
    std::ofstream outFile;
    outFile.open("LeftFront.txt");
    outFile << LeftFront.index();
    outFile.close();

    outFile.open("LeftMiddle.txt");
    outFile << LeftMiddle.index();
    outFile.close();

    outFile.open("LeftRear.txt");
    outFile << LeftRear.index();
    outFile.close();

    outFile.open("RightFront.txt");
    outFile << RightFront.index();
    outFile.close();

    outFile.open("RightMiddle.txt");
    outFile << RightMiddle.index();
    outFile.close();

    outFile.open("RightRear.txt");
    outFile << RightRear.index();
    outFile.close();

    outFile.open("UpperIntake.txt");
    outFile << UpperIntake.index();
    outFile.close();

    outFile.open("MiddleIntake.txt");
    outFile << MiddleIntake.index();
    outFile.close();

    outFile.open("LowerIntake.txt");
    outFile << LowerIntake.index();
    outFile.close();

}


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
motor LowerIntake = motor(PORT14, ratio6_1, false);
rotation LeftEncoder = rotation(PORT7, true);
rotation RightEncoder = rotation(PORT8, true);
rotation BackEncoder = rotation(PORT9, true);
inertial InertialSensor = inertial(PORT15);
digital_out Tongue = digital_out(Brain.ThreeWirePort.A);

bool isMotorReversed(vex::motor motor){
    if(motor.index()==LeftFront.index() || motor.index()==LeftMiddle.index() || motor.index()==LeftRear.index() || motor.index()==MiddleIntake.index() ){
        return true;
    }
    return false;
}

vex::gearSetting getMotorCartridge(vex::motor motor){
    return ratio6_1;
}


