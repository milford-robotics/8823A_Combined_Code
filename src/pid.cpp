#include <vex.h>


void PID_Controller::setParams(float kp,float ki,float anti_wind){
    this->kp=kp;
    this->ki=ki;
    this->anti_wind=anti_wind;

}
void PID_Controller::setParams(float kp,float ki){
    this->kp=kp;
    this->ki=ki;
    this->kd=0;

}

void tune(){
    InertialSensor.calibrate();
    while(InertialSensor.isCalibrating());
    PID_Tuner tuner=PID_Tuner(20, 20,100,3);
    tuner.tune();
}