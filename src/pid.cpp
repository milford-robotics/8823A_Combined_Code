#include <vex.h>

class PID_Controller
{
private:
    float tol; //constants
    float ki, kp, kd; //the big three
    float integral, error, derivative, olderror, dt, start; //things changed in pid loop
    float anti_wind; //other things that can be changed
public:
    PID_Controller(){
        ki=1; kp=1; kd=0; anti_wind=0.2;
    };
    PID_Controller(float ki,float kp,float kd,float anti_wind){
        this->ki=ki; this->kp=kp; this->kd=kd; this->anti_wind=anti_wind;
    };
    
    void setParams(float kp,float ki,float kd); // {externally defined} set params
    void setParams(float kp,float ki); // overload of above
    float calculate (int angle){ // Turn function
        if(olderror==6741){start=InertialSensor.rotation(); integral=0; this->olderror=angle;}
        this->error=angle-(InertialSensor.rotation()-this->start); //get error
        if(fabs(this->error)+fabs(this->olderror)>tol){ //if pid isnt right

            float speed=ki*integral+kp*error; //calculate target speed
            if(fabs(this->error)<=fabs(angle*this->anti_wind)) integral+=error*0.005; //update integral
            this->olderror=this->error; //set old error
            return speed<-80?-80:speed>80?80:speed; //return (-80<error<80)
        }
        else{
            StopDriveTrain();
            this->olderror=6741;
            return 6741;
        }

    }
    

};

class PID_Tuner
{
    private:
        PID_Controller mainControl=PID_Controller();
        
    public:

};

void PID_Controller::setParams(float kp,float ki,float kd){
    this->kp=kp;
    this->ki=ki;
    this->kd=kd;

}
void PID_Controller::setParams(float kp,float ki){
    this->kp=kp;
    this->ki=ki;
    this->kd=0;

}
