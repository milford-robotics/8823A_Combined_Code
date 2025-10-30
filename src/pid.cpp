#include <vex.h>

class PID_Controller
{
private:
    float tol; //constants
    float ki, kp, kd; //the big three
    float integral, error, derivative, olderror, dt, start; //things changed in pid loop
    float anti_wind; //other things that can be changed
public:
    void setParams(float kp,float ki,float kd);
    void setParams(float kp,float ki);
    float calculate (int angle){ // Turn function
        this->error=angle-(InertialSensor.rotation()-this->start); //get error
        if(fabs(this->error)+fabs(this->olderror)>tol){ //if pid isnt right

            float speed=ki*integral+kp*error; //calculate target speed
            if(fabs(this->error)<=fabs(angle*this->anti_wind)) integral+=error*0.005; //update integral
            this->olderror=this->error; //set old error
            return speed<-80?-80:speed>80?80:speed; //return (-80<error<80)
        }
        else{
            StopDriveTrain();
            printf("done \n\n\n");
            return 0;
        }

    }
    void reset(){
        this->olderror=0.0/0.0;
    }

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
