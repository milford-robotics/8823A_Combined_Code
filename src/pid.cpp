#include <vex.h>

class PID_Controller
{
private:
    float tol; //constants
    float ki, kp, kd; //the big three
    float integral, error, derivative, olderror, dt, start; //things changed in pid loop
    float anti_wind; //other things that can be changed
    float fitness=0; //for tuning
public:
    PID_Controller(){
        ki=1; kp=1; kd=0; anti_wind=0.2;
    };
    PID_Controller(float ki,float kp,float kd,float anti_wind){
        this->ki=ki; this->kp=kp; this->kd=kd; this->anti_wind=anti_wind;
    };
    PID_Controller(float ki,float kp,float anti_wind){
        this->ki=ki; this->kp=kp; this->kd=0; this->anti_wind=anti_wind;
    };
    
    void setParams(float kp,float ki); // {externally defined} set params
    void setParams(float kp,float ki,float anti_wind); // overload of above
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
    PID_Controller makeSimilar(float kpM, float kiM, float awM){
        return PID_Controller(this->kp+kpM,this->ki+kiM,this->anti_wind+awM);
    }
    

};

class PID_Tuner
{
    private:
        PID_Controller mainControl=PID_Controller();
        std::vector<PID_Controller> otherControls;
        //tuning variables (how much to punish for each)
        double overshoot,time,flip,error;
        //starting vars
        double startkp, startki,startwind;
        //other
        double too_small=0.01,starting_step=2;
    public:
        PID_Controller tune(){
            
            mainControl.setParams(startkp,startki,startwind);
            double step=starting_step;
            while(step>=too_small){
                otherControls.at(0)=mainControl.makeSimilar(step,0,0);
                otherControls.at(1)=mainControl.makeSimilar(-step,0,0);
                otherControls.at(2)=mainControl.makeSimilar(0,step,0);
                otherControls.at(3)=mainControl.makeSimilar(0,-step,0);
                otherControls.at(4)=mainControl.makeSimilar(0,0,step);
                otherControls.at(5)=mainControl.makeSimilar(0,0,-step);

                float fitness=0;
                float spd=mainControl.calculate(90);
                while(spd!=6741){
                    float spd=mainControl.calculate(90);
                    LeftFront.spin(forward,error,percent);
                    LeftMiddle.spin(forward,error,percent);
                    LeftRear.spin(forward,error,percent);
                    RightFront.spin(reverse,error,percent);
                    RightMiddle.spin(reverse,error,percent);
                    RightRear.spin(reverse,error,percent);
                }
            }



            return mainControl;
        }

};

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
