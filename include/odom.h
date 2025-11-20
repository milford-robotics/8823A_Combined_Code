// extern  constexpr  double gear_ratio;
// extern constexpr      double encoder_wheel_radius;
// extern  constexpr double encoder_wheel_circumference;
// extern    constexpr      double start_heading;
#pragma once


double Heading();
extern long double robotAngle;
void odometry();
void thePMOThing();
long double getTheMotorPositionTsInRotationsTypeSquirtOnGod(vex::turnType direction);

extern long double robotX;
extern    long double robotY;
extern long double rawRightDist;
extern long double rawLeftDist;
extern long double rightDist;
extern long double leftDist;

extern long double oldRightDist, oldLeftDist;
void tune();


class PID_Controller
{
private:
    float tol=0.5,maxspd=40; //constants
    float ki=0, kp=0, kd=0; //the big three
    float integral=0, error=0, olderror=0, dt=0, start=0; //things changed in pid loop
    float anti_wind=0; //other things that can be changed
    float fitness=99999; //for tuning
public:
    PID_Controller(){
        ki=1; kp=1; kd=0; anti_wind=0.2;
    };
    PID_Controller(float ki,float kp,float kd,float anti_wind){
        this->ki=ki; this->kp=kp; this->kd=kd; this->anti_wind=anti_wind;
    };
    PID_Controller(float ki,float kp,float anti_wind){
        this->ki=ki>0?ki:0; this->kp=kp>0?kp:0; this->kd=kd>0?kp:0; this->anti_wind=anti_wind;
    };
    
    void setParams(float kp,float ki); // {externally defined} set params
    void setParams(float kp,float ki,float anti_wind); // overload of above
    float calculate (int angle){ // Turn function
        if(olderror==6741){start=InertialSensor.rotation(); integral=0; this->olderror=angle;}
        this->error=angle-(InertialSensor.rotation()-this->start); //get error
        if(fabs(this->error)+fabs(this->olderror)>tol*2){ //if pid isnt right

            float speed=ki*integral+kp*error; //calculate target speed
            if(fabs(this->error)<=fabs(angle*this->anti_wind)) integral+=error*0.005; //update integral
            this->olderror=this->error; //set old error
            return speed<-maxspd?-maxspd:speed>maxspd?maxspd:speed; //return (-80<error<80)
        }
        else{
            StopDriveTrain();
            return 0;
        }

    }
    PID_Controller makeSimilar(float kpM, float kiM, float awM){
        return PID_Controller(this->kp+kpM,this->ki+kiM,this->anti_wind+awM);
    }
    void reset(){
        this->start=InertialSensor.rotation();
        this->integral=0;
        this->olderror=99;
        this->error=99;
    }
    void setFitness(float fitness){this->fitness=fitness;}
    void addFitness(float toAdd){this->fitness+=fabs(toAdd);}
    float getFitness(){return this->fitness;}
    float getError(){return this->error;}
    float getOldError(){return this->olderror;}
    void printController(){std::printf("KP: %.3f  KI: %.3f  AW: %.3f  Fitness: %.1f\n",kp,ki,anti_wind,fitness);}
    

};

class PID_Tuner
{
    private:
        PID_Controller mainControl=PID_Controller();
        PID_Controller otherControls[6];
        //tuning variables (how much to punish for each)
        double overshootVal,timeVal,flipVal,errorVal;
        //starting vars
        double startkp, startki,startwind;
        //other
        double too_small=0.001,starting_step=1;

        double pidStart, runtime;
    public:
        PID_Tuner(double overshootVal,double timeVal,double flipVal, double errorVal){
            startkp=0.3;
            startki=0.038;
            startwind=0.2;
            this->overshootVal=overshootVal;
            this->timeVal=timeVal;
            this->flipVal=flipVal;
            this->errorVal=errorVal;
        }
        PID_Controller tune(){
            
            mainControl.setParams(startkp,startki,startwind);
            double step=starting_step;
            mainControl=test(mainControl);
            mainControl.printController();
            double bestFitness=mainControl.getFitness();
            while(step>=too_small){
                mainControl=test(mainControl);
                mainControl.printController();
                bool didntchange=true;
                otherControls[0]=mainControl.makeSimilar(step,0,0);
                otherControls[1]=mainControl.makeSimilar(-step,0,0);
                otherControls[2]=mainControl.makeSimilar(0,step,0);
                otherControls[3]=mainControl.makeSimilar(0,-step,0);
                otherControls[4]=mainControl.makeSimilar(0,0,step);
                otherControls[5]=mainControl.makeSimilar(0,0,-step);

                for(PID_Controller control : otherControls){
                    control.setFitness(1);
                    control.reset();
                    control=test(control);
                    control.printController();
                    if(control.getFitness()<mainControl.getFitness()){ mainControl=control; didntchange=false; printf("New best \n");}
                    vex::task::sleep(10000);
                }
                if(didntchange){
                    step/=2;
                    printf("step is now %f\n",step);
                }
                vex::task::sleep(2000);

            }


            return mainControl;
        }
        //function that tests the fitness of turn params by turning it and punishing based on values
        PID_Controller test(PID_Controller control){
            runtime=0; //initialize values
            control.reset();
            float spd=control.calculate(90);
            pidStart=control.getError();

            while(spd!=0.0){ //while error is not in acceptable margin
                spd=control.calculate(90); //main inertial turn loop
                LeftFront.spin(forward,spd,percent);
                LeftMiddle.spin(forward,spd,percent);
                LeftRear.spin(forward,spd,percent);
                RightFront.spin(reverse,spd,percent);
                RightMiddle.spin(reverse,spd,percent);
                RightRear.spin(reverse,spd,percent); 


                control.addFitness(control.getError()*errorVal); //change fitness values
                control.addFitness(runtime++*timeVal); //add to runtime and punish based on time taken
                if(pidStart<0!=control.getError()<0) control.addFitness(control.getError()*overshootVal); //if the signs of the start and current errors don't match, punish for overshoot
                if(control.getError()<0!=control.getOldError()<0) control.addFitness(flipVal); //if the sign of the error just flipped, punish for flipping
                if(runtime>=300) {control.addFitness(100000000*pow(control.getError(),2)); printf("DNF %i %f \n",runtime,control.getError()); break;};
                vex::task::sleep(50); //give motors time to respond
            }
            StopDriveTrain(); //after loop, stop
            return control;

        }

};
