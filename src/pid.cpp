#include <vex.h>

class pid
{
private:
    double ki, kp, kd, dt;
public:
    pid();
    void setParams(){
        
    };
};

pid::pid(){

}



void pid (int angle){ // Turn function
  auto clamp=[](float val,float bottom,float top){return std::min(std::max(val,bottom),top);};
  float error=0, tol=0.2,start=InertialSensor.rotation(), speed=0, ki=0.2,kp=0.25,integral=0,oldError;

  do{
    oldError=error;
    
    vex::task::sleep(10);

    printf("e: %.2f h: %.2f i: %.2f s: %.2f \n",error,InertialSensor.rotation(),integral, speed);
    error=angle-(InertialSensor.rotation()-start);
    speed=ki*integral+kp*error;
    speed=clamp(speed,-80,80);
    if(fabs(error)<=fabs(angle*0.3)) integral+=error*0.005;

    LeftFront.spin(forward,speed,percent);
    LeftMiddle.spin(forward,speed,percent);
    LeftRear.spin(forward,speed,percent);
    RightFront.spin(reverse,speed,percent);
    RightMiddle.spin(reverse,speed,percent);
    RightRear.spin(reverse,speed,percent);

  }while(fabs(error)+fabs(oldError)>tol);
  StopDriveTrain();
  printf("done \n\n\n");
  vex::task::sleep(1000);

}
