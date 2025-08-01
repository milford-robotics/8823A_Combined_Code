#include <vex.h>
#include <robot-config.h>



using namespace vex;

const char * autonSelection="Skills";

int screen=1300;


const char * motorIndexes[]={"nil1","nil2", "nil3", "nil4", "nil5",
                             "nil6", "nil7", "nil8", "nil9", "nil10", 
                             "nil11", "nil12", "nil13", "nil14", "nil15"
                             "nil16", "nil17", "nil18", "nil19", "nil20",
                             "nil21", "nil22", "nil23", "nil24",
                              }; //A list for storing full motor names  (Front Left, Front Right, Middle Right)

const char * motorIndexesShort[23]={"nil1","nil2", "nil3", "nil4", "nil5",
                             "nil6", "nil7", "nil8", "nil9", "nil10", 
                             "nil11", "nil12", "nil13", "nil14", "nil15"
                             "nil16", "nil17", "nil18", "nil19", "nil20",
                             "nil21", "nil22", "nil23", "nil24",
                              }; //A list for storing shortened motor names (FL, FR, MR)

void setMotorNames(){
    motorIndexesShort[LeftFront.index()]="FL"; //Establish motor shorthands
    motorIndexesShort[LeftMiddle.index()]="ML";
    motorIndexesShort[LeftRear.index()]="BL";
    motorIndexesShort[RightFront.index()]="FR";
    motorIndexesShort[RightMiddle.index()]="MR";
    motorIndexesShort[RightRear.index()]="BR";
    motorIndexesShort[UpperIntake.index()]="UI";
    motorIndexesShort[MiddleIntake.index()]="MI";
    motorIndexesShort[LowerIntake.index()]="LI";

    motorIndexes[LeftFront.index()]="Front Left"; //Establish motor names
    motorIndexes[LeftMiddle.index()]="Middle Left";
    motorIndexes[LeftRear.index()]="Back Left";
    motorIndexes[RightFront.index()]="Front Right";
    motorIndexes[RightMiddle.index()]="Middle Right";
    motorIndexes[RightRear.index()]="Back Right";
    motorIndexes[UpperIntake.index()]="Upper Intake";
    motorIndexes[MiddleIntake.index()]="Middle Intake";
    motorIndexes[LowerIntake.index()]="Lower Intake";
}

void motorInfoScreen(vex::motor selectedMotor){

    Brain.Screen.setFillColor(black); //Big font
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFont(mono40);
    Brain.Screen.setCursor(4,1);
    Brain.Screen.print(motorIndexes[selectedMotor.index()]); //Print full motor name
    Brain.Screen.setCursor(1,1);

    Brain.Screen.setFont(mono20); //Reset font
    
    if(isMotorReversed(selectedMotor)==false) Brain.Screen.print("Forward"); 
    if(isMotorReversed(selectedMotor)==true) Brain.Screen.print("Reversed"); 
    Brain.Screen.newLine();

    Brain.Screen.print("Port: "); Brain.Screen.print(selectedMotor.index()+1); Brain.Screen.newLine(); //Print the port
    if(selectedMotor.installed()){ //Only print the following info if the motor is installed
        Brain.Screen.print("Temperature: "); Brain.Screen.print(selectedMotor.temperature(fahrenheit)); Brain.Screen.print(" F"); Brain.Screen.newLine(); //Print the temperature
        Brain.Screen.print("Wattage: ");  //Print whether the motor is 5.5 or 11 watts
        if(selectedMotor.getMotorType()==0) Brain.Screen.print("11W"); 
        if(selectedMotor.getMotorType()==1) Brain.Screen.print("5.5W"); 
        Brain.Screen.newLine();
    } else{ //If it's not installed...
        Brain.Screen.setFont(mono60); //Print in big red font
        Brain.Screen.newLine();
        Brain.Screen.setPenColor(red);
        Brain.Screen.print("PLUG IN MOTOR!!"); //Tell user to plug motor in
    }
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFont(mono20);
    Brain.Screen.setFillColor(orange);
    Brain.Screen.drawRectangle(270, 10, 125, 50);
    Brain.Screen.printAt(280,30,"Emergency");
    Brain.Screen.printAt(280, 50, "Reassign");  
}


void displayMotor(vex::motor selectedMotor,int x,int y){
    Brain.Screen.setCursor((y+60)/20,(x+20)/10); //Set brain to the right row and column (converted from pixels)
    Brain.Screen.setPenWidth(1);
    if (!selectedMotor.installed()){Brain.Screen.setFillColor(red);} //Change color to red if unplugged
    else {Brain.Screen.setFillColor(100);} //Else change color to green
  
    Brain.Screen.drawRectangle(x,y,100,50); //Draw the button (pixels)
  
    Brain.Screen.print(motorIndexesShort[selectedMotor.index()]); //Print the name of the motor
  
    Brain.Screen.print(" P");           
    Brain.Screen.print(selectedMotor.index()+1); //Print the port using the .index method
  
    if (selectedMotor.installed()){ //If the motor is installed...
      Brain.Screen.print(" ");
      Brain.Screen.print(int(selectedMotor.temperature(fahrenheit))); //Print the temperature in fahrenheit
      Brain.Screen.print("F");
    }
} 
  

void switchScreen(){

    printf("%i, %i \n",Brain.Screen.xPosition(), Brain.Screen.yPosition());

    if(screen>1300 && screen<=1399 ) {
        if(Brain.Screen.xPosition()<=395  && Brain.Screen.xPosition()>=270 &&
           Brain.Screen.yPosition()<=60   && Brain.Screen.yPosition()>=10){
            screen+=100;
        }
    }

    else if(screen>1400 && screen<=1499 ) {
        int32_t motorPort=-1;
        bool clickedOnMotor=false;

        for(int ix=0; ix<7; ix++){ //Iterate through all possible ports
            for(int iy=0; iy<3; iy++){
                int port=ix+7*iy;
                int x=(ix*68.5)+10;
                int y=(iy*66)+45;

                if(Brain.Screen.xPosition()>=x && Brain.Screen.xPosition()<=x+50 && //If the motor is clicked...
                   Brain.Screen.yPosition()>=y && Brain.Screen.yPosition()<=y+50 && 
                   !(port==LeftFront.index() || port==LeftMiddle.index() || port==LeftRear.index() || 
                   port==RightFront.index() || port==RightMiddle.index() || port==RightRear.index())){

                    motorPort=port; //Set the port to the motor port and tell the program to reassign
                    clickedOnMotor=true;
                }
            }
        }

        if(motorPort!=-1){
            if(screen==1401) LeftFront=vex::motor(motorPort,getMotorCartridge(LeftFront),isMotorReversed(LeftFront)); //Set the motor to whatever port was clicked
            if(screen==1402) LeftMiddle=vex::motor(motorPort,getMotorCartridge(LeftMiddle),isMotorReversed(LeftMiddle));
            if(screen==1403) LeftRear=vex::motor(motorPort,getMotorCartridge(LeftRear),isMotorReversed(LeftRear));
            if(screen==1404) RightFront=vex::motor(motorPort,getMotorCartridge(RightFront),isMotorReversed(RightFront));
            if(screen==1405) RightMiddle=vex::motor(motorPort,getMotorCartridge(RightMiddle),isMotorReversed(RightMiddle));
            if(screen==1406) RightRear=vex::motor(motorPort,getMotorCartridge(RightRear),isMotorReversed(RightRear));
            if(screen==1407) UpperIntake=vex::motor(motorPort,getMotorCartridge(UpperIntake),isMotorReversed(UpperIntake));
            if(screen==1408) MiddleIntake=vex::motor(motorPort,getMotorCartridge(MiddleIntake),isMotorReversed(MiddleIntake));
            if(screen==1409) LowerIntake=vex::motor(motorPort,getMotorCartridge(LowerIntake),isMotorReversed(LowerIntake));
        }

        if(clickedOnMotor) { screen-=100; setAllMotorPorts(); setMotorNames();};
    }    


    /* If the screen is set to motor config (1300)... */ else if(screen==1300){
        /* If Front Left motor pressed, set screen to 1301 */ if(Brain.Screen.xPosition()>=25 && Brain.Screen.yPosition()>=25 && Brain.Screen.xPosition()<=125 && Brain.Screen.yPosition()<=75){
                screen=1301;
                
            }
        /* If Middle Left motor pressed, set screen to 1302 */ else if(Brain.Screen.xPosition()>=150 && Brain.Screen.yPosition()>=25 && Brain.Screen.xPosition()<=250 && Brain.Screen.yPosition()<=75){
                screen=1302;
            }
        /* If Back Left motor pressed, set screen to 1303 */ else if(Brain.Screen.xPosition()>=275 && Brain.Screen.yPosition()>=25 && Brain.Screen.xPosition()<=375 && Brain.Screen.yPosition()<=75){
                screen=1303;
            }
        /* If Front Right motor pressed, set screen to 1304 */ else if(Brain.Screen.xPosition()>=25 && Brain.Screen.yPosition()>=154 && Brain.Screen.xPosition()<=125 && Brain.Screen.yPosition()<=204){
                screen=1304;
                
            }
        /* If Middle Right motor pressed, set screen to 1305 */ else if(Brain.Screen.xPosition()>=150 && Brain.Screen.yPosition()>=154 && Brain.Screen.xPosition()<=250 && Brain.Screen.yPosition()<=204){
                screen=1305;
            }
        /* If Back Right motor pressed, set screen to 1306 */ else if(Brain.Screen.xPosition()>=275 && Brain.Screen.yPosition()>=154 && Brain.Screen.xPosition()<=375 && Brain.Screen.yPosition()<=204){
                screen=1306;
            }
        /* If Lady Brown motor pressed, set screen to 1307 */ else if(Brain.Screen.xPosition()>=25 && Brain.Screen.yPosition()>=90 && Brain.Screen.xPosition()<=125 && Brain.Screen.yPosition()<=140){
                screen=1307;
            }
        /* If Lift motor pressed, set screen to 1308 */ else if(Brain.Screen.xPosition()>=150 && Brain.Screen.yPosition()>=90 && Brain.Screen.xPosition()<=250 && Brain.Screen.yPosition()<=140){
                screen=1308;
            }
        /* If Intake motor pressed, set screen to 1309 */ else if(Brain.Screen.xPosition()>=275 && Brain.Screen.yPosition()>=90 && Brain.Screen.xPosition()<=375 && Brain.Screen.yPosition()<=140){
                screen=1309;
            }
            
        else if(Brain.Screen.xPosition()>=380 && Brain.Screen.yPosition()>=180 ){
            screen=800;
        }
    }

    /* If the screen is set to auton selector (800)... */ if(screen==800){

        if(Brain.Screen.xPosition()>250 && Brain.Screen.xPosition()<390
        && Brain.Screen.yPosition()>20 && Brain.Screen.yPosition()<120){
            autonSelection="RightSide";
        }

        if(Brain.Screen.xPosition()>250 && Brain.Screen.xPosition()<390
        && Brain.Screen.yPosition()>130 && Brain.Screen.yPosition()<230){
            autonSelection="LeftSide";
        }
        if(Brain.Screen.xPosition()>20 && Brain.Screen.xPosition()<160
        && Brain.Screen.yPosition()>130 && Brain.Screen.yPosition()<230){
            autonSelection="MoveForward";
        }
        
    } 

    if(Brain.Screen.xPosition()>=420 &&
        Brain.Screen.yPosition()<=55){
        screen=1300;
    }

}

void motorReassignScreen(vex::motor selectedMotor){
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFont(mono40);
    Brain.Screen.setPenWidth(1);
    Brain.Screen.print(motorIndexes[selectedMotor.index()]);

    Brain.Screen.setFont(mono20);
    

    for(int ix=0; ix<7; ix++){
        for(int iy=0; iy<3; iy++){
            int port=ix+7*iy+1;
            Brain.Screen.setFillColor(red);
            Brain.Screen.setPenWidth(1);
            if(port-1==selectedMotor.index()){ Brain.Screen.setPenWidth(5); }

            if(port-1==LeftFront.index() || port-1==LeftMiddle.index() || port-1==LeftRear.index() || 
                    port-1==RightFront.index() || port-1==RightMiddle.index() || port-1==RightRear.index() || 
                    port-1==UpperIntake.index() || port-1==MiddleIntake.index() || port-1==LowerIntake.index()){
                        Brain.Screen.setFillColor(green);
                    }
            ;
            int x=(ix*68.5)+10;
            int y=(iy*66)+45;
            Brain.Screen.drawRectangle(x,y,50,50);
            Brain.Screen.setCursor((y+60)/20,(x+20)/10);
            Brain.Screen.print(port);
        }
    }

}


void drawAllUi(){

    Brain.Screen.clearScreen();
    /* If on motor config screen (1300)... */ if(screen==1300){

        Brain.Screen.clearScreen();//Clear the screen

        setMotorNames();

        displayMotor(LeftFront,25,25); //Display every motor
        displayMotor(LeftMiddle,150,25);
        displayMotor(LeftRear,275,25);        
        
        displayMotor(UpperIntake,25,90);
        displayMotor(MiddleIntake,150,90);
        displayMotor(LowerIntake,275,90);

        displayMotor(RightFront,25,154);
        displayMotor(RightMiddle,150,154);
        displayMotor(RightRear,275,154);

        Brain.Screen.setFont(mono60); //Large font
        Brain.Screen.setFillColor(black);
        Brain.Screen.setCursor(1,14);
        Brain.Screen.print(Brain.Battery.capacity(percent)); //Print the battery percentage
        Brain.Screen.setFont(mono20); //Reset font size
        Brain.Screen.setPenColor(white);
        Brain.Screen.setPenWidth(3);
        
        Brain.Screen.setFillColor(black);
        Brain.Screen.drawRectangle(380,180,90,50);
        Brain.Screen.setCursor(10,39);
        Brain.Screen.print("Auton ");
        Brain.Screen.setCursor(11,39);
        Brain.Screen.print("Selector");


            }
            /* If on motor viewer (1301-1306)... */ if(screen>1300 && screen<=1399 ) {
                Brain.Screen.setPenColor(0); //Draw the "x" to leave the screen
                Brain.Screen.setPenWidth(5);
                Brain.Screen.drawLine(465,19,440,39);
                Brain.Screen.drawLine(465,39,440,19);
                if(screen==1301){motorInfoScreen(LeftFront);} //Draw all motor screens
                if(screen==1302){motorInfoScreen(LeftMiddle);}
                if(screen==1303){motorInfoScreen(LeftRear);}
                if(screen==1304){motorInfoScreen(RightFront);}
                if(screen==1305){motorInfoScreen(RightMiddle);}
                if(screen==1306){motorInfoScreen(RightRear);}
                if(screen==1307){motorInfoScreen(UpperIntake);}
                if(screen==1308){motorInfoScreen(MiddleIntake);}
                if(screen==1309){motorInfoScreen(LowerIntake);}
            };

            /* If on emergency reassign (1401-1406)... */ if(screen>1400 && screen<=1499 ) {
                if(screen==1401){motorReassignScreen(LeftFront);}
                if(screen==1402){motorReassignScreen(LeftMiddle);}
                if(screen==1403){motorReassignScreen(LeftRear);}
                if(screen==1404){motorReassignScreen(RightFront);}
                if(screen==1405){motorReassignScreen(RightMiddle);}
                if(screen==1406){motorReassignScreen(RightRear);}
                if(screen==1407){motorReassignScreen(UpperIntake);}
                if(screen==1408){motorReassignScreen(MiddleIntake);}
                if(screen==1409){motorReassignScreen(LowerIntake);}
                Brain.Screen.setPenColor(0);
                Brain.Screen.setPenWidth(5);
                Brain.Screen.drawLine(465,15,440,25);
                Brain.Screen.drawLine(465,25,440,15);
            };

            if(screen==800){
                Brain.Screen.setPenColor(white);
                Brain.Screen.setPenWidth(3);
                Brain.Screen.setFillColor(black);
                
                if(autonSelection=="RightSide") Brain.Screen.setPenWidth(5);
                else Brain.Screen.setPenWidth(1);

                Brain.Screen.drawRectangle(250,20,140,100);

                if(autonSelection=="LeftSide") Brain.Screen.setPenWidth(5);
                else Brain.Screen.setPenWidth(1);

                Brain.Screen.drawRectangle(250,130,140,100);
                
                if(autonSelection=="MoveForward") Brain.Screen.setPenWidth(5);
                else Brain.Screen.setPenWidth(1);

                Brain.Screen.drawRectangle(20,130,140,100);
                
                Brain.Screen.setCursor(3,27);

                Brain.Screen.print("Left Side");

                Brain.Screen.setCursor(8,27);

                Brain.Screen.print("Right Side");

                Brain.Screen.setCursor(8,3);

                Brain.Screen.print("Go Forward");

                Brain.Screen.setPenColor(0);
                Brain.Screen.setPenWidth(5);
                Brain.Screen.drawLine(465,19,440,39);
                Brain.Screen.drawLine(465,39,440,19);
            }
        Brain.Screen.render();

            
}