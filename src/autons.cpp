/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       mwedd                                                     */
/*    Created:      4/28/2025, 6:19:12 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <cmath>
#include <robot-config.h>

void WinPointLeft(){
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(forward,15,pct);
  UpperIntake.spin(forward,15,pct);
  Drive(28,20);
  wait(3,sec);
  LowerIntake.stop();
  MiddleIntake.stop();
  UpperIntake.stop();
  Turn(80);
  wait(25,msec);
  Tongue.set(true);
  wait(25,msec);
  Drive(13,55);
  wait(25,msec);
  LowerIntake.spin(forward,100,pct);
  MiddleIntake.spin(reverse,75,pct);
  UpperIntake.spin(reverse,85,pct);
}

void WinPointRight(){
  Drive (10,75);
}

void TournamentLeft(){
  Drive (10,75);
}

void TournamentRight(){
  Drive (10,75);
}

void Skills(){
  Drive (10,75);
}