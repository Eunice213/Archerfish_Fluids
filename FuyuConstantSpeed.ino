// ConstantSpeed.pde
// -*- mode: C++ -*-
//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
/// \author  Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2009 Mike McCauley
// $Id: ConstantSpeed.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $
//--------------------------------------------------------------------------------
// Settin gup the Motors 
#include <AccelStepper.h>
int pulpin = 8; 
int dirpin = 9; 
AccelStepper stepper(1,pulpin,dirpin); // (mode, PUL,DIR) Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
// In the future this code can be told how many motors it has at which pins 
// int num_motors = 1; // number of motors controlled by the arduino 

//----------------------------------------------------------------------------------------
// Motor Movement 
bool ramping = false;
float ramp = 5000; // time in ms
float base = 0; 
float x = 0;
int set = 100;
unsigned long start; 


//----------------------------------------------------------------------------------
// Serial Communication 
String command ;

void setup()
{  Serial.begin(9600); //start serial - tip: don't use serial if you don't need it (speed considerations)
   stepper.setMaxSpeed(1000);
   stepper.setSpeed(set);	
   start = millis();
}

void loop(){   
  if (Serial.available() > 0) {
    // Read the incoming command
    command = Serial.readStringUntil('\n');
    command.trim();}
    
  if (command.equals("ramp")){
      if ((millis() - start)>= ramp && x<= 10){
      stepper.setSpeed(base*x); 
      x+=1 ; 
      start = millis();
      }
    }
  else if (command.equals("stop")) {
    stepper.setSpeed(0);
    }
  else if (command.equals("constant")) {
    stepper.setSpeed(set);
    }
  else if (command.equals("take")) {
    takecontrol(pulpin,dirpin);
  }
  else if (command.equals("give")) {
    givecontrol(pulpin,dirpin);
  }  
   stepper.runSpeed();
}

void takecontrol(int pul,int dir){
   pinMode(pul, OUTPUT); 
   pinMode(dir, OUTPUT); 
}
void givecontrol(int pul,int dir){
   pinMode(pul, INPUT); 
   pinMode(dir, INPUT); 
}
