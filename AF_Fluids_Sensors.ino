#include <Wire.h> //This is for i2C
#include <AccelStepper.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
#include <ezButton.h>
#include <AccelStepper.h>

//---------------------------------------------------------------------------
//Stepper Motor Stuff
int pulpin = 8; 
int dirpin = 9; 
AccelStepper stepper(1,pulpin,dirpin); // (mode, PUL,DIR) Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 
int set = 1000; // Stage speed during calibration 
//---------------------------------------------------------------------------
//Calculating Flow rate from Encoder
float pulses = 3200; // number of pulses per revolution set by the DM320T
float lead = 2 ; //T8*2 screw has 2mm lead
float stroke = 0; // how many mm to screw travels per step
float theory = 0;
float r = 6.67 / 2; // syringe radius in mm
float previousAngle = 0;
float angleChange;
float distance = 0; // Total distance travled in mm
float velocity = 0; // plunger velocity
float flowvolume = 0; // total fluid dispensed
float flowrate = 0; // volumetric flow rate
unsigned long previousTime = 0;
unsigned long sampleTime;
unsigned long timeChange;
float rawread; // Raw ADC Value for the Encoder
//---------------------------------------------------------------------------
//Magnetic sensor things
int magnetStatus = 0; //value of the status register (MD, ML, MH)
int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8
int rawAngle; //final raw angle
float degAngle; //raw angle in degrees (360/4096 * [value between 0-4095])
int quadrantNumber, previousquadrantNumber; //quadrant IDs
float numberofTurns = 0; //number of turns
float correctedAngle = 0; //tared angle - based on the startup value
float startAngle = 0; //starting angle
float totalAngle = 0; //total absolute angular displacement

//---------------------------------------------------------------------------
//Digital FIlter Things

float xn0 ; // Total Angle
float yn0;
float xn1 = 0;
float yn1 = 0;

float xnv ;  //Velocity
float ynv;
float xn1v = 0;
float yn1v = 0;

float xnd ; // Distance
float ynd;
float xn1d = 0;
float yn1d = 0;

float xnfr ; // Flowrate
float ynfr;
float xn1fr = 0;
float yn1fr = 0;

float xnp ; // Potentiometer
float ynp;
float xn1p = 0;
float yn1p = 0;

unsigned long sampleRate = 500; // cycle numbers for intervals to take speed measurements at
  // For sample rate 500 milliseconds works best the lowest allowable sample rate is 
bool newVelocity = false; // do not change this
int motspeed = 300;
//---------------------------------------------------------------------------
//Potentiometer Things
float pot;
int ynpint;
unsigned long time1;
unsigned long time0;
int16_t results;
float resultsmv;
float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */
int potint;
float potfloat;
float ynpfloat;
float SF;// the pot mV values when the Syringe is full
float ST = 27;// Desired Syringe travel from the limit siwtch to calibrate in mm
float rotate; 
float SFD; // the pot mV value when the syringe is fully dispensed
float mvtomm; // proportionality constant relating the pot voltage to displacement
unsigned long timestart;
unsigned long elasp;
unsigned long timelimit = 300000; // time limit to move the stage 5 minutes 
bool error = false; 
unsigned long caltime = 30000; // time to calibrate the starting position of the potentiometer 
float startdist; 
//---------------------------------------------------------------------------
//Limit Switch
ezButton limitSwitch(7);  // create ezButton object that attach to pin 7;
bool atswitch = false; 
//---------------------------------------------------------------------------
//Serial Communication 
String command; 



void setup()
{ //---------------------------------------------------------------------------
  //Setting up Stepper Motor
  stepper.setMaxSpeed(1000);
   
  //---------------------------------------------------------------------------
  //Setting up Limit Switch
   limitSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds
   
  //---------------------------------------------------------------------------
  //Setting up Serial and I2C
  Serial.begin(9600); //start serial - tip: don't use serial if you don't need it (speed considerations)
  Wire.begin(); //start i2C
  Wire.setClock(800000L); //fast clock
  
  //---------------------------------------------------------------------------
  //Setting up the Encoder
  checkMagnetPresence(); //check the magnet (blocks until magnet is found)
  delay(500); // This delay is important because it allows the correct start angle to be obtained after the shaft has initialzed and stablized
  
  //---------------------------------------------------------------------------
  //Set up ADC for Potentiometer
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }


}

void loop()
{ 
  if (Serial.available() > 0) {
    // Read the incoming command
    command = Serial.readStringUntil('\n');
    command.trim();}
    
  if (command.equals("calibrate")){
    //----------------------------------------------------------------------------
    // Calibate the Pot and the Encoder
    calibratepot(); 
    givecontrol(pulpin,dirpin);
    ReadRawAngle(); //make a reading so the degAngle gets updated
    startAngle = degAngle; //update startAngle with degAngle - for taring
    stroke = lead / pulses ; // how many mm to screw travels per step
    theory = motspeed * stroke; // Theoretical screw velocity in mm/s is the motor speed in steps/ second times the stroke
    time1 = millis();
    }
  else if(command.equals("take")){
    takecontrol(pulpin,dirpin);
  }
  else if(command.equals("give")){
    givecontrol(pulpin,dirpin);
  }

  else if(command.equals("start")){
    while(command.equals("start")){        
      //-------------------------------------------------------------------------------------------------------------
      // Functions 
      readADC(); // Read ADC value
      ReadRawAngle(); //ask the value from the sensor
      correctAngle(); //tare the value
      checkQuadrant(); //check quadrant, check rotations, check absolute angular position
      distancetraveled(); // Find distance traveled
    
      //----------------------------------------------------------------------------------------------------------------
      // Digital Filter Values
      // xn is the unfiltered signal
      // yn is the filtered signal
      xn1 = xn0;// Encoder Angular Distance
      yn1 = yn0;
    
      xn1v = xnv;// Encoder velocity 
      yn1v = ynv;
    
      xn1d = xnd;// Encoder Distance
      yn1d = ynd;
    
      xn1fr = xnfr; // Encoder Flow rate
      yn1fr = ynfr;
    
      xn1p = xnp; // Pot Value 
      yn1p = ynp;
    
      //---------------------------------------------------------------------------------------------------------------------
      // Outputs
      Serial.print(millis()); // time in millis
      Serial.print(" ");
      Serial.print(velocity,3); // Encoder Velocity Reading
      //Serial.print(" ");
      //Serial.print(rawread,3); // Raw digital input from the encoder 
      Serial.print(" ");
      Serial.print(ynv,3); // Filtered Encoder Velocity
      Serial.print(" ");
      Serial.print(theory,3); // Theretical Encoder Output based on given encoder speed variable motspeed 
      Serial.print(" ");
      Serial.print(mvtomm,3); // Conversion factor from potentiometer mV reading to location in mm
      Serial.print(" ");
      Serial.print(results); // Raw Pot reading
      Serial.print(" ");
      Serial.print(pot,3); // Unfiltered pot location 
      Serial.print(" ");
      Serial.println(ynp,3); // Filtered pot location
      if (Serial.available() > 0) { // Check for new command
        // Read the incoming command
        command = Serial.readStringUntil('\n');
        command.trim();}
     }
  }
  else if (command.equals("stop")) {
    Serial.println("stopped"); 
    }
}

void ReadRawAngle()
{
  //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor

  while (Wire.available() == 0); //wait until it becomes available
  lowbyte = Wire.read(); //Reading the data after the request

  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);

  while (Wire.available() == 0);
  highbyte = Wire.read();

  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  highbyte = highbyte << 8; //shifting to left
  //What is happening here is the following: The variable is being shifted by 8 bits to the left:
  //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
  //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in

  //Finally, we combine (bitwise OR) the two numbers:
  //High: 00001111|00000000
  //Low:  00000000|00001111
  //      -----------------
  //H|L:  00001111|00001111
  rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)
  rawread = rawAngle * 1.0;

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  degAngle = rawAngle * 0.087890625;

  // Calculate Velocity From the Encoder Value
  sampleTime = millis();
  timeChange = sampleTime - previousTime; // should always be positive
  if (timeChange >= sampleRate) {
    previousTime = sampleTime;
    newVelocity = true;
  }

}

void readADC(){
  // Read Potentiometer Value
  results = ads.readADC_Differential_0_1();
  time0=millis();
  pot = results*multiplier*(1/mvtomm);
  pot = pot - startdist; 
  // Filter the value
  xnp = pot;
  ynp = 0.728*yn1p + 0.136*xnp + 0.136*xn1p; // cutoff frequency of 50hz
  //ynp = pot; // no filter
  //ynp = 0.777*yn1p + 0.112*xnp + 0.112*xn1p; // cutoff frequency of 40hz
  //ynp = 0.823*yn1p + 0.086*xnp + 0.086*xn1p; // cutoff frequency of 30hz
  //ynp = 0.882*yn1p + 0.059*xnp + 0.059*xn1p; // cutoff frequency of 20hz
  //ynp = 0.939*yn1p + 0.03*xnp + 0.03*xn1p; // cutoff frequency of 10hz
  //ynp = 0.969*yn1p + 0.0155*xnp + 0.0155*xn1p; // cutoff frequency of 5hz
  //ynp= 0.985*yn1p + 0.01*xnp + 0.01*xn1p; // cutoff frequency of 4hz
  
  // round to the 0.001th place 
  ynpfloat = ynp*1000.0;
  ynpint = ynpfloat+0.5; // start 1.11153 * 1000 = 1111.53 + 0.5 = 1112.03 then turn to an so 1112 
  ynpfloat = ynpint;
  ynp = ynpfloat/1000.0; // divide by 1000 to get 1.1112 
}


void correctAngle()
{
  //recalculate angle
  correctedAngle = degAngle - startAngle; //this tares the position

  if (correctedAngle < 0) //if the calculated angle is negative, we need to "normalize" it
  {
    correctedAngle = correctedAngle + 360; //correction for negative numbers (i.e. -15 becomes +345)
  }
  else
  {
    //do nothing
  }
  //Serial.print("Corrected angle: ");
  //Serial.println(correctedAngle, 2); //print the corrected/tared angle
}

void checkQuadrant()
{
  /*
    //Quadrants:
    4  |  1
    ---|---
    3  |  2
  */

  //Quadrant 1
  if (correctedAngle >= 0 && correctedAngle <= 90)
  {
    quadrantNumber = 1;
  }

  //Quadrant 2
  if (correctedAngle > 90 && correctedAngle <= 180)
  {
    quadrantNumber = 2;
  }

  //Quadrant 3
  if (correctedAngle > 180 && correctedAngle <= 270)
  {
    quadrantNumber = 3;
  }

  //Quadrant 4
  if (correctedAngle > 270 && correctedAngle < 360)
  {
    quadrantNumber = 4;
  }
  //Serial.print("Quadrant: ");
  //Serial.println(quadrantNumber); //print our position "quadrant-wise"

  if (quadrantNumber != previousquadrantNumber) //if we changed quadrant
  {
    if (quadrantNumber == 1 && previousquadrantNumber == 4)
    {
      numberofTurns++; // 4 --> 1 transition: CW rotation
    }

    if (quadrantNumber == 4 && previousquadrantNumber == 1)
    {
      numberofTurns--; // 1 --> 4 transition: CCW rotation
    }
    //this could be done between every quadrants so one can count every 1/4th of transition

    previousquadrantNumber = quadrantNumber;  //update to the current quadrant

  }
  //Serial.print("Turns: ");
  //Serial.println(numberofTurns,0); //number of turns in absolute terms (can be negative which indicates CCW turns)
  //after we have the corrected angle and the turns, we can calculate the total absolute position
  totalAngle = (numberofTurns * 360) + correctedAngle; //number of turns (+/-) plus the actual angle within the 0-360 range
  xn0 = totalAngle;
  yn0 = 0.969 * yn1 + 0.0155 * xn0 + 0.0155 * xn1; // cutoff frequency of 5hz
  if (newVelocity) {
    angleChange = totalAngle - previousAngle;
    previousAngle = totalAngle;
  }
  //Serial.print("Total angle: ");
  //Serial.println(totalAngle, 2); //absolute position of the motor expressed in degree angles, 2 digits
}

void checkMagnetPresence()
{
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly
  while ((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading
    //Serial.println("checking1");

    Wire.beginTransmission(0x36); //connect to the sensor
    //Serial.println("checking2");
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    //Serial.println("checking3");
    Wire.endTransmission(); //end transmission
    //Serial.println("checking4");
    Wire.requestFrom(0x36, 1); //request from the sensor
    //Serial.println("checking5");

    while (Wire.available() == 0) { //wait until it becomes available
      //Serial.println("checking7");
    }
    magnetStatus = Wire.read(); //Reading the data after the request
    if ((magnetStatus & 16) == 16) {
      Serial.println("Too weak: Move Magnet Closer");
      Serial.println(magnetStatus);
    }
    if ((magnetStatus & 8) == 8) {
      Serial.println("Too strong: Move Magnet Away");
      Serial.println(magnetStatus);
    }
  }
  //Note that serial print does not display the actual value of magnetstatus (ie. it will not display 00100000 or 00010000 or 00001000)
  //However internally the arduino does read 00100000 or 00010000 or 00001000 and uses these values
  // I don't know why this happens.

  //Serial.println("Magnet found!");
  //delay(1000);

  //Status register output: 0 0 MD ML MH 0 0 0
  //MH: Too strong magnet
  //ML: Too weak magnet
  //MD: OK magnet

}

void distancetraveled() {
  //--------------------------------------------------------------------------
  // Calculate the Encoder Velocity and Linear distance traveled 
  distance = totalAngle * lead / 360; // 2mm lead = 2mm traveled per revolution
  if (newVelocity) {
    velocity = angleChange * lead * 1000 / (360 * timeChange); // mm/s
    newVelocity = false;
  }
  // Calculate Flow Volumes and Rates from the Encoder 
  flowvolume = PI * (r * r) * distance; // mm^3
  flowrate = PI * r * r * velocity; // mm^3/s

  // Let's filter velocity, Distance, and Flowrate
  xnv = velocity;
  ynv = 0.969 * yn1v + 0.0155 * xnv + 0.0155 * xn1v; // cutoff frequency of 5hz
  xnd = distance;
  ynd = 0.969 * yn1d + 0.0155 * xnd + 0.0155 * xn1d; // cutoff frequency of 5hz
  xnfr = flowrate;
  ynfr = 0.969*yn1fr + 0.0155*xnfr + 0.0155*xnfr; // cutoff frequency of 5hz

}

void calibratepot(){
  // This function runs the stage until it hits the limit switch and checks voltages and other values to calibrate the pot 
  Serial.println("Calibration Started"); 
  stepper.setSpeed(set); 
  // Check the state of the limit switch 
  // while the switch is not pressed state is high when untouched 
  timestart = millis();
  while(true){
    limitSwitch.loop(); // MUST call the loop() function first
    //Serial.println(limitSwitch.getState());
    // move towards it until it is presssed 
    stepper.runSpeed();
    // set bool atswitch = true;
    if (limitSwitch.getState()==LOW){
      break;
    }
    elasp = millis() - timestart; 
    if (elasp > timelimit){
      Serial.println("Error: Trying to move to the Limit Switch for calibration is taking too long, the stage may be damaged"); 
      break;
      error = true; 
    }
  }
  if (!error){
    Serial.println("No error"); 
    // else if the switch is pressed     
    // set bool atswitch = true; 
    if (limitSwitch.getState() == LOW){
      atswitch = true; 
    }
    // if atswitch
    if (atswitch){
      // read the current pot value 
      results = ads.readADC_Differential_0_1();
      // calculate the voltage float multiplier = 0.1875F; in mV 
      resultsmv = results*multiplier ; 
      // set that value to SF = Syringe full 
      SF = resultsmv; 
      // set the current angle to zero 
      delay(500);
      // This delay is important because it allows the correct start angle to be obtained after the shaft has initialzed and stablized
      ReadRawAngle(); //make a reading so the degAngle gets updated
      startAngle = degAngle; //update startAngle with degAngle - for taring
      correctAngle(); //tare the value
      checkQuadrant(); //check quadrant, check rotations, check absolute angular position output is totalAngle 
      // Move the stage forward for the amount of distance the syringe travels until it is empty (var is ST =  syringetravel (27mm usually) )
      stepper.setSpeed(-1*set); 
      // Turn the desired distance into an angle 
      rotate = ST*360/lead; // the number of degrees to rotate based on the desired travel 
      timestart = millis();
      while (totalAngle < rotate){
        //Serial.print(totalAngle); Serial.print(" "); Serial.println(rotate); 
        ReadRawAngle(); //make a reading so the degAngle gets updated
        correctAngle(); //tare the value
        checkQuadrant(); //check quadrant, check rotations, check absolute angular position output is totalAngle 
        stepper.runSpeed();
        elasp = millis() - timestart; 
        if (elasp > timelimit){
          Serial.println("Error: Trying to move to the Empty Syringe Position for calibration is taking too long, the stage may be damaged"); 
          break;
          error = true; 
        }
      }
      Serial.print(totalAngle); Serial.print(" "); Serial.println(rotate);
      // read current pot value 
      results = ads.readADC_Differential_0_1();
      // calculate the voltage float multiplier = 0.1875F; in mV 
      resultsmv = results*multiplier ; 
      // set that value to SFD = Syringe fully dispensed 
      SFD = resultsmv; 
      Serial.print("SF in mV: ");Serial.println(SF);
      Serial.print("SFD in mV: ");Serial.println(SFD);
      // calculate the propotionality constant between the pot voltage and displacement in mm 
          // PC = (SF - SFD)/ ST = mV/mm ( make sure to remove the decimal points as these contain noise ( ie turn to an int) 
          // ST = lead (mm/360)*totalAngle
      mvtomm = (SF-SFD)/(lead*totalAngle/360); 
      // Set Current position to 0 ( var is CurrentPos ) 
      // moving forward CurrentPos = PC * (potVoltage - SFD)
      timestart = millis();
      while(true){
      results = ads.readADC_Differential_0_1(); 
      pot = results*multiplier*(1/mvtomm);
      // Filter the value
      xnp = pot;
      ynp = 0.728*yn1p + 0.136*xnp + 0.136*xn1p; // cutoff frequency of 50hz
      xn1p = xnp; // Pot Value 
      yn1p = ynp;
        if(millis() - timestart > caltime){
          break; 
        }
      }
      // Set the distance when the syringe is fully dispensed 
      startdist = ynp; 
      Serial.print("Starting Distance in mm: "); Serial.println(startdist); 
      // Set the motor speed to 0 so it doesn;t cause trouble
      stepper.setSpeed(0); 
      Serial.println("Finished Calibration"); 
      // set atswitch to false
      atswitch = false; 
    }
    // else 
    else {
      // error the stage is not moving unable to calibrate Syringe 
      Serial.println("Error: Limit Switch Was Never Reached. Unable to move stage and Calibrate Syringe"); 
    }
  }

}


void takecontrol(int pul,int dir){
   pinMode(pul, OUTPUT); 
   pinMode(dir, OUTPUT); 
}
void givecontrol(int pul,int dir){
   pinMode(pul, INPUT); 
   pinMode(dir, INPUT); 
}
