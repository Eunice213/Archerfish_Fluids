#include <Wire.h> //This is for i2C
#include <SSD1306Ascii.h> //i2C OLED
#include <SSD1306AsciiWire.h> //i2C OLED
#include <AccelStepper.h>

AccelStepper stepper(1,8,9); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
// i2C OLED
#define I2C_ADDRESS 0x3C
#define RST_PIN -1
// #define PI 3.1415926535897932384626433832795
SSD1306AsciiWire oled;
float OLEDTimer = 0; //Timer for the screen refresh
//I2C pins:
//STM32: SDA: PB7 SCL: PB6
//Arduino: SDA: A4 SCL: A5

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
float previousAngle = 0; 
float angleChange;  
float previoustotalAngle = 0; //for the display printing
float distance = 0; // Total distance travled in mm 
float velocity = 0; // plunger velocity 
float flowvolume = 0; // total fluid dispensed 
float flowrate = 0; // volumetric flow rate 
float flowvolumeF = 0; // total fluid dispensed 
float flowrateF = 0;
float r = 6.67/2; // syringe radius in mm 
unsigned long previousTime = 0; 
unsigned long sampleTime;  
unsigned long timeChange;

float xn0 ;
float yn0;
float xn1 = 0;
float yn1 = 0;

float xnv ;
float ynv;
float xn1v = 0;
float yn1v = 0;

float xnd ;
float ynd;
float xn1d = 0;
float yn1d = 0;

float xnfr ;
float ynfr;
float xn1fr = 0;
float yn1fr = 0;

int k = 0;
unsigned long sampleRate = 500; // cycle numbers for intervals to take speed measurements at
bool newVelocity = false; // do not change this 
int motspeed = -50; 
void setup()
{
 
  Serial.begin(9600); //start serial - tip: don't use serial if you don't need it (speed considerations)
  Wire.begin(); //start i2C  
  Wire.setClock(800000L); //fast clock

  checkMagnetPresence(); //check the magnet (blocks until magnet is found)
  
  stepper.setMaxSpeed(1000);
  stepper.setSpeed(motspeed); 
  delay(500);
  // This delay is important because it allows the correct start angle to be obtained after the shaft has initialzed and stablized
  ReadRawAngle(); //make a reading so the degAngle gets updated
  startAngle = degAngle; //update startAngle with degAngle - for taring

  //------------------------------------------------------------------------------
  //OLED part
  #if RST_PIN >= 0
    oled.begin(&Adafruit128x32, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
    oled.begin(&Adafruit128x32, I2C_ADDRESS);
  #endif // RST_PIN >= 0

  oled.setFont(Adafruit5x7);
  oled.clear(); //clear display
  oled.set1X(); //double-line font size - better to read it
  oled.println("Welcome!"); //print a welcome message  
  oled.println("AS5600"); //print a welcome message
  delay(10000);
  OLEDTimer = millis(); //start the timer 
  
  
}

void loop()
{    
    ReadRawAngle(); //ask the value from the sensor
    correctAngle(); //tare the value
    checkQuadrant(); //check quadrant, check rotations, check absolute angular position
    distancetraveled(); // Find distance traveled
    refreshDisplay(); // Display distance traveled
    xn1 = xn0;
    yn1 = yn0;

    xn1v = xnv;
    yn1v = ynv;

    xn1d = xnd;
    yn1d = ynd;

    xn1fr = xnfr;
    yn1fr = ynfr;
    // xn is the unfiltered signal 
    // yn is the filtered signal
    //if(k % 3 == 0){
    // This extra conditional statement is here to reduce
    // the number of times the data is sent through the serial port
    // because sending data through the serial port
    // messes with the sampling frequency
  
    // Output
    Serial.print(velocity);
    Serial.print(" ");
    Serial.println(ynv);
    //Serial.print(" ");
    //Serial.println(flowrate);
    //lnSerial.print(" ");
    //Serial.print(flowrateF);
    //Serial.print(" ");
    //Serial.println(ynfr);
  //}
  //k = k+1;
   
  stepper.runSpeed();
    

    //Serial.print("Total Angle");

  //delay(100); //wait a little - adjust it for "better resolution"

}

void ReadRawAngle()
{ 
  //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor
  
  while(Wire.available() == 0); //wait until it becomes available 
  lowbyte = Wire.read(); //Reading the data after the request
 
  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  
  while(Wire.available() == 0);  
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

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  degAngle = rawAngle * 0.087890625;
  sampleTime = millis();  
  timeChange = sampleTime - previousTime; // should always be positive
  if (timeChange >= sampleRate) {
    previousTime = sampleTime; 
    //Serial.println("timeChange"); 
    //Serial.println(timeChange);
    newVelocity = true; 
  }
  
  //Serial.print("Deg angle: ");
  //Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle
  
}

void correctAngle()
{
  //recalculate angle
  correctedAngle = degAngle - startAngle; //this tares the position

  if(correctedAngle < 0) //if the calculated angle is negative, we need to "normalize" it
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
  if(correctedAngle >= 0 && correctedAngle <=90)
  {
    quadrantNumber = 1;
  }

  //Quadrant 2
  if(correctedAngle > 90 && correctedAngle <=180)
  {
    quadrantNumber = 2;
  }

  //Quadrant 3
  if(correctedAngle > 180 && correctedAngle <=270)
  {
    quadrantNumber = 3;
  }

  //Quadrant 4
  if(correctedAngle > 270 && correctedAngle <360)
  {
    quadrantNumber = 4;
  }
  //Serial.print("Quadrant: ");
  //Serial.println(quadrantNumber); //print our position "quadrant-wise"

  if(quadrantNumber != previousquadrantNumber) //if we changed quadrant
  {
    if(quadrantNumber == 1 && previousquadrantNumber == 4)
    {
      numberofTurns++; // 4 --> 1 transition: CW rotation
    }

    if(quadrantNumber == 4 && previousquadrantNumber == 1)
    {
      numberofTurns--; // 1 --> 4 transition: CCW rotation
    }
    //this could be done between every quadrants so one can count every 1/4th of transition

    previousquadrantNumber = quadrantNumber;  //update to the current quadrant
  
  }  
  //Serial.print("Turns: ");
  //Serial.println(numberofTurns,0); //number of turns in absolute terms (can be negative which indicates CCW turns)  

  //after we have the corrected angle and the turns, we can calculate the total absolute position
  totalAngle = (numberofTurns*360) + correctedAngle; //number of turns (+/-) plus the actual angle within the 0-360 range 
  xn0 = totalAngle;
  yn0 = 0.969*yn1 + 0.0155*xn0 + 0.0155*xn1; // cutoff frequency of 5hz 
  if (newVelocity) {  
    angleChange = totalAngle - previousAngle;
    previousAngle = totalAngle; 
    //Serial.println("angleChange");
    //Serial.println(angleChange);
  }

  //Serial.print("Total angle: ");
  //Serial.println(totalAngle, 2); //absolute position of the motor expressed in degree angles, 2 digits
}

void checkMagnetPresence()
{  
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

  while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading
    Serial.println("checking1");
    

    Wire.beginTransmission(0x36); //connect to the sensor
    Serial.println("checking2");
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Serial.println("checking3");
    Wire.endTransmission(); //end transmission
    Serial.println("checking4");
    Wire.requestFrom(0x36, 1); //request from the sensor
    Serial.println("checking5");

    while(Wire.available() == 0){ //wait until it becomes available 
    Serial.println("checking7");
    }
    magnetStatus = Wire.read(); //Reading the data after the request
    if ((magnetStatus & 16)==16){
        Serial.println("Too weak: Move Magnet Closer");
        Serial.println(magnetStatus); 
        } 
        if ((magnetStatus & 8)==8){
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

void distancetraveled(){
  distance = totalAngle*2/360; // 2mm lead = 2mm traveled per revolution 
  if (newVelocity){
    velocity = angleChange*2*1000/(360*timeChange); // mm/s 
    //Serial.println("angleChange " + String(angleChange,3)+" timeChange "+String(timeChange) + " Velocity " + String(velocity,5));
    newVelocity = false; 
  } 
    // Let's filter velocity and Distance too
  xnv = velocity; 
  ynv = 0.969*yn1v + 0.0155*xnv + 0.0155*xn1v; // cutoff frequency of 5hz 

  xnd = distance; 
  ynd = 0.969*yn1d + 0.0155*xnd + 0.0155*xn1d; // cutoff frequency of 5hz 
 
  //Serial.println("angleChange:");
  //Serial.println(angleChange);
  //Serial.println("timeChange");
  //Serial.println(timeChange);
  flowvolume = PI*(r*r)*distance; // mm^3 
  flowrate = PI*r*r*velocity; // mm^3/s 
  flowvolumeF = PI*(r*r)*ynd; // mm^3 
  flowrateF = PI*r*r*ynv; // mm^3/s
  xnfr = flowrate; 
  //ynfr = 0.969*yn1fr + 0.0155*xnfr + 0.0155*xnfr; // cutoff frequency of 5hz 
  ynfr = 0.9752*yn1fr + 0.0124*xnfr + 0.0124*xnfr; // cutoff frequency of 4hz 
}

void refreshDisplay()
{
  if (millis() - OLEDTimer > 100) //chech if we will update at every 100 ms                                                                  
  { 
    if(totalAngle != previoustotalAngle) //if there's a change in the position*
    {
        oled.clear(); //delete the content of the display
        oled.println(String(totalAngle) + " DEG"); //print the new absolute position 
        oled.println(String(distance) + " mm"); //print the new absolute distance
        oled.println(String(velocity,5) + " mm/s"); 
        OLEDTimer = millis(); //reset timer   
        previoustotalAngle = totalAngle; //update the previous value
        //Serial.println(totalAngle);
    }
  }
  else
  {
    //skip
  }
  //*idea: you can define a certain tolerance for the angle so the screen will not flicker
  //when there is a 0.08 change in the angle (sometimes the sensor reads uncertain values)
} 
