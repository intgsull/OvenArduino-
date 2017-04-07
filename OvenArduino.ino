//Startup: set up lcd screen to accept inputs:
// time desirec <<
// Temp desired <<
//once entered begin the pid algorithm to get to set point
// have lcd monitor show the elapsed time in minutes and
// the temperature in celsius, which can be toggled to each view
// have a stop function if it wants to be terminated early.
//this will turn oven off and go back to home screen
//where the time and temp can be chosen again





#include "Adafruit_MAX31855.h"
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>

//Standard Libraries
#include <SPI.h>
#include <Wire.h>
#include "PID_AutoTune_v0.h"
#include "PID_v1.h"
//Variables
double temp = -1; //stores temperature
int switcher; //stores option from potentiometer
int potVal; //stores value of potentiometer
int ovenStatus = -1; //will be 1 for on, 0 for off
int valveStatus = -1;
double tempDesired = 80; //celsius
int timeDesired = 120; //minutes
int timer = 0;
unsigned long initiation = 0;
unsigned long startTime = 0;
double time = 0;
unsigned long holdTime = 0;
double minutes = 0;
double PIDoutput = 0;
unsigned long timeElapsed = 0;

//Analog inputs
int potPin = A0; //A0 pin goes to pot

//Digital outputs
int ovenRelay = 6; //powerswitch tail attaches to pin 6
int valveRelay = 5; //valve relay goes to pin 5

//Thermocouple pins are 3, 4, 5
#define MAXDO   5
#define MAXCS   4
#define MAXCLK  3

Adafruit_MAX31855 thermocouple(MAXDO, MAXCS, MAXCLK);
//Adafruit_7segment matrix = Adafruit_7segment();

//auxilliary functions
void disp();
void grabpot();
void heat();
void switchOven(int trigger);
void generateKeyTable(int vcc, int* array);
unsigned char GetKey(int value);

//pins for lcd, check manual 
LiquidCrystal lcd(12,13,8,9,10,11);
#define Rbase ((unsigned long)10)
#define Rup ((unsigned long)100)
#define Rdown ((unsigned long)75)
#define Rleft ((unsigned long)51)
#define Rright ((unsigned long)39)
#define Rselect ((unsigned long)15)
int sensorValue;
int KeyTable[5];
int relayCount = 0;

//PID constants
double kp=10000;
double ki=0 ;
double kd= 2000;
boolean tuning = true;

PID myPID(&temp, &PIDoutput, &tempDesired, kp, ki, kd, DIRECT);
PID_ATune aTune(&temp, &PIDoutput);

void setup() {
  int tmpInt;
  //newdisplay
  lcd.begin(16,2);
//lcd.print("Heat Cycling");
  sensorValue = 1023;
  Serial.begin(9600); //initialize serial port
  //displays
  //matrix.begin(0x70); //initialize i2c bus at address 0x70 for the led display

  generateKeyTable(analogRead(A0),KeyTable); //generate key table for current VCC (should be 5V)
  for(tmpInt = 0; tmpInt < 5; tmpInt++)
  {
    Serial.print(KeyTable[tmpInt]); //Serial print key table
  }

  //define pinmodes
  pinMode(potPin, INPUT);
  pinMode(ovenRelay, OUTPUT);
  pinMode(valveRelay, OUTPUT);

  delay(2000);

  //set everything off first
  switchOven(0);
  digitalWrite(5, LOW);
 
  lcd.clear();


 ////Thermocouple test/////
 lcd.print("MAX31855 test");
 delay(1000);
 
 lcd.clear();
   lcd.setCursor(0, 0);
   lcd.print("Int. Temp = ");
   lcd.println(thermocouple.readInternal());
   Serial.print("Int. Temp = ");
   Serial.print(thermocouple.readInternal());
     
   double c = thermocouple.readCelsius();
   lcd.setCursor(0, 1);
   if (isnan(c)) 
   {
     lcd.print("T/C Problem");
     Serial.print(thermocouple.readError());
   } 
   else 
   {
     lcd.print("C = "); 
     lcd.print(c);
     lcd.print("  "); 
     Serial.print("Thermocouple Temp = *");
     Serial.print(c);
   }
 
   delay(10000);
   Serial.end();
 
}
void loop() {
lcd.clear();
lcd.setCursor(0,0);
lcd.print("Heat Cycling");
delay (5000);
lcd.clear();
    
/*   
  
  ////////////////Set Temperature//////////////////////
  int firstDigit = 0;
  lcd.setCursor(0,0);
  lcd.print("Temperature:***");
  lcd.setCursor(0,1);
  lcd.print(firstDigit);
  lcd.setCursor(0,1);
  
  while(analogRead(A0) > 625 )
  {
    if(analogRead(A0) != sensorValue)
    {
      delay(50);
    }
    if( analogRead(A0) < 940 && analogRead(A0) > 920)
    {
      firstDigit++;
      lcd.print(firstDigit);
      lcd.setCursor(0,1);
      delay(100);
    }//increment
    if( analogRead(A0) < 912 && analogRead(A0) > 890 && firstDigit != 0)
    {
      firstDigit--;
      lcd.print(firstDigit);
      lcd.setCursor(0,1);
      delay(100);
    }//decrement
  }
  
  tempDesired = firstDigit;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Ok!");
  lcd.setCursor(0,1);
  lcd.print("Temp =");
  lcd.print(tempDesired);
  delay(2000);
  lcd.clear();
  
////////////////Set Time Desired//////////////////////
 firstDigit = 0;
// secondDigit = 0;
//thirdDigit = 0;
  lcd.setCursor(0,0);
  lcd.print("Time(Mins):***");
  lcd.setCursor(0,1);
  lcd.print(firstDigit);
 // lcd.print(secondDigit);
 // lcd.print(thirdDigit);
  lcd.setCursor(0,1);
  
  while(analogRead(A0) > 625 )
  {
    if(analogRead(A0) != sensorValue)
    {
      delay(50);
    }
    if( analogRead(A0) < 940 && analogRead(A0) > 920)
    {
      firstDigit++;
      lcd.print(firstDigit);
      lcd.setCursor(0,1);
      delay(100);
    }//increment
    if( analogRead(A0) < 912 && analogRead(A0) > 890 && firstDigit != 0)
    {
      firstDigit--;
      lcd.print(firstDigit);
      lcd.setCursor(0,1);
      delay(100);
    }//decrement
  }
   
  lcd.clear();
  timeDesired = firstDigit;
  lcd.setCursor(0,0);
  lcd.print("Ok!");
  lcd.setCursor(0,1);
  lcd.print("Time =");
  lcd.print(timeDesired);
  delay(3000);
  lcd.clear();

*/

/////Testing/////////
tempDesired = 90;
timeDesired = 10;


  //start oven, get close to oven temp desired

  temp = thermocouple.readCelsius();
  Serial.begin(9600);
  while (temp < .925 * tempDesired) {
    heat();
    Serial.println(temp);
    delay(5000);
    }  
/////////////////PID loop////////////////////////////
int WindowSize = 10000;
unsigned long windowStartTime;
windowStartTime = millis();
myPID.SetOutputLimits(0, WindowSize);
myPID.SetSampleTime(200);
myPID.SetMode(AUTOMATIC);

unsigned long switchtime = 0;
unsigned long timeout = 5000;
int lastOvenState = 1;

////PID loop///
//autotune//

lcd.print(tempDesired); 
delay(3000);
lcd.clear();
startTime = millis();
holdTime = millis() - startTime;

while (holdTime < (timeDesired * 60000))
{
timeElapsed = holdTime/60000; 
temp = thermocouple.readCelsius();
lcd.setCursor(0,0);
lcd.print("temp:");
lcd.print(temp);
lcd.setCursor(0,1);
lcd.print("time:");
lcd.print(timeElapsed);
myPID.Compute();

/////regulate analog PID to digital output
unsigned long now = millis();
if((now - switchtime) < timeout) continue; 

if((now - windowStartTime)>WindowSize)
{
windowStartTime += WindowSize;
Serial.println(temp);
}

if(PIDoutput > (now - windowStartTime)) 
{
  digitalWrite(ovenRelay, LOW);
  relayCount += 1; /* monitor how many times we switch states */
  if (lastOvenState = 0)
  {
    lastOvenState = 1; 
    switchtime = millis();
  }
}
else {
  digitalWrite(ovenRelay, HIGH);
  if (lastOvenState = 1)
  {
    lastOvenState = 0;
    switchtime = millis();
  }
}

/*AutoTune Maybe
 * if(tuning)
{
  byte val = (aTune.Runtime())
  if (val!=0){
  tuning = false;
  }
  if (!tuning)
  {
  kp = aTune.GetKp();
  ki = aTune.GetKi();
  kd = aTune.GetKd();
  myPID.SetTunings(kp,ki,kd);
  }
 */
 holdTime = millis() - startTime;

}
 Serial.println(relayCount);
 digitalWrite(ovenRelay, HIGH);
    ovenStatus = 0;
 lcd.clear();
 lcd.setCursor(0,0);
 lcd.print("Done");
 lcd.setCursor(0,1);
 lcd.print("Wait 10 secs");
 delay(10000);
}









//grabs potentiometer value and assigns it an integer
void grabPot() {
  potVal = analogRead(potPin);
  // most of left half
  // show temp
  if (potVal >= 5 && potVal < 512) {
    switcher = 1;
  }
  // right half
  // shows time
  else if (potVal >= 512 && potVal <= 1023) {
    switcher = 2;
  }
  // all the way left, turn off permanently
  else if (potVal < 5) {
    switcher = 3;
  }
}

//display function
/*
 void disp() {
  temp = thermocouple.readCelsius();
  grabPot();
  switch (switcher) {
    case 1:
      if (isnan(temp)) {
        matrix.print(-1);
        matrix.writeDisplay();
      }
      else {
        matrix.print(temp);
        matrix.writeDisplay();
      }
      break;
    case 2:
      time = millis();
      minutes = time / 60000;

      matrix.print(minutes);
      matrix.writeDisplay();
      break;
    case 3:
      while (switcher = 3) {
        switchOven(0);
        grabPot();
      }
      break;
  }

  time = millis();
  minutes = time / 60000;

  // For outputting to logger
//  Serial.print(minutes); //outputs the time elapsed in minutes
//  Serial.print(",");
//  Serial.print(holdTime/60000); // outputs the time it has been at temp
//  Serial.print(",");
//  if (isnan(temp)) {
//    Serial.print("nan"); //outputs "nan" if failure to read temp
//  }
//  else {
//    Serial.print(temp); //if all good, outputs temp
//  }
//  Serial.print(",");
//  Serial.println(ovenStatus); //reads 1 when oven on, 0 when off

  delay(500);
}
*/

//switches on/off oven
void switchOven(int trigger) {
  if (trigger) {
    digitalWrite(ovenRelay, LOW);
    ovenStatus = 1;
  }
  else {
    digitalWrite(ovenRelay, HIGH);
    ovenStatus = 0;
  }
}


//Generates key table for single button presses.
void generateKeyTable(int vcc, int* array)
{
  float resistor;
  /////////1 key///////////////////
  //up is keytable[0]
  resistor = ((float)Rup)/(Rbase + Rup);
  *array++ = resistor*vcc; 
  //down is keytable[1]
  resistor = ((float)Rdown)/(Rbase + Rdown);
  *array++ = resistor*vcc;
  //left is keytable[2]
  resistor = ((float)Rleft)/(Rbase + Rleft);
  *array++ = resistor*vcc;
  //right is keytable[3]
  resistor = ((float)Rright)/(Rbase + Rright);
  *array++ = resistor*vcc;
  //select is keytable[4]
  resistor = ((float)Rselect)/(Rbase + Rselect);
  *array++ = resistor*vcc;
  
}

unsigned char GetKey(int value)
{
  char tmpChar;
  unsigned int Rst;
  tmpChar = 0;
  do{
      if(value > KeyTable[tmpChar]) 
        Rst = value - KeyTable[tmpChar];
        else Rst = KeyTable[tmpChar] - value;
  }while(Rst > 1);

  return tmpChar--;
}



// binary turn oven on/off based on temp
void heat() {
  temp = thermocouple.readCelsius();
  if (temp > 0 && temp < tempDesired) {
    switchOven(1);
  }
  else {
    switchOven(0);
  }
}
