#include <SPI.h>
#include <HighPowerStepperDriver.h>
#include "TeensyThreads.h"

uint8_t HomingPin = 32;
const uint8_t sleepPin = 14;
const uint8_t CSPinM1 = 29; //Pin 4 on nano every
const uint8_t CSPinM2 = 36; //Pin 4 on nano every
const uint8_t FaultPin = 30; //currently unused
const uint8_t StallPin = 31; //currently unused

const double degrees_per_step = 1.8;
const double microstep = 2; //2^1, n = 0 - 8 //was 4
const double degreesM1 = 20;
const double gear_ratio = 46.656; //old stepper motor gear ratio was 4.25

// This period is the length of the delay between steps, which controls the
// stepper motor's speed.  You can increase the delay to make the stepper motor
// go slower.  If you decrease the delay, the stepper motor will go faster, but
// there is a limit to how fast it can go before it starts missing steps.
const double StepPeriodUs = 400; //best value so far 800
const uint16_t StepPeriodMs = 1; //was 2

HighPowerStepperDriver sd1;
HighPowerStepperDriver sd2;

void setup()
{
  Serial.begin(9600);
  delay(1000);
  
  SPI.begin();
  
  //digitalWrite(SleepPin, HIGH);
  pinMode(HomingPin, INPUT);

   pinMode(sleepPin, OUTPUT);
   digitalWrite(sleepPin, HIGH);
  
  sd1.setChipSelectPin(CSPinM1); //when writing to a new CSPin, does this function set an already active CSPin low?
  Serial.println("Initializing driver");

  // Give the driver some time to power up.
  delay(1);

  // Reset the driver to its default settings and clear latched status
  // conditions.
  sd1.resetSettings();
  sd1.clearStatus();

  // Select auto mixed decay.  TI's DRV8711 documentation recommends this mode
  // for most applications, and we find that it usually works well.
  sd1.setDecayMode(HPSDDecayMode::AutoMixed);

  // Set the current limit. You should change the number here to an appropriate
  // value for your particular system.
  sd1.setCurrentMilliamps36v4(2800); //4000 max

  // Set the number of microsteps that correspond to one full step.

  Serial.print("microstep: ");
  Serial.println(microstep);
  Serial.print("step delay: ");
  Serial.print(StepPeriodUs);
  Serial.println(" microseconds");
  Serial.print("degrees of rotation: ");
  Serial.println(degreesM1);
  Serial.print("gear ratio: ");
  Serial.println(gear_ratio);
  double steps_approx = abs(degreesM1/(degrees_per_step/(gear_ratio*microstep)));
  Serial.print("Approximate steps per rotation: ");
  Serial.println(steps_approx);
  double time_approx = (steps_approx*StepPeriodUs)/1000000;
  Serial.print("Approximate seconds per rotation: ");
  Serial.println(time_approx);
  float rpm_approx = (60/time_approx)*degreesM1/float(360);
  Serial.print("Approximate rpm: ");
  Serial.println(rpm_approx);
  
  sd1.setStepMode(microstep);

  // Enable the motor outputs.
  
  sd1.enableDriver();
  Serial.println("Driver 1 enabled\n");




  sd2.setChipSelectPin(CSPinM2); //when writing to a new CSPin, does this function set an already active CSPin low?
  delay(1);
  sd2.resetSettings();
  sd2.clearStatus();
  sd2.setDecayMode(HPSDDecayMode::AutoMixed);
  sd2.setCurrentMilliamps36v4(2800); //4000 max
  sd2.setStepMode(microstep);
  sd2.enableDriver();
  Serial.println("Driver 2 enabled\n");

  
  
  pinMode(FaultPin, INPUT);
  pinMode(StallPin, INPUT);

  threads.addThread(turn_degrees_M1, 360);
  threads.addThread(turn_degrees_M2, -360);
}

void loop()
{

}

void turn_degrees_M1(int degrees){
  double steps = abs(degrees/(degrees_per_step/(gear_ratio*microstep)));
  if(degrees>=0){
    sd1.setDirection(0);
  }
  else{
    sd1.setDirection(1);
  }
  //Serial.print("steps: ");
  //Serial.println(steps);
  for(unsigned int x = 0; x <= steps; x++)
  {
    sd1.step();
    delayMicroseconds(StepPeriodUs);
  }
  //Serial.print("Rotation #");
  //Serial.println(rotations_counter);
}

void turn_degrees_M2(int degrees){
  double steps = abs(degrees/(degrees_per_step/(gear_ratio*microstep)));
  if(degrees>=0){
    sd2.setDirection(0);
  }
  else{
    sd2.setDirection(1);
  }
  //Serial.print("steps: ");
  //Serial.println(steps);
  for(unsigned int x = 0; x <= steps; x++)
  {
    sd2.step();
    //delay(StepPeriodMs); //use delayMicroseconds() for delays less than 1ms
    delayMicroseconds(StepPeriodUs);
  }
  //Serial.print("Rotation #");
  //Serial.println(rotations_counter);
}
