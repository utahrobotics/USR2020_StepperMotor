#include <SPI.h>
#include <HighPowerStepperDriver.h>
#include "TeensyThreads.h"

const uint8_t CSPinM1 = 10; //Pin 4 on nano every
const uint8_t CSPinM2 = 9;
const uint8_t FaultPin = 2; //currently unused
const uint8_t StallPin = 2; //currently unused

const double degrees_per_step = 1.8;
const double microstep = 2; //2^1, n = 0 - 8 //was 4
const double degreesM1 = 360;
const double degreesM2 = 360;
const double gear_ratio = 46.656; //old stepper motor gear ratio was 4.25

// This period is the length of the delay between steps, which controls the
// stepper motor's speed.  You can increase the delay to make the stepper motor
// go slower.  If you decrease the delay, the stepper motor will go faster, but
// there is a limit to how fast it can go before it starts missing steps.
const double StepPeriodUs = 500; //best value so far 800
const uint16_t StepPeriodMs = 1; //was 2

HighPowerStepperDriver sd;

void setup()
{
  Serial.begin(9600);
  delay(1000);
  
  SPI.begin();
  
  digitalWrite(SleepPin, HIGH);
  sd.setChipSelectPin(CSPinM1); //when writing to a new CSPin, does this function set an already active CSPin low?
  Serial.println("Initializing driver");

  // Give the driver some time to power up.
  delay(1);

  // Reset the driver to its default settings and clear latched status
  // conditions.
  sd.resetSettings();
  sd.clearStatus();

  // Select auto mixed decay.  TI's DRV8711 documentation recommends this mode
  // for most applications, and we find that it usually works well.
  sd.setDecayMode(HPSDDecayMode::AutoMixed);

  // Set the current limit. You should change the number here to an appropriate
  // value for your particular system.
  sd.setCurrentMilliamps36v4(2800); //4000 max

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
  
  sd.setStepMode(microstep);

  // Enable the motor outputs.
  
  sd.enableDriver();
  Serial.println("Driver enabled\n");
  
  pinMode(FaultPin, INPUT);
  pinMode(StallPin, INPUT);
}

void loop()
{
  //float t1 = millis();
  turn_degrees(degreesM1);
  /*
  float t2 = millis();
  float delta_t = (t2 - t1)/1000;
  float rpm = (60/delta_t)*degreesM1/float(360);
  Serial.print("seconds per rotation: ");
  Serial.println(delta_t);
  Serial.print("rpm: ");
  Serial.println(rpm);
  */
  // Wait for 3 s
  delay(3000);


  //t1 = millis();
  turn_degrees(-degreesM1);
  /*
  t2 = millis();
  delta_t = (t2 - t1)/1000;
  rpm = (60/delta_t)*degreesM1/float(360);
  Serial.print("seconds per rotation: ");
  Serial.println(delta_t);
  Serial.print("rpm: ");
  Serial.println(rpm);
  */
  // Wait for 3 s
  delay(3000);
  /*
  Serial.print("Total time elapsed: ");
  Serial.print(millis()/1000);
  Serial.println(" seconds\n");
  */
}

void turn_degrees(int degrees){
  double steps = abs(degrees/(degrees_per_step/(gear_ratio*microstep)));
  if(degrees>=0){
    sd.setDirection(0);
  }
  else{
    sd.setDirection(1);
  }
  //Serial.print("steps: ");
  //Serial.println(steps);
  for(unsigned int x = 0; x <= steps; x++)
  {
    sd.step();
    //delay(StepPeriodMs); //use delayMicroseconds() for delays less than 1ms
    delayMicroseconds(StepPeriodUs);
  }
  //Serial.print("Rotation #");
  //Serial.println(rotations_counter);
}
