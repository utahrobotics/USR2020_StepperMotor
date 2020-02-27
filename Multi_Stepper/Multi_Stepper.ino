#include <SPI.h>
#include "HighPowerStepperDriver.h"
#include "TeensyThreads.h"

#define MOTOR_COUNT 2

const uint8_t CSPinM1 = 10; //Pin 4 on nano every
const uint8_t CSPinM2 = 9;
const uint8_t FaultPin = 2; //currently unused
const uint8_t StallPin = 2; //currently unused
const uint8_t HomingPin = 7;

const double degrees_per_step = 1.8;
const double microstep = 4; //2^1, n = 0 - 8 //was 4
const double degreesM0 = 180;
const double degreesM1 = -160;
const double gear_ratio = 46.656; //old stepper motor gear ratio was 4.25

const double StepPeriodUs = 200; //best value so far 800
const uint16_t StepPeriodMs = 1; //was 2

double stepper_degrees[MOTOR_COUNT] = {degreesM0, degreesM1};
double stepper_degrees_R[MOTOR_COUNT] = {-degreesM0, -degreesM1};

HighPowerStepperDriver sd0;
HighPowerStepperDriver sd1;

HighPowerStepperDriver sd[MOTOR_COUNT] = {sd0, sd1};
HighPowerStepperDriver *sd_ptr[MOTOR_COUNT] = {&sd0, &sd1};

void setup()
{
  Serial.begin(9600);
  delay(1000);
  
  SPI.begin();

  pinMode(HomingPin,INPUT);
  //pinMode(SleepPin, OUTPUT);
  //digitalWrite(SleepPin, HIGH);
  sd0.setChipSelectPin(CSPinM1);
  sd1.setChipSelectPin(CSPinM2);
  Serial.println("Initializing driver");

  // Give the driver some time to power up.
  delay(1);

  sd0.resetSettings();
  sd0.clearStatus();
  sd1.resetSettings();
  sd1.clearStatus();

  sd0.setDecayMode(HPSDDecayMode::AutoMixed);
  sd1.setDecayMode(HPSDDecayMode::AutoMixed);

  sd0.setCurrentMilliamps36v4(2800); //steppers are rated for 2.8 max continuous current
  sd1.setCurrentMilliamps36v4(2800);

  //Print Stepper config data, such as microsteps, step delay, gear ratio, and predicted RPM
  Serial.print("microstep: ");
  Serial.println(microstep);
  Serial.print("step delay: ");
  Serial.print(StepPeriodUs);
  Serial.println(" microseconds");
  Serial.print("degrees of rotation: ");
  Serial.println(degreesM0);
  Serial.print("gear ratio: ");
  Serial.println(gear_ratio);
  double steps_approx = abs(degreesM0/(degrees_per_step/(gear_ratio*microstep)));
  double time_approx = (steps_approx*StepPeriodUs)/1000000;
  Serial.print("Approximate seconds per rotation: ");
  Serial.println(time_approx);
  float rpm_approx = (60/time_approx)*degreesM0/float(360);
  Serial.print("Approximate rpm: ");
  Serial.println(rpm_approx);
  Serial.print("Motor rotations (in degrees): ");
  for(int i=0;i < MOTOR_COUNT; i++){
    Serial.print(stepper_degrees[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  // Set the number of microsteps that correspond to one full step.
  for(int i=0; i < MOTOR_COUNT; i++){
    sd[i].setStepMode(microstep);
  }

  // Enable the motor outputs.
  for(int i=0; i < MOTOR_COUNT; i++){
    sd[i].enableDriver();
  }
  Serial.println("Drivers enabled\n");
  
  //pinMode(FaultPin, INPUT);
  //pinMode(StallPin, INPUT);
}

void loop()
{
  Serial.println("stepping...");
  turn_degrees(sd_ptr, stepper_degrees);
  Serial.println("Steppers reached targets");
  Serial.println();

  // Wait for 3 s
  delay(3000);
  Serial.println("stepping reverse...");
  turn_degrees(sd_ptr, stepper_degrees_R);
  Serial.println("Steppers reached targets");
  Serial.println();

  // Wait for 3 s
  delay(3000);
}

void turn_degrees(HighPowerStepperDriver *sd_ptr[], double stepper_degrees[]){
  int max_steps = 0;
  int steps[4] = {};
  for(int i=0; i < MOTOR_COUNT; i++){
    steps[i] = abs(stepper_degrees[i]/(degrees_per_step/(gear_ratio*microstep)));
    if(steps[i] > max_steps){
      max_steps = steps[i];
    }
  }
  Serial.print("steps in array: ");
  for(int i=0;i < MOTOR_COUNT; i++){
    Serial.print(steps[i]);
    Serial.print(", ");
  }
  Serial.println();
  Serial.print("max steps: ");
  Serial.println(max_steps);
  
  for(int i=0; i < MOTOR_COUNT; i++){
    if(stepper_degrees[i] >= 0){
      Serial.print("M");
      Serial.print(i);
      Serial.println(" set CCW");
      sd_ptr[i]->setDirection(0);
    }
    else{
      Serial.print("M");
      Serial.print(i);
      Serial.println(" set CW");
      sd_ptr[i]->setDirection(1);
    }
  }
  int m1_finished = 0;
  int m2_finished = 0;
  for(int i = 0; i <= max_steps; i++)
  {
    if(i < steps[0]){
      sd0.step();
    }
    else if(!m1_finished){
      m1_finished = true;
      Serial.print("M0 reached target rotation at ");
      Serial.print(i);
      Serial.println(" steps");
    }
    if(i < steps[1]){
      sd1.step();
    }
    else if(!m2_finished){
      m2_finished = true;
      Serial.println("M1 reached target rotation");
      Serial.print(i);
      Serial.println(" steps");
    }

    if(digitalRead(HomingPin)){
      Serial.println("HOMING SWITCH TRIGGERED");
    }
    delayMicroseconds(StepPeriodUs);
  }
}
