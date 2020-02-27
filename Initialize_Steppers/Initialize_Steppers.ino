#include <SPI.h>
#include "HighPowerStepperDriver.h"
#include "TeensyThreads.h"

#define MOTOR_COUNT 2

const uint8_t CSPinM1 = 10;
const uint8_t CSPinM2 = 9;
const uint8_t HomingPinM0 = 7;
const uint8_t HomingPins[MOTOR_COUNT] = {HomingPinM0};

int16_t thetas[MOTOR_COUNT] = {-1, -1};
const float degrees_per_step = 1.8; //deg/step of stepper shaft, not gearbox shaft
const uint8_t microstep = 4; //2^1, n = 0 - 8 //was 4
const float gear_ratio = 46.656; //old stepper motor gear ratio was 4.25
const uint16_t StepPeriodUs = 200; 

HighPowerStepperDriver sd0;
HighPowerStepperDriver sd1;
HighPowerStepperDriver *sd_ptr[MOTOR_COUNT] = {&sd0, &sd1};


void calibrate_steppers();
bool align_stepper(uint16_t step_count, HighPowerStepperDriver *sd);

void setup() {
  Serial.begin(9600);
  delay(500);
  SPI.begin();
  delay(500);
  pinMode(HomingPinM0,INPUT);

  Serial.println("Initializing drivers");
  sd0.setChipSelectPin(CSPinM1);
  sd1.setChipSelectPin(CSPinM2);
  //the following fuctions rely on SPI, so they must follow setChipSelectPin()
  //the following stepper config functions can be placed in a single for loop
  for(int i=0; i < MOTOR_COUNT; i++){
    sd_ptr[i]->resetSettings();
    sd_ptr[i]->clearStatus();
    sd_ptr[i]->setDecayMode(HPSDDecayMode::AutoMixed);
    sd_ptr[i]->setCurrentMilliamps36v4(2800);
    sd_ptr[i]->setStepMode(microstep);
    sd_ptr[i]->enableDriver();
  }
  Serial.println("Drivers enabled, beginning calibration routine\n");

  calibrate_steppers(sd_ptr);

}

void loop() {
  // put your main code here, to run repeatedly:

}

void calibrate_steppers(HighPowerStepperDriver *sd_ptr[]){
  int16_t full_rotation = 360;
  uint16_t min_activeHoming_step[MOTOR_COUNT] = {0,0}; //probably initialize these to 0
  uint16_t max_activeHoming_step[MOTOR_COUNT] = {0,0}; //initialize these to 360??
  bool stepper_isAligned[MOTOR_COUNT] = {false, false};
  bool homing_isActive[MOTOR_COUNT] = {false, false};
  
  int16_t steps = (full_rotation*gear_ratio*microstep)/degrees_per_step; //this value won't be exact. It takes 37324.8 steps/360 degrees. Steps can only be ints, so 37324 or 37325
  Serial.print("steps required for 360 degree rotation: ");
  Serial.println(steps);
  for(int i = 0; i < MOTOR_COUNT; i++){
    sd_ptr[i]->setDirection(0);
  }
  for(int i = 0; i < steps; i++){
    for(int j =0; j < MOTOR_COUNT; j++){
      if(!stepper_isAligned[j]){
        sd_ptr[j]->step();
      }
      if(digitalRead(!homing_isActive[j] && HomingPins[j]) /*&& !stepper_isAligned[j]*/){ //switching homingpins[j] and homing_isActive[j] might save some computing time
        min_activeHoming_step[j] = i;              //there should be some kind of debouncer here
        homing_isActive[j] = true;
      }
      else if(!digitalRead(!stepper_isAligned[j] && HomingPins[j]) && homing_isActive[j]){
        max_activeHoming_step[j] = i;
        //homing_isActive[j] = false; //this might not need to be set false again. 
        uint16_t delta_activeHoming_steps = max_activeHoming_step[j] - min_activeHoming_step[j];
        stepper_isAligned[j] = align_stepper(delta_activeHoming_steps, sd_ptr[j]);
        thetas[j] = 0;
      }
      delayMicroseconds(StepPeriodUs);
    }
  }
}

bool align_stepper(uint16_t step_count, HighPowerStepperDriver *sd){
  bool isAligned = true;
  sd->setDirection(1);
  for(int i=0; i < step_count; i++){
    sd->step();
    delayMicroseconds(StepPeriodUs);
  }
  return isAligned;
}
