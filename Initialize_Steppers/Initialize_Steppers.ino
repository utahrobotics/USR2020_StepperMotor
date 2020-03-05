#include <SPI.h>
#include "HighPowerStepperDriver.h"
#include "TeensyThreads.h"

#define MOTOR_COUNT 1

const uint8_t CSPinM1 = 10;
const uint8_t CSPinM2 = 9;
const uint8_t HomingPinM0 = 7;
const uint8_t HomingPins[MOTOR_COUNT] = {HomingPinM0};

int16_t thetas[MOTOR_COUNT] = {-1};
const float degrees_per_step = 1.8; //deg/step of stepper shaft, not gearbox shaft
const uint8_t microstep = 4; //2^1, n = 0 - 8 //was 4
const float gear_ratio = 46.656; //old stepper motor gear ratio was 4.25
const uint16_t StepPeriodUs = 200; 

HighPowerStepperDriver sd0;
HighPowerStepperDriver sd1;
HighPowerStepperDriver *sd_ptr[MOTOR_COUNT] = {&sd0};


void calibrate_steppers(HighPowerStepperDriver *sd_ptr[]);
bool align_stepper(uint16_t step_count, HighPowerStepperDriver *sd);
void validate_calibration(int16_t thetas[]);

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
  validate_calibration(thetas);
  }

void loop() {
  // put your main code here, to run repeatedly:

}

void calibrate_steppers(HighPowerStepperDriver *sd_ptr[]){
  int16_t full_rotation = 500;
  uint16_t min_activeHoming_step[MOTOR_COUNT] = {0}; //probably initialize these to 0
  uint16_t max_activeHoming_step[MOTOR_COUNT] = {0}; //initialize these to 360??
  bool stepper_isAligned[MOTOR_COUNT] = {false};
  bool homing_isActive[MOTOR_COUNT] = {false};

  //!!!steps doesn't calculate the correct steps 360 degrees. It currenlty does about a 300 degree rotation
  int16_t steps = (full_rotation*gear_ratio*microstep)/degrees_per_step; //this value won't be exact. It takes 37324.8 steps/360 degrees. Steps can only be ints, so 37324 or 37325
  Serial.print("steps required for ");
  Serial.print(full_rotation);
  Serial.print(" degree rotation: ");
  Serial.println(steps); //!!!steps doesn't even change if full_rotation changes from 360 to 500
  for(int i = 0; i < MOTOR_COUNT; i++){
    sd_ptr[i]->setDirection(0);
  }
  for(int i = 0; i < steps; i++){
    for(int j =0; j < MOTOR_COUNT; j++){
      if(!stepper_isAligned[j]){
        sd_ptr[j]->step();
      }
      if(!homing_isActive[j] && digitalRead(HomingPins[j]) /*&& !stepper_isAligned[j]*/){ //switching homingpins[j] and homing_isActive[j] might save some computing time
        min_activeHoming_step[j] = i;              //there should be some kind of debouncer here
        homing_isActive[j] = true;
        Serial.print("Homing switch active. Step count = ");
        Serial.println(min_activeHoming_step[j]);
      }
      else if(!stepper_isAligned[j] && !digitalRead(HomingPins[j]) && homing_isActive[j]){
        max_activeHoming_step[j] = i;
        //homing_isActive[j] = false; //this might not need to be set false again. 

        Serial.print("Homing switch no longer active. Step count = ");
        Serial.println(max_activeHoming_step[j]);
        
        uint16_t delta_activeHoming_steps = max_activeHoming_step[j] - min_activeHoming_step[j];
        stepper_isAligned[j] = align_stepper(delta_activeHoming_steps/2, sd_ptr[j]);
        thetas[j] = 0;
        //can add an optional loop break if all the motors have been aligned
      }
      delayMicroseconds(StepPeriodUs);
    }
  }
}

bool align_stepper(uint16_t step_count, HighPowerStepperDriver *sd){
  delayMicroseconds(StepPeriodUs*2); //delay a little longer than normal
  bool isAligned = true;
  sd->setDirection(1);
  for(int i=0; i < step_count; i++){
    sd->step();
    delayMicroseconds(StepPeriodUs);
  }
  return isAligned;
}

void validate_calibration(int16_t thetas[]){
  bool steppers_areAligned = true;
  for(int i=0; i < MOTOR_COUNT; i++){
    if(thetas[i] != 0){
      steppers_areAligned = false;
    }
  }
  if(steppers_areAligned){
    Serial.println("All stepper motors are aligned");
  }
  else{
    Serial.println("Error during calibration. Not all steppers are aligned");
  }
}
