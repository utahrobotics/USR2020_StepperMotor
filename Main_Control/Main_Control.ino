#include <SPI.h>
#include <HighPowerStepperDriver.h>
#include <TeensyThreads.h>
#include <numeric>

#define CMD_BYTES 8
#define LED_PIN 13

volatile int cmd = 0;

const uint8_t sleepPin = 14;

uint8_t homingPins[4] = {3, 32, 33, 20};
const uint8_t CSPins[4] = {0, 29, 36, 23};
const uint8_t FaultPins[4] = {1, 30, 35, 22}; //currently unused
const uint8_t StallPin[4] = {2, 31, 34,21}; //currently unused

uint8_t currentPositions[4] = {180, 180, 180, 180};

const double degrees_per_step = 1.8;
const double microstep = 2; //2^1, n = 0 - 8 //was 4
const double degreesM1 = 360;
const double gear_ratio = 46.656; //old stepper motor gear ratio was 4.25

// This period is the length of the delay between steps, which controls the
// stepper motor's speed.  You can increase the delay to make the stepper motor
// go slower.  If you decrease the delay, the stepper motor will go faster, but
// there is a limit to how fast it can go before it starts missing steps.
const double StepPeriodUs = 500; //best value so far 800
const uint16_t StepPeriodMs = 1; //was 2

bool isHomed=false;

HighPowerStepperDriver sd;

void recieve_command(){
  while(1){ 
    if(Serial.available() > 0) {
      char read_byte = Serial.read();
      
      switch(read_byte){
        case 0x1:
          send_msg("init cmd recieved", false);
          init_all();
          send_msg("init_all processed");
          break;

        case 0x2:
          send_msg("align_cmd_recieved", false);
          uint8_t degs[4];
          for(int i = 0; i < 4; i++){
            while(Serial.available() < 1){}
            char read_byte = Serial.read();
            send_msg(read_byte, false);
            degs[i] = read_byte;
          }

          align_all(degs);
          
          break;
          
        case 0x6:
          send_msg("blink cmd recieved\n", false);
          while(Serial.available() < 1){}
          int num_blinks = Serial.read();
          send_msg("blinking " + (String)num_blinks + " times\n", false);
          for(int i = 0; i < num_blinks; i++){
              digitalWrite(LED_PIN, HIGH);
              delay(600);
              digitalWrite(LED_PIN, LOW);
              delay(600);
          }
          send_msg("blink processed");
          break;
          
         default:
          Serial.println("recieved some wacky bytes");
          break;
      }
    }
  }
}

void init_all(){
  bool homed[4] = {false, false, false, false};

  while(std::accumulate(homed, homed + 4, 0) < 4){
    for(int i = 0; i < 4; i++){
      if(homed[i]==false){
        if(digitalRead(homingPins[i]) == 0){
          turn_degrees(2, CSPins[i]);
        }
        else{
          homed[i] = true;
        }
      }
    }
  }

  isHomed = true;
}

void align_all(uint8_t degs[]){
  while(!twoArrEqual(degs, currentPositions)){
    for(int i = 0; i < 4; i++){
      if(degs[i]<currentPositions[i]){
        turn_degrees(-1, CSPins[i]);
        currentPositions[i]--;
      }
      else if(degs[i]>currentPositions[i]){
        turn_degrees(1, CSPins[i]);
        currentPositions[i]++;
      }
    }
  }
}

void send_msg(String msg){
  send_msg(msg, true);
}

void send_msg(String msg, bool terminate){
  if(!terminate){
    Serial.print(msg);
  }
  else{
    Serial.print(msg + '\0');
  }
}

void turn_degrees(int degrees, int CSPin){
  sd.setChipSelectPin(CSPin);
  double steps = abs(degrees/(degrees_per_step/(gear_ratio*microstep)));
  if(degrees>=0){
    sd.setDirection(0);
  }
  else{
    sd.setDirection(1);
  }
  for(unsigned int x = 0; x <= steps; x++)
  {
    sd.step();
    delayMicroseconds(StepPeriodUs);
  }
}

void init_drivers(int CSPin, int homingPin){
    pinMode(homingPin, INPUT);
  
    sd.setChipSelectPin(CSPin); 
    delay(1);
    sd.resetSettings();
    sd.clearStatus();
    sd.setDecayMode(HPSDDecayMode::AutoMixed);
    sd.setCurrentMilliamps36v4(2800); //4000 max
    double steps_approx = abs(degreesM1/(degrees_per_step/(gear_ratio*microstep)));
    double time_approx = (steps_approx*StepPeriodUs)/1000000;
    float rpm_approx = (60/time_approx)*degreesM1/float(360);  
    sd.setStepMode(microstep);
    
    sd.enableDriver();
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  
  delay(1000);
  
  SPI.begin();
  
  //digitalWrite(SleepPin, HIGH);

  pinMode(sleepPin, OUTPUT);
  digitalWrite(sleepPin, HIGH);

  for(int i = 0; i < 4; i++){
    init_drivers(CSPins[i], homingPins[i]);
  }
  
//  pinMode(FaultPin, INPUT);
//  pinMode(StallPin, INPUT);
}

bool twoArrEqual(uint8_t arr1[], uint8_t arr2[]) 
{ 
  for (int i = 0; i < 4; i++){
    if (arr1[i] != arr2[i]) {
      return false;
    }
  }
  
  return true;
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("waiting on command");
  recieve_command();
}
