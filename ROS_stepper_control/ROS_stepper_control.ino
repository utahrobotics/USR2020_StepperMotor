#include "ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "HighPowerStepperDriver.h"

ros::NodeHandle nh;

void messageCb(const std_msgs::Int16MultiArray& degree_array){
  std_msgs::Int16MultiArray msg = degree_array;
  String title = msg.layout.dim->label;
  Serial.println(msg.data[0]); //Int16MulitArrays have many data memebers, check ros to find what they are
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("turn_steppers", &messageCb );

void setup() {
  nh.initNode();
  nh.subscribe(sub);

}

void loop() {
  nh.spinOnce();
  delay(1000);
  

}
