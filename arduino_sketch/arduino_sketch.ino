#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>


ros::NodeHandle nh;
Servo servo1;

void messageCb( const std_msgs::UInt16& pattern) {
 if(pattern.data == 1){
  int deg=30; // put your main code here, to run repeatedly:
  for (deg=30; deg<150; deg++){
    servo1.write(deg);
    delay(20);
  }
  for (deg=150; deg>30; deg--){
    servo1.write(deg);
    delay(20);
  }
 }
 if(pattern.data == 2){
  int deg=30;
  for (deg=30; deg<150; deg++){
    servo1.write(deg);
    delay(20);
  }
 }
 if(pattern.data == 3){
  int deg=150;
  for (deg=150; deg>30; deg--){
    servo1.write(deg);
    delay(20);
  }
 }
}

ros::Subscriber<std_msgs::UInt16> sub("pattern", messageCb);

void setup() {
  // put your setup code here, to run once:
  servo1.attach(9);
  nh.initNode();
  nh.subscribe(sub);
}

 

void loop() {
  nh.spinOnce();
  delay(1);
}
