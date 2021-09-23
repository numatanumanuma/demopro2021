
#include "../include/FJT.h"


void servo_pubNode::on_off(){
    sleep(1);
    pattern.data = 1;
    pub_pattern.publish(pattern);
    sleep(1);
}
   

void servo_pubNode::on(){
    sleep(1);
     pattern.data = 2;
     pub_pattern.publish(pattern);
     sleep(1);

}

void servo_pubNode::off(){
    sleep(1);
     pattern.data = 3;
     pub_pattern.publish(pattern);
     sleep(1);

}