#include "human_disinfector/human_detector.h"
   
Detector::Detector(){
    ros::NodeHandle nh("~");
    twist_pub_ = nh.advertise<geometry_msgs::Twist>("/sample/cmd_vel", 10);
    darknet_sub_ = nh.subscribe("/darknet_ros/bounding_boxes", 100,
        &Detector::darknetCallback, this);
    timer_ = nh.createTimer(ros::Duration(0.05), &Detector::timerCallback, this);
    
}

Detector::~Detector(){

}

void Detector::darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& results) {
    results_ = *results;
}

void Detector::timerCallback(const ros::TimerEvent&){
    ROS_INFO("x:%0.3f, y:%0.3f", twist_.linear.x, twist_.linear.y);
}

void Detector::publishCurrentTwist(){
    twist_pub_.publish(twist_);
    ros::Duration(0.01).sleep();
}