#ifndef human_detector_H
#define human_detector_H

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
   
class Detector{
public:
    Detector();
    ~Detector();
    void msgsCallback(const geometry_msgs::Twist::ConstPtr& msgs);
    void timerCallback(const ros::TimerEvent&);
   
private:

    void darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
    void publishCurrentTwist();

    ros::Publisher twist_pub_;
    ros::Subscriber darknet_sub_;
    ros::Timer timer_;
    geometry_msgs::Twist twist_;
    darknet_ros_msgs::BoundingBoxes results_;

};

#endif