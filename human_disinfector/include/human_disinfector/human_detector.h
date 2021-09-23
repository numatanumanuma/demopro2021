#ifndef human_detector_H
#define human_detector_H

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#define Disp(x) std::cout << #x << ':' << x << std::endl
   
class Detector{
public:
    Detector();
    ~Detector();
    void msgsCallback(const geometry_msgs::Twist::ConstPtr& msgs);
    void timerCallback(const ros::TimerEvent&);
    void getHumanDirAndDist(double& dir, double& dist);
   
private:
    // 取得めんどくさいので直に書き込み
    const int camera_height = 480;
    const int camera_width = 848;
    const double camera_angle = 69.4;

    const int dist_search_range = 10; // デプス情報を探す範囲
    void darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
    void publishCurrentTwist();

    ros::Publisher twist_pub_;
    ros::Subscriber darknet_sub_;
    ros::Subscriber depth_sub_;
    ros::Timer timer_;
    geometry_msgs::Twist twist_;
    darknet_ros_msgs::BoundingBoxes bb_results_;
    sensor_msgs::Image depth_results_;


};

#endif