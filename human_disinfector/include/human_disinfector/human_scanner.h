#ifndef human_scanner_H
#define human_scanner_H

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

#define Disp(x) std::cout << #x << ':' << x << std::endl
   
class Scanner{
public:
    Scanner();
    ~Scanner();
    void msgsCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent&);
    double getDist(double degree);
    void findObstacle(double dir, double dist, int dmin, int dmax);
   
private:

    ros::Subscriber scan_sub_;
    ros::Timer timer_;
    sensor_msgs::LaserScan scan_;

};

#endif