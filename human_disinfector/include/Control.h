#ifndef _CONTROL_H_
#define _CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


class Control_node{
    private:
        ros::Publisher pub_twist_;

    public:
        Control_node(){
            ros::NodeHandle nh;
            pub_twist_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
        }
    
        void move_ahead(double, double);
        void move_turn(double, double);
};

#endif