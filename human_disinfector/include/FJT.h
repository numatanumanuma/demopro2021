#ifndef _FJT_H_
#define _FJT_H_
#include <ros/ros.h>
#include <std_msgs/UInt16.h>



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

class servo_pubNode
{
    private:
        ros::Publisher pub_pattern;
        std_msgs::UInt16 pattern;
    
    public:
        servo_pubNode()
        {
            ros::NodeHandle nh;
            pub_pattern = nh.advertise<std_msgs::UInt16>("pattern",1000);
        }
        
        void on();
        void off();
        void on_off();

};

#endif // _FJT_H_