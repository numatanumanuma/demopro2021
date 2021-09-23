#include "Control.h"



void move_ahead(double dist, double d_thresh){
    ros::Rate rate(10.0);
    ros::Time start = ros::Time::now();
  while (ros::ok())
  {
    ros::spinOnce();
    ros::Time now = ros::Time::now();
    geometry_msgs::Twist cmd_vel;

    if (dist > d_thresh){
      cmd_vel.linear.x = 0.3;
      cmd_vel.angular.z = 0.0;
        ROS_INFO("go");
    }
    else{
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
       ROS_INFO("stop");
       break;
    }

    rate.sleep();
  }
} 

void move_turn(double yaw, double y_thresh){
    ros::Rate rate(10.0);
    geometry_msgs::Twist cmd_vel;
    ros::Time start = ros::Time::now();
  while (ros::ok())
  {
    ros::spinOnce();
    ros::Time now = ros::Time::now();

    if (fabs(yaw) < y_thresh){
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.2;
        ROS_INFO("turn");
    }
    else{
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
       ROS_INFO("stop");
       break;
    }

    rate.sleep();
  }
} 