#ifndef _FJT_H_
#define _FJT_H_
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <fstream>
#include <vector>

 
#define POS_TOLERANCE 0.001        // 状態遷移時に許容する位置誤差 [m]
#define RAD_TOLERANCE M_PI / 180.0 // 状態遷移時に許容する角度誤差 [rad]
#define LINEAR_VEL 0.15            // ロボットの並進速度 [m/s]
#define ANGULAR_VEL 0.5 

class trace{
    private:
        geometry_msgs::Twist twist_msg;
        geometry_msgs::Pose ref_pose;
        geometry_msgs::Pose now_pose;
        ros::Publisher vel_pub_ ;

        geometry_msgs::Pose pose_; // 現在の位置・姿勢
        sensor_msgs::LaserScan scan_; // 現在のセンサ情報
    
    public:
        trace(){
            ros::NodeHandle nh;
            ros::Subscriber odom_sub = nh.subscribe("/odom", 10, &trace::odomCallback, this);
            vel_pub_    = nh.advertise<geometry_msgs::Twist>("/beego/diff_drive_controller/cmd_vel", 10);
        }
        double normalize_angle(double); 
        int trace_main(geometry_msgs::Pose, geometry_msgs::Pose);
        double calcYaw(geometry_msgs::Pose);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void getCurrentPose(geometry_msgs::Pose& pose);
};

#endif