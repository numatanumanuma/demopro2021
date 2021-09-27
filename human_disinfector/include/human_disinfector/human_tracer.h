#ifndef _HUMAN_TRACER_H_
#define _HUMAN_TRACER_H_
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <fstream>
#include <vector>

#define LINEAR_VEL 0.2           // ロボットの並進速度 [m/s]
#define ANGULAR_VEL 0.3

class Tracer{
    private:
        geometry_msgs::Twist twist_msg_;
        geometry_msgs::Pose ref_pose_;
        ros::Publisher vel_pub_ ;
        ros::Subscriber odom_sub_;

        geometry_msgs::Pose pose_; // 現在の位置・姿勢
        sensor_msgs::LaserScan scan_; // 現在のセンサ情報

        double ref_x_;
        double ref_y_;
        double ref_yaw_;
        // 目標点までの距離角度
        double goal_angle_;
        double goal_trans_;
        // 到着判定の距離角度
        double goal_threshold_angle_;
        double goal_threshold_trans_;
    
    public:
        Tracer();
        double normalize_angle(double); 
        // int Tracer_main(geometry_msgs::Pose, geometry_msgs::Pose);
        double calcYaw(geometry_msgs::Pose);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void getCurrentPose(geometry_msgs::Pose& pose);
        void set_goal(double dir, double dist);
        void set_threshold(double trans, double angle);
        void straight();
        void spinLeft();
        void spinRight();
        void stop();
        
        // bool run();
};

#endif