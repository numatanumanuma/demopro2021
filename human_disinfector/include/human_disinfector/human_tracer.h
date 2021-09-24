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
 
#define POS_TOLERANCE 0.001        // 状態遷移時に許容する位置誤差 [m]
#define RAD_TOLERANCE M_PI / 180.0 // 状態遷移時に許容する角度誤差 [rad]
#define LINEAR_VEL 0.4            // ロボットの並進速度 [m/s]
#define ANGULAR_VEL 0.9

class Tracer{
    private:
        geometry_msgs::Twist twist_msg_;
        geometry_msgs::Pose ref_pose_;
        ros::Publisher vel_pub_ ;
        ros::Subscriber odom_sub_;

        geometry_msgs::Pose pose_; // 現在の位置・姿勢
        sensor_msgs::LaserScan scan_; // 現在のセンサ情報

        // 目標点までの距離角度
        double goal_angle_;
        double goal_trans_;
        // 到着判定の距離角度
        double goal_threshold_angle_;
        double goal_threshold_trans_;
    
    public:
        Tracer(){
            ros::NodeHandle nh;
            odom_sub_ = nh.subscribe("/odom", 10, &Tracer::odomCallback, this);
            vel_pub_    = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
            goal_threshold_angle_ = 0.02;
            goal_threshold_trans_ = 1.0;
        }
        double normalize_angle(double); 
        // int Tracer_main(geometry_msgs::Pose, geometry_msgs::Pose);
        double calcYaw(geometry_msgs::Pose);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void set_goal(double dir, double dist);
        void set_threshold(double trans, double angle);
        void stop();
        bool run();
};

#endif