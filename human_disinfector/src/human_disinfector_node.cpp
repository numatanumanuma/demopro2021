#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

#include "human_disinfector/human_detector.h"
#include "sound_player/sound_player.h"
#include "../include/FJT.h"
#include "kbhit.h"

#define POS_TOLERANCE 0.001        // 状態遷移時に許容する位置誤差 [m]
#define RAD_TOLERANCE M_PI / 180.0 // 状態遷移時に許容する角度誤差 [rad]
#define LINEAR_VEL 0.15            // ロボットの並進速度 [m/s]
#define ANGULAR_VEL 0.5            // ロボットの角速度 [rad/s]

geometry_msgs::Pose pose;       // 現在の位置・姿勢
sensor_msgs::LaserScan scan;    // 現在のセンサ情報
ros::Time start_t;
ros::Duration limit_t;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    scan = *msg;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    pose = msg->pose.pose;
}

double calcYaw(geometry_msgs::Pose pose)
{ // geometry_msgs::PoseからYaw角を計算する関数
    tf::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

double normalize_angle(double angle)
{ // 角度を-π〜+πに正規化する関数
    double result = angle;
    while (result > M_PI)
        result -= 2.0 * M_PI;
    while (result < -M_PI)
        result += 2.0 * M_PI;
    return result;
}

void startTimer(double t) {
    start_t = ros::Time::now();
    limit_t = ros::Duration(t);
}

bool checkTimer() {
    ros::Duration t = ros::Time::now() - start_t;
    if (ros::Time::now() - start_t > limit_t) {
        return true;
    }else {
        return false;
    }
}

double getDist(double degree) {
    int i = (- scan.angle_min + degree * M_PI / 180) / scan.angle_increment;
    if (i >= 0 && i < scan.ranges.size()){
        if (scan.ranges[i] >= scan.range_min &&
            scan.ranges[i] <= scan.range_max &&
            ! std::isnan(scan.ranges[i])){
            return scan.ranges[i];
        }
    }
    return -1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "human_disinfector");
    
    ros::NodeHandle nh("~");
    ros::Rate looprate(2);

    std::string obutu_sound;
    std::string mitiwoakero_sound;
    bool debug;
    nh.param("obutu_sound", obutu_sound, std::string("obutuhasyoudokuda.wav"));
    nh.param("mitiwoakero_sound", mitiwoakero_sound, std::string("mitiwoakero.wav"));
    nh.param("debug", debug, true);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);;
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);

    Detector detector;
    servo_pubNode servo;
    SoundPlayer player;

    geometry_msgs::Twist twist_msg;
    geometry_msgs::Pose ref_pose; // 基準となる姿勢
    geometry_msgs::Pose now_pose; // 現在の姿勢
    sensor_msgs::LaserScan scan;
    double human_dir = 0;
    double human_dist = 0;

    char key = '@';
    int state = 0;
    ros::spinOnce();

    while(ros::ok()) {

        double dir, dist;
        detector.getHumanDirAndDist(dir, dist);
        human_dir   = dir;
        // human_dist  = dist;
        human_dist  = getDist(dir);
        std::cout << human_dir << ", "<< human_dist << std::endl;

        if (debug) {
            if (kbhit()) {
                std::cin >> key;
                std::cout << "input key... " << key << std::endl;
            }
            switch (key)
            {
            case '1':
                player.setSound(obutu_sound);
                player.play();
                servo.on();
                std::cout << "on" << std::endl;
                break;
            case '2':
                servo.off();
                std::cout << "off" << std::endl;
                break;
            case '3':
                servo.on_off();
                std::cout << "on off" << std::endl;
                break;
            case '4':
                player.setSound(obutu_sound);
                player.play();
                break;
            case '5':
                player.setSound(mitiwoakero_sound);
                player.play();
                break;
            default:
                break;
            }
            key = '@';
            ros::spinOnce();
            looprate.sleep();
            continue;
        }

        now_pose = pose;
        double dist_diff = hypot(pose.position.x - ref_pose.position.x,
                                 pose.position.y - ref_pose.position.y); // poseとref_poseの距離
        double yaw_diff = normalize_angle(calcYaw(pose) - calcYaw(ref_pose));      // poseとref_poseのなす角度
        std::cout << "state:" << state << ", dist:" << dist_diff << ",  yaw:" << yaw_diff << std::endl;

        switch (state)
        {
        case 0:
            std::cout << "iie"<< std::endl;
            break;
        
        default:
            break;
        }

        ros::spinOnce();
        looprate.sleep();
    }

    return 0;
}