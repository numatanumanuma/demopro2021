#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

#include "sound_player/sound_player.h"
#include "human_disinfector/human_detector.h"
#include "human_disinfector/human_tracer.h"
#include "human_disinfector/human_scanner.h"
#include "human_disinfector/FJT.h"
#include "kbhit.h"

#define POS_TOLERANCE 0.001        // 状態遷移時に許容する位置誤差 [m]
#define RAD_TOLERANCE M_PI * 10 / 180.0 // 状態遷移時に許容する角度誤差 [rad]
#define HUMAN_TOLERANCE 0.4

ros::Time start_t;
ros::Duration limit_t;
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
    nh.param("debug", debug, false);

    Detector detector;
    servo_pubNode servo;
    SoundPlayer player;
    Tracer tracer;
    Scanner scanner;
    ros::Duration(1).sleep();
    ros::spinOnce();

    double human_dir = 0;
    double human_dist = 0;
    char key = '@';
    int state = 0;
    double pre_dir = -1;
    double pre_dist = -1;

    geometry_msgs::Pose pose; // 現在の姿勢
    geometry_msgs::Pose ref_pose; // 基準となる姿勢
    tracer.getCurrentPose(ref_pose); // はじめの姿勢を基準として保存

    while(ros::ok()) {

        std::cout << "state..." << state << std::endl;

        if (debug) {
            state = -1;
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
            // continue;
        }

        // <--- ここからメイン処理 --->

        tracer.getCurrentPose(pose); // 現在の姿勢を取得
        double dist_diff = hypot(pose.position.x - ref_pose.position.x,
                                 pose.position.y - ref_pose.position.y);
        double yaw_diff = tracer.normalize_angle(tracer.calcYaw(pose) - tracer.calcYaw(ref_pose));
        std::cout << "state:" << state << ", dist:" << dist_diff << ",  yaw:" << yaw_diff << std::endl;

        switch (state)
        {
        case 0:
            // 汚物探索
            double dir, dist, left , right;
            detector.getHumanDirAndDist(dir, dist, left, right);
            human_dir = dir ;
            human_dist = scanner.getDist(dir); 
            std::cout << human_dir << ", " << human_dist << std::endl;
            if (abs(human_dir) < 180){
                tracer.set_goal(human_dir, human_dist);
                startTimer(2);
                state = 1;
            }
            break;
        case 1:
            // 汚物へGO!!(旋回)
            if(checkTimer()){
                player.setSound(mitiwoakero_sound);
                player.play();
                startTimer(2);
            }
            if(human_dir > 0){
                // 左旋回
                tracer.spinLeft();
                if (yaw_diff > human_dir * M_PI / 180 - RAD_TOLERANCE)
                {
                    tracer.getCurrentPose(ref_pose);
                    tracer.stop();
                    state = 2;
                }
            } else {
                // 右旋回
                tracer.spinRight();
                if (yaw_diff < human_dir * M_PI / 180 + RAD_TOLERANCE)
                {
                    tracer.getCurrentPose(ref_pose);
                    tracer.stop();
                    state = 2;
                }
            }
            break;
        case 2:
            // 汚物へGO!!(直進)
            if(checkTimer()){
                player.setSound(mitiwoakero_sound);
                player.play();
                startTimer(2);
            }
            tracer.straight(); // 直進
            if (dist_diff > human_dist - HUMAN_TOLERANCE - POS_TOLERANCE)
            { 
                tracer.getCurrentPose(ref_pose);
                tracer.stop();
                state = 3;
            }
            break;
        case 3:
            // 汚物を消毒
            player.setSound(obutu_sound);
            player.play();
            servo.on();
            startTimer(5);
            state = 4;
            break;
        case 4:
            // 汚物消毒中...
            if(checkTimer())
                state = 5;
            break;
        case 5:
            // 汚物消毒完了
            servo.off();
            state = 6;
            startTimer(3);
            break;
        case 6:
            if(checkTimer()){
                state = 0;
            }
            break;
        case -1:
            // debugモードのためスルー
            break;
        default:
            break;
        }

        ros::spinOnce();
        looprate.sleep();
    }

    return 0;
}