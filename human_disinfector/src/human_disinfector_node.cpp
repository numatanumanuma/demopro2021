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
    ros::spinOnce();

    double human_dir = 0;
    double human_dist = 0;
    char key = '@';
    int state = 0;
    double pre_dir = -1;
    double pre_dist = -1;
    tracer.set_threshold(0.6, -1);

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

        switch (state)
        {
        case 0:
            // 汚物探索
            double dir, dist, left , right;
            detector.getHumanDirAndDist(dir, dist, left, right);
            if (abs(dir) < 180){
                tracer.set_goal(dir,dist);
                startTimer(2);
                state = 1;
            }
            break;
        case 1:
            // 汚物へGO!!
            if(checkTimer()){
                player.setSound(mitiwoakero_sound);
                player.play();
            }
            if(tracer.run()) {
                // state = 2;
                state = -1;
            }
            break;
        case 2:
            // 汚物を消毒
            player.setSound(obutu_sound);
            player.play();
            servo.on();
            startTimer(5);
            state = 3;
            break;
        case 3:
            // 汚物消毒中...
            if(checkTimer())
                state = 4;
            break;
        case 4:
            // 汚物消毒完了
            servo.off();
            state = 5;
            break;
        case 5:
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