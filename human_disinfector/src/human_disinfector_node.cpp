#include "human_disinfector/human_detector.h"
#include "sound_player/sound_player.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "human_disinfector");
    
    ros::NodeHandle nh("~");
    ros::Rate looprate(1);

    std::string hoge;
    nh.param("hoge", hoge, std::string("hoge"));

    Detector detector;
    SoundPlayer player;

    while(ros::ok()) {


        ros::spinOnce();
        looprate.sleep();
    }

    return 0;
}