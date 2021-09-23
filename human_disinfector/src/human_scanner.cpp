#include "human_disinfector/human_scanner.h"
   
Scanner::Scanner(){
    ros::NodeHandle nh("~");
    scan_sub_ = nh.subscribe("/scan", 10, &Scanner::msgsCallback, this);
    timer_ = nh.createTimer(ros::Duration(0.05), &Scanner::timerCallback, this);
    
}

Scanner::~Scanner(){}

void Scanner::timerCallback(const ros::TimerEvent&){

}

void Scanner::msgsCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    scan_ = *msg;
}

/*
 引数の角度(度)方向の距離を返す
 無限遠の場合は-1
*/
double Scanner::getDist(double degree) {
    int i = (- scan_.angle_min + degree * M_PI / 180) / scan_.angle_increment;
    if (i >= 0 && i < scan_.ranges.size()){
        if (scan_.ranges[i] >= scan_.range_min &&
            scan_.ranges[i] <= scan_.range_max &&
            ! std::isnan(scan_.ranges[i])){
            return scan_.ranges[i];
        }
    }
    return -1;
}

/*
 dmin(度)~dmax(度)範囲の一番近い物体の方向(dir(度)), 距離(dist(m))を渡す
*/
void Scanner::findObstacle(double dir, double dist, int dmin, int dmax) {
    int index_min = (- scan_.angle_min + dmin * M_PI / 180) / scan_.angle_increment;
    int index_max = (- scan_.angle_min + dmax * M_PI / 180) / scan_.angle_increment;
    if (index_min > index_max){
        int tmp = index_min;
        index_min = index_max;
        index_max = tmp;
    }
    index_min = std::max(index_min, 0);
    index_max = std::min(index_max, (int)scan_.ranges.size());
    int index;
    int degree;
    float distance = 1000;
    for (int i = index_min; i <= index_max; i++){
        if (scan_.ranges[i] < distance){
            distance = scan_.ranges[i];
            index = i;
        }
    }
    if (distance == 1000){
        distance = -1;
        degree = -1;
    }
    degree = (index * scan_.angle_increment + scan_.angle_min) * 180 / M_PI; 
    dir = degree;
    dist = distance;
    return;
}
