#include "human_disinfector/human_detector.h"
   
Detector::Detector(){
    ros::NodeHandle nh("~");
    twist_pub_ = nh.advertise<geometry_msgs::Twist>("/sample/cmd_vel", 10);
    darknet_sub_ = nh.subscribe("/darknet_ros/bounding_boxes", 100,
        &Detector::darknetCallback, this);
    depth_sub_ = nh.subscribe("/camera/depth/image_rect_raw", 100,
        &Detector::depthCallback, this);
    timer_ = nh.createTimer(ros::Duration(0.05), &Detector::timerCallback, this);
    
}

Detector::~Detector(){}

void Detector::darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& results) {
    bb_results_ = *results;
}

void Detector::depthCallback(const sensor_msgs::Image::ConstPtr& results) {
    depth_results_ = *results;
}

void Detector::timerCallback(const ros::TimerEvent&){
    ROS_INFO("x:%0.3f, y:%0.3f", twist_.linear.x, twist_.linear.y);
}

void Detector::publishCurrentTwist(){
    twist_pub_.publish(twist_);
    ros::Duration(0.01).sleep();
}

// 人検出して，その人の方向(yaw)と距離を入れる
void Detector::getHumanDirAndDist(double& dir, double& dist) {
    for(auto bb : bb_results_.bounding_boxes) {
        if(bb.Class == "person" && bb.probability >= 0.7) {
            int xcenter = (bb.xmax - bb.xmin)/2;
            int ycenter = (bb.ymax - bb.ymin)/2;
            double yaw_ratio = (camera_width - (bb.xmax - bb.xmin))/camera_width;
            dir = yaw_ratio*camera_angle;

            double tmp_dist = 1e5;
            for(int i = -dist_search_range/2; i <= dist_search_range/2; i++) {
                for(int j = -dist_search_range/2; j <= dist_search_range/2; j++) {
                    int nowx = xcenter + j, nowy = ycenter + i;
                    if(0 <= nowx && nowx < camera_width
                    && 0 <= nowy && nowy < camera_height) {
                        int idx = nowy*camera_width + nowx;
                        double depth = depth_results_.data[idx];         
                        if(tmp_dist < depth) tmp_dist = depth;
                    }
                }
            }
            dist = tmp_dist;
        }
    }
}