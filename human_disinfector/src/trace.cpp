#include "trace.h"


void trace::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    pose_ = msg->pose.pose;
}
 
double trace::calcYaw(geometry_msgs::Pose pose)
{ // geometry_msgs::PoseからYaw角を計算する関数
    tf::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

double trace::normalize_angle(double angle)
{ // 角度を-π〜+πに正規化する関数
    double result = angle;
    while (result > M_PI)
    result -= 2.0 * M_PI;
    while (result < -M_PI)
    result += 2.0 * M_PI;
    return result;
}

void trace::getCurrentPose(geometry_msgs::Pose& pose)
{ // 現在の姿勢を取得する関数
    pose = pose_;
}

int trace::trace_main(geometry_msgs::Pose pose, geometry_msgs::Pose ref_pose)
{
    //ros::init(argc, argv, "sample1"); //ノード名の初期化
    ros::Duration(1.0).sleep(); // 1.0秒待機
    ros::spinOnce(); // はじめにコールバック関数を呼んでおく
    bool end_flag = false;
    int state = 0;

    ros::Rate loop_rate(10);

    ref_pose = pose_; // はじめの姿勢を基準として保存

    while (ros::ok() && !end_flag)
    { // Ctrl-Cが押されるとros::ok()はfalseとなる（=ループを抜ける） 
        pose = pose_;// 現在の姿勢を取得
        // ref_poseとの差を取得
        double dist_diff = hypot(pose.position.x - ref_pose.position.x,

        pose.position.y - ref_pose.position.y); // poseとref_poseの距離 

        double yaw_diff = normalize_angle(calcYaw(pose) - calcYaw(ref_pose)); // poseとref_poseのなす角度

        std::cout << "state:" << state << ", dist:" << dist_diff << ", yaw:" << yaw_diff << std::endl;

        // 状態遷移

        geometry_msgs::Twist msg;
        msg.linear.x = LINEAR_VEL;
        msg.angular.z = -ANGULAR_VEL;;
        vel_pub_.publish(msg);

        if (dist_diff > 1.0 - POS_TOLERANCE)
        { // 一定距離以上(1.0m)進んだら，状態を進める

            pose = pose_; // 基準の更新          
            state = 1;
            
        }
    }

}