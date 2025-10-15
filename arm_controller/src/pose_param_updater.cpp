#include "pose_param_updater.h"
#include <ros/ros.h>

namespace arm_controller {

PoseParamUpdater::PoseParamUpdater(ros::NodeHandle& nh) : nh_(nh) {
    // 从参数服务器获取配置,如果没有则使用默认值
    nh_.param<std::string>("pose_topic", pose_topic_, "/target_pose");
    nh_.param<std::string>("param_x", param_x_, "/test/line_point_x");
    nh_.param<std::string>("param_y", param_y_, "/test/line_point_y");
    nh_.param<std::string>("param_z", param_z_, "/test/line_point_z");
    
    // 订阅PoseStamped话题
    pose_sub_ = nh_.subscribe(pose_topic_, 10, &PoseParamUpdater::poseCallback, this);
    
    ROS_INFO("PoseParamUpdater initialized");
    ROS_INFO("Subscribing to topic: %s", pose_topic_.c_str());
    ROS_INFO("Will update parameters: %s, %s, %s", 
             param_x_.c_str(), param_y_.c_str(), param_z_.c_str());
}

PoseParamUpdater::~PoseParamUpdater() {
    ROS_INFO("PoseParamUpdater shutting down");
}

void PoseParamUpdater::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // 提取位置信息
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;
    
    // 更新ROS参数
    nh_.setParam(param_x_, x);
    nh_.setParam(param_y_, y);
    nh_.setParam(param_z_, z);
    
    ROS_INFO("Updated parameters: x=%.3f, y=%.3f, z=%.3f", x, y, z);
    ROS_DEBUG("Frame ID: %s, Timestamp: %.3f", 
              msg->header.frame_id.c_str(), 
              msg->header.stamp.toSec());
}

} // namespace arm_controller
