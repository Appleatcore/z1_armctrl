#include "pose_param_updater.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace arm_controller {

PoseParamUpdater::PoseParamUpdater(ros::NodeHandle& nh) : nh_(nh) {
    // 从参数服务器获取配置,如果没有则使用默认值
    nh_.param<std::string>("pose_topic", pose_topic_, "/target_pose");

    // TF参数
    nh_.param<std::string>("target_frame", target_frame_, "link00");
    nh_.param<bool>("enable_tf_transform", enable_tf_transform_, true);
    nh_.param<double>("tf_timeout", tf_timeout_, 3.0);
    
    // 订阅PoseStamped话题
    pose_sub_ = nh_.subscribe(pose_topic_, 10, &PoseParamUpdater::poseCallback, this);
    
    // 发布转换后的 PoseStamped
    transformed_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/transformed_pose", 10);
    
    ROS_INFO("PoseParamUpdater initialized");
    ROS_INFO("Subscribing to topic: %s", pose_topic_.c_str());
    ROS_INFO("Will update parameters: %s, %s, %s", 
             param_x_.c_str(), param_y_.c_str(), param_z_.c_str());
    
    if (enable_tf_transform_) {
        ROS_INFO("TF transform enabled: target_frame='%s', timeout=%.1fs", 
                 target_frame_.c_str(), tf_timeout_);
    } else {
        ROS_INFO("TF transform disabled");
    }
}

PoseParamUpdater::~PoseParamUpdater() {
    ROS_INFO("PoseParamUpdater shutting down");
}

void PoseParamUpdater::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    geometry_msgs::PoseStamped transformed_pose;
    
    // 如果启用TF变换且源坐标系不是目标坐标系
    if (enable_tf_transform_ && msg->header.frame_id != target_frame_) {
        try {
            // 等待TF变换可用
            if (!tf_listener_.waitForTransform(target_frame_, msg->header.frame_id, 
                                               msg->header.stamp, ros::Duration(tf_timeout_))) {
                ROS_WARN("TF transform timeout: %s -> %s", 
                         msg->header.frame_id.c_str(), target_frame_.c_str());
                return;
            }
            
            // 执行TF变换
            tf_listener_.transformPose(target_frame_, *msg, transformed_pose);
            
            ROS_INFO("Transformed pose from '%s' to '%s'", 
                     msg->header.frame_id.c_str(), target_frame_.c_str());
            ROS_DEBUG("  Original: (%.3f, %.3f, %.3f)", 
                      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            ROS_DEBUG("  Transformed: (%.3f, %.3f, %.3f)", 
                      transformed_pose.pose.position.x, 
                      transformed_pose.pose.position.y, 
                      transformed_pose.pose.position.z);
            
        } catch (tf::TransformException& ex) {
            ROS_ERROR("TF transform failed: %s", ex.what());
            return;
        }
    } else {
        // 不需要变换或已经在目标坐标系
        transformed_pose = *msg;
        
        if (enable_tf_transform_) {
            ROS_DEBUG("Pose already in target frame '%s'", target_frame_.c_str());
        }
    }
    
    // 提取变换后的位置信息
    double x = transformed_pose.pose.position.x;
    double y = transformed_pose.pose.position.y;
    double z = transformed_pose.pose.position.z;
    
    // 将四元数转换为 roll, pitch, yaw
    tf::Quaternion quat;
    tf::quaternionMsgToTF(transformed_pose.pose.orientation, quat);
    
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    // 更新ROS参数（使用 calculate_node 的命名空间）
    nh_.setParam("/calculate_node/test/line_point_x", x);
    nh_.setParam("/calculate_node/test/line_point_y", y);
    nh_.setParam("/calculate_node/test/line_point_z", z);
    nh_.setParam("/calculate_node/test/line_point_pitch", pitch);
    nh_.setParam("/calculate_node/test/line_point_roll", roll);
    nh_.setParam("/calculate_node/test/line_point_yaw", yaw);
    
    ROS_INFO("Updated parameters: x=%.3f, y=%.3f, z=%.3f, pitch=%.3f, roll=%.3f, yaw=%.3f (frame: %s)", 
             x, y, z, pitch, roll, yaw, transformed_pose.header.frame_id.c_str());
    
    // 发布转换后的 pose
    transformed_pose.header.stamp = ros::Time::now();
    transformed_pose_pub_.publish(transformed_pose);
}

} // namespace arm_controller
