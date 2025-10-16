#ifndef POSE_PARAM_UPDATER_H
#define POSE_PARAM_UPDATER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <string>

namespace arm_controller {

/**
 * @brief 订阅PoseStamped话题并更新ROS参数的节点类
 * 
 * 该类订阅一个PoseStamped话题,提取位置信息(x, y, z),
 * 并将这些值更新到ROS参数服务器上的指定参数中
 */
class PoseParamUpdater {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    PoseParamUpdater(ros::NodeHandle& nh);
    
    /**
     * @brief 析构函数
     */
    ~PoseParamUpdater();

private:
    /**
     * @brief PoseStamped消息的回调函数
     * @param msg 接收到的PoseStamped消息
     */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Publisher transformed_pose_pub_;  // 发布转换后的 pose
    tf::TransformListener tf_listener_;
    
    // 参数名称
    std::string param_x_;
    std::string param_y_;
    std::string param_z_;
    
    // 话题名称
    std::string pose_topic_;
    
    // TF参数
    std::string target_frame_;  // 目标坐标系
    bool enable_tf_transform_;  // 是否启用TF变换
    double tf_timeout_;         // TF等待超时
};

} // namespace arm_controller

#endif // POSE_PARAM_UPDATER_H
