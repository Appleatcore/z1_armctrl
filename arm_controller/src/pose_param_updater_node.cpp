#include <ros/ros.h>
#include "pose_param_updater.h"

/**
 * @brief PoseParamUpdater节点的主函数
 * 
 * 该节点订阅PoseStamped话题,并将接收到的位置数据(x, y, z)
 * 更新到ROS参数服务器的指定参数中
 * 
 * 使用方法:
 * rosrun arm_controller pose_param_updater_node _pose_topic:=/your/pose/topic
 */
int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "pose_param_updater_node");
    ros::NodeHandle nh("~");
    
    ROS_INFO("Starting PoseParamUpdater node...");
    
    try {
        // 创建PoseParamUpdater对象
        arm_controller::PoseParamUpdater updater(nh);
        
        // 进入ROS事件循环
        ros::spin();
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception in PoseParamUpdater: %s", e.what());
        return 1;
    }
    
    return 0;
}
