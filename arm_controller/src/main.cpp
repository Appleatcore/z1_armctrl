#include <ros/ros.h>

#include "arm_controller/arm_controller.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "arm_controller_node");
  ros::NodeHandle nh("~");  // 使用私有命名空间
  ros::AsyncSpinner spinner(2);
  spinner.start();
  arm_controller::ArmController arm_controller(nh);
  arm_controller.launch();
  ros::waitForShutdown();
  // ros::spin();
}
