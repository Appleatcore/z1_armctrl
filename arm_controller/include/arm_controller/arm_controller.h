#ifndef ARM_CONTROLLER_H_
#define ARM_CONTROLLER_H_

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
// #include <shared_mutex>
#include <mutex>
#include <thread>

#include "arm_api.h"
#include "arm_controller_srvs/BackToHome.h"
#include "arm_controller_srvs/CheckPoseInWorkspace.h"
#include "arm_controller_srvs/GripperControl.h"
#include "arm_controller_srvs/JoyStickControl.h"
#include "arm_controller_srvs/Plan.h"
#include "js_api.h"
#include "js_dev.h"
// #include "arm_planner.h"
#include "math_fn.h"
// #include "arm_controller/PlanAction.h"

namespace arm_controller {

enum class ArmControlFsm {
  Invalid,
  Home,
  Back2Home,
  Arrived,
  PlanMove,
  JoyStickControl
};

class ArmController {
 public:
  ArmController(const ros::NodeHandle& nh);
  ~ArmController();
  void launch();
  /**
   * @brief Subscribe and Publish Arm State
   *
   */
  void initSubsAndPubs();
  void initServers();
  void publishStates();

  void controlStep();
  void setArmControlFsm(ArmControlFsm control_fsm);

  void updateArmCmdByJs();
  /**
   * @brief Update joint trajectory
   *
   */
  void lazyPlan(const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& start,
                const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& goal,
                unsigned long ticks);
  /**
   * @brief Usually for passive control mode
   * @details When the feedforward mode enable, the function will use
   * present joint states to calculate the inverse dynamics
   * @param kp
   * @param kd
   * @param enable_feedforward_control Default false
   */
  void setControlCmd(double kp, double kd,
                     bool enable_feedforward_control = false);
  /**
   * @brief
   * @details When the feedforward enable, the function will use the arm's goal
   * joint states to calculate inverse dynamics
   * @param kp
   * @param kd
   * @param enable_free_
   */
  void setControlCmd(std::vector<double> kp, std::vector<double> kd,
                     bool enable_feedforward_control = true);
  void checkArmMotorSafe();

  // utility functions
  Z1ArmModel* getArmModel() { return arm_model_.get(); }

 public:
  // service
  bool isInWorkspaceServer(
      arm_controller_srvs::CheckPoseInWorkspace::Request& req,
      arm_controller_srvs::CheckPoseInWorkspace::Response& res);
  bool planServer(arm_controller_srvs::Plan::Request& req,
                  arm_controller_srvs::Plan::Response& res);
  bool searchPlanServer(arm_controller_srvs::Plan::Request& req,
                        arm_controller_srvs::Plan::Response& res);
  bool back2HomeServer(arm_controller_srvs::BackToHome::Request& req,
                       arm_controller_srvs::BackToHome::Response& res);
  bool jsControlServer(arm_controller_srvs::JoyStickControlRequest& req,
                       arm_controller_srvs::JoyStickControlResponse& res);
  bool gripperControlServer(arm_controller_srvs::GripperControl::Request& req,
                            arm_controller_srvs::GripperControl::Response& res);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
  // action
  // void planActionServer(const arm_controller::PlanGoalConstPtr& goal);

 protected:
  // robotic arm communication
  ArmLowCmd low_cmd_;
  ArmLowState low_state_;
  std::mutex data_mutex_;
  std::unique_ptr<Z1ArmModel> arm_model_;
  std::unique_ptr<ArmApi> arm_api_;
  // communication with the robot arm
  double communication_period_{0.002};
  std::thread communication_thread_, control_thread_;
  bool communication_state_{false}, control_state_{false};
  // robotic arm control
  double control_period_{0.005};
  double average_move_speed_{0.1};
  ArmControlFsm arm_control_fsm_{ArmControlFsm::Home};
  long unsigned int arm_control_tick_{0};
  Eigen::Matrix<double, 6, 1> arm_control_joint_pos_, arm_control_joint_vel_;
  bool arm_motor_safe_{true};
  std::vector<double> default_kp_{5, 7.5, 7.5, 5, 3.75, 2.5},
      default_kd_{500, 500, 500, 500, 500, 500};
  // moveit planner
  // std::unique_ptr<ArmPlanner> planner_;
  // planning
  const Eigen::Vector3d kCameraPosBias_E_{0.03702, 0.0, 0.0502};
  Eigen::Matrix<double, 6, 1> arm_joint_goal_, KJointHome_;
  Eigen::Matrix4d ee_pose_goal_, kEePoseHome_;
  std::vector<Eigen::Matrix<double, 6, 1>> joint_pos_trajectory_;
  std::vector<Eigen::Matrix<double, 6, 1>> joint_vel_trajectory_;
  double process_{1.0};
  QuinticInterpolationFn<Eigen::Matrix<double, 6, 1>> joint_interp_fn_;
  long unsigned int plan_max_tick_{0};
  // gripper
  double gripper_goal_{0.0};
  double gripper_current_{0.0};
  // joy stick
  js::JsState js_state_;
  std::shared_ptr<JsRos> js_api_;
  // ros
  ros::NodeHandle nh_;
  std::vector<std::string> arm_joint_names_{"joint1", "joint2", "joint3",
                                            "joint4", "joint5", "joint6"};
  sensor_msgs::JointState joint_state_msgs_;
  sensor_msgs::JointState cmd_joint_state_msgs_;
  geometry_msgs::PoseStamped ee_pose_msg_;
  std_msgs::Float64 process_msgs_;
  // publisher and subscriber
  ros::Publisher ee_pose_pub_;
  ros::Publisher process_pub_;
  ros::Publisher arm_joint_states_pub_;
  ros::Publisher arm_cmd_joint_states_pub_;
  ros::Subscriber imu_sub_;
  // server
  ros::ServiceServer back2home_server_, check_pose_in_workspace_server_,
      plan_server_, search_plan_server_, js_control_server_,
      gripper_control_server_;
  // action server
  // std::unique_ptr<actionlib::SimpleActionServer<arm_controller::PlanAction>>
  //     plan_action_server_;
};

}  // namespace arm_controller

#endif
