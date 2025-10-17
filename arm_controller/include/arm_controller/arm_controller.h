#ifndef ARM_CONTROLLER_H_
#define ARM_CONTROLLER_H_

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
// #include <shared_mutex>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <thread>

#include "arm_api.h"
#include "arm_controller_srvs/BackToHome.h"
#include "arm_controller_srvs/CheckPoseInWorkspace.h"
#include "arm_controller_srvs/GripperControl.h"
#include "arm_controller_srvs/JoyStickControl.h"
#include "arm_controller_srvs/Plan.h"
#include "arm_controller_srvs/PlanToDefault.h"
#include "arm_controller_srvs/PlanTofivepoint.h"
#include "js_api.h"
#include "js_dev.h"
// #include "arm_planner.h"
#include "math_fn.h"
// #include "arm_controller/PlanAction.h"

namespace arm_controller {

// 3D直线的参数化表示
struct Line3D {
  Eigen::Vector3d point;      // 直线上的一个点
  Eigen::Vector3d direction;  // 方向向量（单位向量）
  
  // 在直线上采样点
  std::vector<Eigen::Vector3d> samplePoints(double t_start, double t_end, 
                                             int num_samples) const;
};

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
  bool IsPlanServer(arm_controller_srvs::Plan::Request& req,
                    arm_controller_srvs::Plan::Response& res);
  bool searchPlanServer(arm_controller_srvs::Plan::Request& req,
                        arm_controller_srvs::Plan::Response& res);
  bool back2HomeServer(arm_controller_srvs::BackToHome::Request& req,
                       arm_controller_srvs::BackToHome::Response& res);
  bool planToDefaultServer(arm_controller_srvs::PlanToDefault::Request& req,
                           arm_controller_srvs::PlanToDefault::Response& res);
  bool jsControlServer(arm_controller_srvs::JoyStickControlRequest& req,
                       arm_controller_srvs::JoyStickControlResponse& res);
  bool gripperControlServer(arm_controller_srvs::GripperControl::Request& req,
                            arm_controller_srvs::GripperControl::Response& res);
  bool planToFivePointServer(arm_controller_srvs::PlanTofivepoint::Request& req,
                             arm_controller_srvs::PlanTofivepoint::Response& res);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
  void executeProcessCallback(const std_msgs::Float64::ConstPtr& msg);
  
  /**
   * @brief 规划并执行到目标位姿的运动
   * @param target_pose 目标位姿 (geometry_msgs::Pose)
   * @return true 如果规划成功，false 否则
   */
  bool planToTargetPose(const geometry_msgs::Pose& target_pose);
  
  /**
   * @brief 控制 Joint6 和夹爪位置
   * @param joint6_pos Joint6 目标位置（弧度，范围: -3.14 到 3.14）
   * @param gripper_pos 夹爪目标位置（0.0 张开到 -0.85 闭合）
   * @return true 如果控制成功，false 否则
   */
  bool controlJoint6AndGripper(double joint6_pos, double gripper_pos);
  
  /**
   * @brief 对位姿列表按照到参考点的距离进行排序
   * @param poses 输入的位姿列表
   * @param reference_point 参考点（例如管道中心点）
   * @param target_distance 目标距离，距离此值最近的点排在前面
   * @return 排序后的位姿列表
   */
  std::vector<geometry_msgs::PoseStamped> sortPosesByDistanceToPoint(
      const std::vector<geometry_msgs::PoseStamped>& poses,
      const Eigen::Vector3d& reference_point,
      double target_distance);
  
  /**
   * @brief 执行到目标点的运动并控制夹爪
   * @param target_pose 目标位姿
   * @param pitch 夹爪 pitch 角度（弧度）
   * @param roll 夹爪 roll 角度（弧度）
   * @param timeout_seconds 超时时间（秒），默认 10 秒
   * @return true 如果执行成功，false 否则
   */
  bool executeMotionToTarget(const geometry_msgs::Pose& target_pose,
                             double pitch,
                             double roll,
                             double timeout_seconds = 10.0);
  
  /**
   * @brief 计算直线绕旋转轴旋转后的解析式
   * @param line 原始直线
   * @param rotation_center 旋转中心点
   * @param rotation_axis 旋转轴方向（单位向量）
   * @param angle_rad 旋转角度（弧度）
   * @return 旋转后的直线
   */
  Line3D rotateLine(const Line3D& line,
                    const Eigen::Vector3d& rotation_center,
                    const Eigen::Vector3d& rotation_axis,
                    double angle_rad) const;
  
  /**
   * @brief 计算直线沿指定方向平移指定距离后的解析式
   * @param line 原始直线
   * @param direction 平移方向向量
   * @param distance 平移距离
   * @param t_start 采样起始参数
   * @param t_end 采样结束参数
   * @param num_samples 采样点数
   * @param reachable_poses 输出参数：可达的位姿列表
   * @param pitch 输出参数：pitch 角度
   * @param roll 输出参数：roll 角度
   * @return 平移后的直线
   */
  Line3D translateLine(const Line3D& line,
                       const Eigen::Vector3d& direction,
                       double distance,
                       double t_start,
                       double t_end,
                       int num_samples,
                       std::vector<geometry_msgs::PoseStamped>& reachable_poses,
                       double& pitch,
                       double& roll);
  
  /**
   * @brief 计算让相机朝向指定方向所需的末端姿态角度（自动计算最优roll角）
   * @param camera_direction 期望的相机朝向（单位向量）
   * @param pitch 输出参数：末端需要的pitch角（弧度）
   * @param roll 输出参数：末端需要的roll角（弧度，自动计算）
   * @param yaw 输出参数：参考yaw角（需通过机械臂位置实现）
   * @param radius 目标距离（米）
   * @return 是否成功计算
   */
  bool calculateCameraOrientation(const Eigen::Vector3d& camera_direction,
                                  double& pitch,
                                  double& roll,
                                  double& yaw,
                                  double radius = 1.0) const;
  
  /**
   * @brief 生成多个旋转角度的直线并在每条直线上采样点
   * @param line 原始直线
   * @param rotation_center 旋转中心点
   * @param rotation_axis 旋转轴方向
   * @param num_rotations 旋转的数量
   * @param angle_step 每次旋转的角度步长（弧度）
   * @param t_start 采样参数起始值
   * @param t_end 采样参数结束值
   * @param num_samples 每条直线上的采样点数
   * @param reachable_poses 输出参数，存储可达位姿
   * @param pitch 输出参数，相机 pitch 角度（弧度）
   * @param roll 输出参数，相机 roll 角度（弧度）
   * @return 所有旋转直线及其采样点的集合
   */
  std::vector<std::pair<Line3D, std::vector<Eigen::Vector3d>>> 
  generateRotatedLinesWithSamples(const Line3D& line,
                                   const Eigen::Vector3d& rotation_center,
                                   const Eigen::Vector3d& rotation_axis,
                                   int num_rotations,
                                   double angle_step,
                                   double t_start,
                                   double t_end,
                                   int num_samples,
                                   std::vector<geometry_msgs::PoseStamped>& reachable_poses,
                                   double& pitch,
                                   double& roll);
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
  // execute process control
  double execute_process_{0.0};  // 控制是否执行过程（>= 1.0 表示执行中，0.0 表示完成）
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
  ros::Publisher target_poses_pub_;  // 发布目标点位供 RViz 可视化 (PoseArray)
  ros::Subscriber imu_sub_;
  ros::Subscriber execute_process_sub_;  // 订阅执行控制信号
  // server
  ros::ServiceServer back2home_server_, check_pose_in_workspace_server_,
      plan_server_, search_plan_server_, rotation_search_plan_server_,
      plan_to_default_server_, js_control_server_, gripper_control_server_,
      plan_to_five_point_server_;
  // action server
  // std::unique_ptr<actionlib::SimpleActionServer<arm_controller::PlanAction>>
  //     plan_action_server_;
};

}  // namespace arm_controller

#endif
