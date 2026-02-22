#ifndef ARM_CONTROLLER_H_
#define ARM_CONTROLLER_H_

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>  // <-- 新增 (tf2 广播时使用)
#include <tf2/LinearMath/Matrix3x3.h>        // <-- 替换 tf/transform_datatypes.h
#include <tf2/LinearMath/Quaternion.h>       // <-- 替换 tf/transform_datatypes.h
#include <tf2/LinearMath/Transform.h>        // <-- 替换 tf/transform_datatypes.h
#include <tf2/LinearMath/Vector3.h>          // <-- 替换 tf/transform_datatypes.h
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>  // <-- 新增 (用于 static_br_ptr_)
#include <tf2_ros/transform_broadcaster.h>         // <-- 替换 tf/transform_broadcaster.h
#include <tf2_ros/transform_listener.h>
// #include <shared_mutex>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <thread>

#include "arm_api.h"
#include "arm_controller_srvs/BackToHome.h"
#include "arm_controller_srvs/CameraToLink00.h"
#include "arm_controller_srvs/CheckPoseInWorkspace.h"
#include "arm_controller_srvs/JoyStickControl.h"
#include "arm_controller_srvs/Plan.h"
#include "arm_controller_srvs/PlanToDefault.h"
#include "arm_controller_srvs/PlanToHorizon.h"
#include "arm_controller_srvs/PlanToHorizonHeight.h"
#include "arm_controller_srvs/getgoalandangle.h"
#include "arm_controller_srvs/planandgrippercontrol.h"
#include "arm_controller_srvs/zedlinktolink00.h"
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
  std::vector<Eigen::Vector3d> samplePoints(double t_start, double t_end, int num_samples) const;
};

// 3D直线的参数化表示
struct SampleLine3D {
  std::string name;
  Line3D line;
  double pitch = 0.0;  //
  double roll = 0.0;   //
  // 采样参数
  double sample_start;
  double sample_end;
  int num_samples;
  double target_distance;
  // 结果
  std::vector<geometry_msgs::PoseStamped> reachable_poses;
  SampleLine3D() = default;

  /**
   * @brief 用于初始化的构造函数
   * @param name_in "MID", "LEFT1", etc.
   * @param point_in 直线的起点
   * @param direction_in 直线的方向
   * @param start 采样的 t_start
   * @param end 采样的 t_end
   * @param num 采样点数量
   * @param dist 目标距离
   */
  SampleLine3D(const std::string& name_in, const Eigen::Vector3d& point_in, const Eigen::Vector3d& direction_in, double start, double end, int num, double dist)
      : name(name_in), sample_start(start), sample_end(end), num_samples(num), target_distance(dist) {
    // 构造函数体，用于初始化嵌套的 line 成员
    line.point = point_in;
    // 确保方向向量总是单位向量
    line.direction = direction_in.normalized();
  }
};
class PinocchioIK;
enum class ArmControlFsm { Invalid, Home, Back2Home, Arrived, PlanMove, JoyStickControl };

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
  void lazyPlan(const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& start, const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& goal, unsigned long ticks);
  /**
   * @brief Usually for passive control mode
   * @details When the feedforward mode enable, the function will use
   * present joint states to calculate the inverse dynamics
   * @param kp
   * @param kd
   * @param enable_feedforward_control Default false
   */
  void setControlCmd(double kp, double kd, bool enable_feedforward_control = false);
  /**
   * @brief
   * @details When the feedforward enable, the function will use the arm's goal
   * joint states to calculate inverse dynamics
   * @param kp
   * @param kd
   * @param enable_free_
   */
  void setControlCmd(std::vector<double> kp, std::vector<double> kd, bool enable_feedforward_control = true);
  void checkArmMotorSafe();

  // utility functions
  ArmModel* getArmModel() { return arm_model_; }

 public:
  // service
  bool isInWorkspaceServer(arm_controller_srvs::CheckPoseInWorkspace::Request& req, arm_controller_srvs::CheckPoseInWorkspace::Response& res);
  bool planServer(arm_controller_srvs::Plan::Request& req, arm_controller_srvs::Plan::Response& res);
  bool IsPlanServer(arm_controller_srvs::Plan::Request& req, arm_controller_srvs::Plan::Response& res);
  bool searchPlanServer(arm_controller_srvs::Plan::Request& req, arm_controller_srvs::Plan::Response& res);
  bool back2HomeServer(arm_controller_srvs::BackToHome::Request& req, arm_controller_srvs::BackToHome::Response& res);
  bool planToDefaultServer(arm_controller_srvs::PlanToDefault::Request& req, arm_controller_srvs::PlanToDefault::Response& res);
  bool planToHorizonServer(arm_controller_srvs::PlanToHorizon::Request& req, arm_controller_srvs::PlanToHorizon::Response& res);
  bool planToHorizonHeightServer(arm_controller_srvs::PlanToHorizonHeight::Request& req, arm_controller_srvs::PlanToHorizonHeight::Response& res);
  bool jsControlServer(arm_controller_srvs::JoyStickControlRequest& req, arm_controller_srvs::JoyStickControlResponse& res);
  bool planAndGripperControlServer(arm_controller_srvs::planandgrippercontrol::Request& req, arm_controller_srvs::planandgrippercontrol::Response& res);
  bool getGoalAndAngleServer(arm_controller_srvs::getgoalandangle::Request& req, arm_controller_srvs::getgoalandangle::Response& res);
  bool getCrossGoalAndAngleServer(arm_controller_srvs::getgoalandangle::Request& req, arm_controller_srvs::getgoalandangle::Response& res);
  bool PlanTouchGoalAndAngleServer(arm_controller_srvs::getgoalandangle::Request& req, arm_controller_srvs::getgoalandangle::Response& res);
  bool zedLinkToLink00Server(arm_controller_srvs::zedlinktolink00::Request& req, arm_controller_srvs::zedlinktolink00::Response& res);
  bool cameraToLink00Server(arm_controller_srvs::CameraToLink00::Request& req, arm_controller_srvs::CameraToLink00::Response& res);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
  void executeProcessCallback(const std_msgs::Float64::ConstPtr& msg);

  /**
   * @brief 规划并执行到目标位姿的运动
   * @param target_pose 目标位姿 (geometry_msgs::Pose)
   * @return true 如果规划成功，false 否则
   */
  bool planToTargetPose(const geometry_msgs::Pose& target_pose, const double& joint6_pos = 0.0, const bool& use_manual_joint6 = false);

  /**
   * @brief 移动到默认点（用于安全过渡）
   * @return true 如果成功移动到默认点，false 否则
   */
  bool goToDefaultPoint();

  /**
   * @brief 对位姿列表按照到参考点的距离进行排序
   * @param poses 输入的位姿列表
   * @param reference_point 参考点（例如管道中心点）
   * @param target_distance 目标距离，距离此值最近的点排在前面
   * @return 排序后的位姿列表
   */
  std::vector<geometry_msgs::PoseStamped> sortPosesByDistanceToPoint(const std::vector<geometry_msgs::PoseStamped>& poses, const Eigen::Vector3d& reference_point, const Eigen::Vector3d& line_direction,
                                                                     double target_distance);

  /**
   * @brief 执行到目标点的运动并控制夹爪
   * @param target_pose 目标位姿
   * @param pitch 夹爪 pitch 角度（弧度）
   * @param roll 夹爪 roll 角度（弧度）
   * @param timeout_seconds 超时时间（秒），默认 10 秒
   * @return true 如果执行成功，false 否则
   */
  bool executeMotionToTarget(const geometry_msgs::Pose& target_pose, double pitch, double roll, double timeout_seconds = 10.0);

  /**
   * @brief 计算直线绕旋转轴旋转后的解析式（纯几何变换，不涉及采样和IK检测）
   * @param line 原始直线
   * @param rotation_center 旋转中心点
   * @param rotation_axis 旋转轴方向（单位向量）
   * @param angle_rad 旋转角度（弧度）
   * @return 旋转后的直线
   */
  Line3D rotateLine(const Line3D& line, const Eigen::Vector3d& rotation_center, const Eigen::Vector3d& rotation_axis, double angle_rad) const;

  /**
   * @brief 计算直线沿指定方向平移指定距离后的解析式（纯几何变换，不涉及采样和IK检测）
   * @param line 原始直线
   * @param direction 平移方向向量
   * @param distance 平移距离
   * @return 平移后的直线
   */
  Line3D translateLineGeometry(const Line3D& line, const Eigen::Vector3d& direction, double distance) const;

  /**
   * @brief 对直线进行采样并检测每个采样点的可达性
   * @param line 要采样的直线
   * @param t_start 采样起始参数
   * @param t_end 采样结束参数
   * @param num_samples 采样点数
   * @param pitch 输出参数：相机 pitch 角度（弧度）
   * @param roll 输出参数：相机 roll 角度（弧度）
   * @return 可达的位姿列表
   */
  std::vector<geometry_msgs::PoseStamped> sampleAndCheckReachability(Line3D& line, double t_start, double t_end, int num_samples, double& pitch, double& roll, geometry_msgs::Pose& debug_camera_poses_msg);

  /**
   * @brief 对直线进行采样并检测每个采样点的可达性
   * @param line 要采样的直线
   * @param t_start 采样起始参数
   * @param t_end 采样结束参数
   * @param num_samples 采样点数
   * @param pitch 输出参数：相机 pitch 角度（弧度）
   * @param roll 输出参数：相机 roll 角度（弧度）
   * @return 可达的位姿列表
   */
  Line3D translateLine(const Line3D& line, const Eigen::Vector3d& direction, double distance, double t_start, double t_end, int num_samples, std::vector<geometry_msgs::PoseStamped>& reachable_poses, double& pitch,
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
  bool calculateCameraOrientation(const Eigen::Vector3d& camera_direction, double& pitch, double& roll, double& yaw, geometry_msgs::Pose& out_cam_pose) const;
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
  std::vector<std::pair<Line3D, std::vector<Eigen::Vector3d>>> generateRotatedLinesWithSamples(const Line3D& line, const Eigen::Vector3d& rotation_center, const Eigen::Vector3d& rotation_axis, int num_rotations,
                                                                                               double angle_step, double t_start, double t_end, int num_samples, std::vector<geometry_msgs::PoseStamped>& reachable_poses,
                                                                                               double& pitch, double& roll, ros::Publisher* line_publisher = nullptr, int viz_points = 50);
  // action
  geometry_msgs::PoseStamped Line3DToPoseStamped(Line3D& line3d_posestamped);
  geometry_msgs::Pose Line3DToPose(Line3D& line3d_pose);
  geometry_msgs::PoseArray createLineVisualization(Line3D& line, double t_start, double t_end, int num_points);
  bool IsPlan(geometry_msgs::Pose& target_pose);
  // void planActionServer(const arm_controller::PlanGoalConstPtr& goal);

 protected:
  // robotic arm communication
  ArmLowCmd low_cmd_;
  ArmLowState low_state_;
  std::mutex data_mutex_;
  // std::unique_ptr<Z1ArmModel> arm_model_;
  ArmModel* arm_model_{nullptr};
  std::unique_ptr<ArmApi> arm_api_;
  // communication with the robot arm
  double communication_period_{0.002};
  std::thread communication_thread_, control_thread_;
  bool communication_state_{false}, control_state_{false};
  // robotic arm control
  double control_period_{0.005};
  double average_move_speed_{0.15};
  ArmControlFsm arm_control_fsm_{ArmControlFsm::Home};
  long unsigned int arm_control_tick_{0};
  Eigen::Matrix<double, 6, 1> arm_control_joint_pos_, arm_control_joint_vel_;
  bool arm_motor_safe_{true};
  // std::vector<double> default_kp_{5, 7.5, 7.5, 5, 3.75, 2.5},
  // default_kd_{500, 500, 500, 500, 500, 500};
  std::vector<double> default_kp_{20, 30, 30, 20, 15, 25}, default_kd_{2000, 2000, 2000, 2000, 2000, 2000};
  Eigen::Matrix<double, 6, 1> arm_control_default_joint_pos_;
  Eigen::Matrix<double, 6, 1> arm_control_horizon_joint_pos_;
  Eigen::Matrix<double, 6, 1> arm_control_horizon_height_joint_pos_;
  // moveit planner
  // std::unique_ptr<ArmPlanner> planner_;
  // planning
  const Eigen::Vector3d kCameraPosBias_E_{0.0, 0.0, 0.0};
  const Eigen::Vector3d Z1Arm_PosBias_E_{0.01, 0.0, 0.004};
  const Eigen::Vector3d Pinocchio_PosBias_E_{0.0, 0.0, 0.004};
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
  std::vector<std::string> arm_joint_names_{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "jointGripper"};
  sensor_msgs::JointState joint_state_msgs_;
  sensor_msgs::JointState cmd_joint_state_msgs_;
  geometry_msgs::PoseStamped ee_pose_msg_;
  std_msgs::Float64 process_msgs_;
  // publisher and subscriber
  ros::Publisher ee_pose_pub_;
  ros::Publisher process_pub_;
  ros::Publisher center_pub_;
  ros::Publisher arm_joint_states_pub_;
  ros::Publisher arm_cmd_joint_states_pub_;
  ros::Publisher target_poses_pub_;             // 发布目标点位供 RViz 可视化 (PoseArray)
  ros::Publisher poses_out1_pub_;               // 发布 OUT1 可达点
  ros::Publisher poses_out2_pub_;               // 发布 OUT2 可达点
  ros::Publisher poses_mid_pub_;                // 发布 MID 可达点
  ros::Publisher poses_half1_pub_;              // 发布 HALF1 可达点
  ros::Publisher poses_half2_pub_;              // 发布 HALF2 可达点
  ros::Publisher poses_mid_all_pub_;            // 发布 MID 所有采样点（包括可达和不可达）
  ros::Publisher transformed_input_pub_;        // 发布变换后的输入姿态
  ros::Publisher camera_transformed_pose_pub_;  // 发布相机坐标系到link00坐标系的变换结果
  // 发布5条直线的可视化
  ros::Publisher line_mid_pub_;          // 发布 MID 直线
  ros::Publisher line_out1_pub_;         // 发布 OUT1 直线
  ros::Publisher line_out2_pub_;         // 发布 OUT2 直线
  ros::Publisher line_half1_pub_;        // 发布 HALF1 直线
  ros::Publisher line_half2_pub_;        // 发布 HALF2 直线
  ros::Publisher reference_points_pub_;  // 发布五个参考点

  // 十字的可视化发布器
  ros::Publisher cross_line_mid_pub_;                // 发布交叉模式 MID 直线
  ros::Publisher cross_line_left1_pub_;              // 发布交叉模式 LEFT1 直线
  ros::Publisher cross_line_left2_pub_;              // 发布交叉模式 LEFT2 直线
  ros::Publisher cross_line_top1_pub_;               // 发布交叉模式 TOP1 直线
  ros::Publisher cross_line_top2_pub_;               // 发布交叉模式 TOP2 直线
  ros::Publisher cross_reference_points_pub_;        // 发布交叉模式参考点
  ros::Publisher cross_center_pub_;                  // 发布交叉模式中心点
  ros::Publisher cross_rotation_transform_top_pub;   // 发布LEFT旋转轴
  ros::Publisher cross_rotation_transform_left_pub;  // 发布TOP旋转轴
  //   tf::TransformBroadcaster tf_broadcaster_; // TF 广播器，用于发布坐标变换
  tf2_ros::TransformBroadcaster tf_broadcaster_;  // TF 广播器，用于发布坐标变换
  tf2_ros::Buffer tf_buffer_;                     // TF2 缓冲区，用于查询坐标变换
  tf2_ros::TransformListener tf_listener_;        // TF2 监听器
  ros::Subscriber imu_sub_;
  ros::Subscriber execute_process_sub_;                                 // 订阅执行控制信号
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_br_ptr_;       // 动态TF广播器（link00 to object_frame）
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_br_ptr_;  // 90度静态TF广播器
  geometry_msgs::PoseArray debug_camera_poses_msg;
  // server
  ros::ServiceServer back2home_server_, check_pose_in_workspace_server_, plan_server_, search_plan_server_, rotation_search_plan_server_, plan_to_default_server_, plan_to_horizon_server_, plan_to_horizon_height_server_, js_control_server_,
      gripper_control_server_, plan_to_five_point_server_, plan_and_gripper_control_server_, get_goal_and_angle_server_, cross_get_goal_and_angle_server_, zed_link_to_link00_server_, camera_to_link00_server_;
  // action server
  // std::unique_ptr<actionlib::SimpleActionServer<arm_controller::PlanAction>>
  //     plan_action_server_;

  // #ifdef USE_PINOCCHIO
  // Pinocchio 运动学求解器（使用智能指针避免默认构造函数问题）
  std::unique_ptr<PinocchioIK> pinocchio_ik_;
  // #endif
};

}  // namespace arm_controller

#endif
