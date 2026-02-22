// clang-format off
#include <pinocchio/fwd.hpp>
#include "arm_controller/pinocchio_ik.h"
// clang-format on
#include "arm_controller/arm_controller.h"
// #include <Eigen/Geometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "arm_controller/geometry_utils.h"
#include "geometry_msgs/PoseStamped.h"

namespace arm_controller {

// 前向声明
geometry_msgs::PoseArray createLineVisualization(const Line3D& line, double t_start, double t_end, int num_points);

ArmController::ArmController(const ros::NodeHandle& nh) : nh_(nh), tf_listener_(tf_buffer_) {
  arm_api_ = std::make_unique<ArmApi>();
  // arm_model_ = std::make_unique<Z1ArmModel>();
  arm_model_ = arm_api_->getArmModel();
  // 修改关节限制：限制 Joint[2] 最小角度以防打到相机
  // arm_model_->setJointQMin(2, -1.9);  // Joint[2] (index 2) 最小角度 -2.0 rad (-115°)
  // ROS_INFO("Joint[3] min limit set to: -2.0 rad (-115 deg)");
  // arm_model_->setJointQMax(1, 2.54);
  // ROS_INFO("Joint[1] max limit set to: 2.54 rad (145.5 deg)");

  // 初始化两个广播器
  dynamic_br_ptr_ = std::make_unique<tf2_ros::TransformBroadcaster>();
  static_br_ptr_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>();

  // 在启动时发布静态变换
  // (input_frame -> object_frame)，让object_frame与input_frame对齐，x轴指向前方
  geometry_msgs::TransformStamped t_static_msg, t_static_msg_flower, t_static_msg_combine;

  t_static_msg_flower.header.stamp = ros::Time::now();
  t_static_msg_flower.header.frame_id = "input_frame";
  t_static_msg_flower.child_frame_id = "flower_frame";
  t_static_msg_flower.transform.translation.x = 0.0;
  t_static_msg_flower.transform.translation.y = 0.0;
  t_static_msg_flower.transform.translation.z = 0.0;
  tf2::Quaternion q_mod_flower, q_mod_combine;
  q_mod_flower.setRPY(0, M_PI / 2.0, 0);  // Roll=0, Pitch=-90 deg, Yaw=0
  // 使用 tf2::toMsg 辅助函数转换
  t_static_msg_flower.transform.rotation = tf2::toMsg(q_mod_flower);
  static_br_ptr_->sendTransform(t_static_msg_flower);

  t_static_msg_combine.header.stamp = ros::Time::now();
  t_static_msg_combine.header.frame_id = "input_frame";
  t_static_msg_combine.child_frame_id = "combine_frame";
  t_static_msg_combine.transform.translation.x = 0.0;
  t_static_msg_combine.transform.translation.y = 0.0;
  t_static_msg_combine.transform.translation.z = 0.0;
  q_mod_combine.setRPY(0, 0, -M_PI / 2.0);  // Roll=0, Pitch=0, Yaw=90 deg
  // 使用 tf2::toMsg 辅助函数转换
  t_static_msg_combine.transform.rotation = tf2::toMsg(q_mod_combine);
  static_br_ptr_->sendTransform(t_static_msg_combine);

  // communicate with the manipulator
  communication_state_ = true;
  if (communication_thread_.joinable()) {
    communication_thread_.join();
  }
  communication_thread_ = std::thread([this]() {
    Rate rate(static_cast<int>(1 / communication_period_));
    // auto start_time = std::chrono::system_clock::now();
    while (communication_state_) {
      auto end_time = std::chrono::system_clock::now();
      // std::cout << "Communication period: "
      //           << std::chrono::duration<double, std::ratio<1, 1000>>(
      //                  end_time - start_time)
      //                  .count()
      //           << std::endl;
      if (!low_state_.arm_connected) {
        rate.sync();
      }
      arm_api_->sendRecv();
      data_mutex_.lock();
      arm_api_->setCmd(low_cmd_);
      arm_api_->getState(low_state_);
      data_mutex_.unlock();
      // start_time = end_time;
      rate.sleep();
    }
  });
  // Initialize the parameters
  arm_control_joint_pos_.setZero();
  arm_control_joint_vel_.setZero();
  // default_kp_ = {15, 7.5, 7.5, 5, 10, 2.5};
  // default_kd_ = {1500, 500, 500, 500, 1200, 500};
  // Waiting for establishing connection with the manipulator
  while (!low_state_.arm_connected) {
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
  // 设置夹爪增益
  low_cmd_.setGripperGain(15.0, 20.0);  // 使用默认增益
  double default_joint_pos_1 = 2.54;
  double default_joint_pos_2 = -1.12;  
  double default_joint_pos_3 = -1.0;
  double horizon_joint_pos_1 = 2.54;
  double horizon_joint_pos_2 = -1.27;
  double horizon_joint_pos_3 = -0.35;
  double horizon_height_joint_pos_1 = 2.54;
  double horizon_height_joint_pos_2 = -1.27;
  double horizon_height_joint_pos_3 = -0.35;
  nh_.param("test/horizon_joint_pos_1", horizon_joint_pos_1, 2.54);
  nh_.param("test/horizon_joint_pos_2", horizon_joint_pos_2, -1.27);
  nh_.param("test/horizon_joint_pos_3", horizon_joint_pos_3, -0.35);
  nh_.param("test/horizon_height_joint_pos_1", horizon_height_joint_pos_1, 2.54);
  nh_.param("test/horizon_height_joint_pos_2", horizon_height_joint_pos_2, -1.27);
  nh_.param("test/horizon_height_joint_pos_3", horizon_height_joint_pos_3, -0.35);
  nh_.param("test/default_joint_pos_1", default_joint_pos_1, 2.54);
  nh_.param("test/default_joint_pos_2", default_joint_pos_2, -1.12);
  nh_.param("test/default_joint_pos_3", default_joint_pos_3, -1.0);
  // Set default joint position
  arm_control_default_joint_pos_ << 0.0, default_joint_pos_1, default_joint_pos_2, default_joint_pos_3, 0.0, 0.0;
  arm_control_horizon_joint_pos_ << 0.0, horizon_joint_pos_1, horizon_joint_pos_2, horizon_joint_pos_3, 0.0, 0.0;
  arm_control_horizon_height_joint_pos_ << 0.0, horizon_height_joint_pos_1, horizon_height_joint_pos_2, horizon_height_joint_pos_3, 0.0, 0.0;
  // Planning
  KJointHome_ << 0, 0, 0, 0, 0, 0;
  kEePoseHome_.setIdentity();
  kEePoseHome_.block<3, 1>(0, 3) = low_state_.endPosture.tail<3>();
  rpyToRot(low_state_.endPosture.head<3>(), kEePoseHome_.block<3, 3>(0, 0));
  arm_joint_goal_.setZero();
  ee_pose_goal_ = kEePoseHome_;
  plan_max_tick_ = 0;
  joint_pos_trajectory_.clear();
  joint_vel_trajectory_.clear();
  // Joy stick
  js_api_ = JsRos::make<js::BeitongMapping>(nh);
  // js_api_ = JsRos::make<js::XboxMapping>(nh);
  js_api_->getState(js_state_);
  // Subscriber, publisher and servers
  initSubsAndPubs();
  initServers();
  // 初始化 Pinocchio IK 求解器
  std::string urdf_path;
  // 优先从 parameter server 获取
  if (nh_.getParam("urdf_path", urdf_path)) {
    try {
      // 使用 make_unique 创建实例
      pinocchio_ik_ = std::make_unique<PinocchioIK>(urdf_path, "camera_link");  // camera_optical_frame，gripperStator，camera_link
      ROS_INFO("Pinocchio IK initialized successfully from: %s", urdf_path.c_str());
      pinocchio_ik_->setJointLimitMin(2, -1.9);// Joint[2] (index 2) 最小角度 -2.0 rad (-115°)
      pinocchio_ik_->setJointLimitMax(1, 2.62);
      pinocchio_ik_->setJointLimitMax(0, 1.57);
      pinocchio_ik_->setJointLimitMin(0, -1.57);
      ROS_INFO("Joint[0] max limit set to: 1.57 rad (90 deg)");
      ROS_INFO("Joint[0] min limit set to: -1.57 rad (-90 deg)");
      ROS_INFO("Joint[1] max limit set to: 2.62 rad (150 deg)");
      ROS_INFO("Joint[2] min limit set to: -1.9 rad (-115 deg)");
    } catch (const std::exception& e) {
      ROS_ERROR("Pinocchio Init Failed: %s", e.what());
    }
  } else {
    ROS_ERROR("Failed to get param 'urdf_path'. IK will not work!");
  }
}

ArmController::~ArmController() {
  communication_state_ = false;
  control_state_ = false;
  if (communication_thread_.joinable()) {
    communication_thread_.join();
  }
  if (control_thread_.joinable()) {
    control_thread_.join();
  }
}

void ArmController::launch() {
  if (control_thread_.joinable()) {
    control_thread_.join();
  }
  control_state_ = true;
  control_thread_ = std::thread([this]() {
    Rate rate(static_cast<int>(1 / control_period_));
    std::cout << "Manipulator controller started" << std::endl;

    // test - 从 ROS 参数服务器读取配置
    double line_x, line_y, line_z;
    double center_x, center_y, center_z;
    double line_pitch_link00, line_yaw_link00, line_roll_link00;
    double line_pitch, line_yaw, line_roll;
    double rotation_angle_deg, sample_start, sample_end;
    double origin_direction_x, origin_direction_y, origin_direction_z;
    double rotation_axis_x, rotation_axis_y, rotation_axis_z;
    // double translation_axis_x, translation_axis_y, translation_axis_z;
    int num_samples, num_rotations;
    double angle_step_deg;
    // 构造一个默认的target_pose
    geometry_msgs::Pose default_target_pose;

    nh_.param("test/line_point_x", line_x, 1.0);
    nh_.param("test/line_point_y", line_y, 0.0);
    nh_.param("test/line_point_z", line_z, 0.15);
    nh_.param("test/line_point_pitch", line_pitch_link00, 0.0);
    nh_.param("test/line_point_yaw", line_yaw_link00, 0.0);
    nh_.param("test/line_point_roll", line_roll_link00, 0.0);
    nh_.param("test/rotation_angle_deg", rotation_angle_deg, 45.0);
    nh_.param("test/sample_start", sample_start, -1.0);
    nh_.param("test/sample_end", sample_end, 0.0);
    nh_.param("test/num_samples", num_samples, 10);
    nh_.param("test/num_rotations", num_rotations, 1);
    nh_.param("test/angle_step_deg", angle_step_deg, 45.0);
    nh_.param("test/default_target_pose_x", default_target_pose.position.x, 0.5);
    nh_.param("test/default_target_pose_y", default_target_pose.position.y, 0.0);
    nh_.param("test/default_target_pose_z", default_target_pose.position.z, 0.15);
    nh_.param("test/default_target_pose_orientation_x", default_target_pose.orientation.x, 0.0);
    nh_.param("test/default_target_pose_orientation_y", default_target_pose.orientation.y, 0.0);
    nh_.param("test/default_target_pose_orientation_z", default_target_pose.orientation.z, 0.0);
    nh_.param("test/default_target_pose_orientation_w", default_target_pose.orientation.w, 1.0);

    while (control_state_) {
      // auto end_time = std::chrono::system_clock::now();
      // std::cout << "Control period: "
      //           << std::chrono::duration<double, std::ratio<1, 1000>>(
      //                  end_time - start_time)
      //                  .count()
      //           << std::endl;
      controlStep();
      publishStates();
      // start_time = end_time;
      rate.sleep();
      // if (!debug_camera_poses_msg.poses.empty()) {
      //   debug_camera_poses_msg.header.stamp = ros::Time::now();
      //   camera_transformed_pose_pub_.publish(debug_camera_poses_msg);
      //   // std::cout << "[Viz] Published " << debug_camera_poses_msg.poses.size() << " camera poses in link00 frame." << std::endl;
      // }
    }
  });
}

void ArmController::initSubsAndPubs() {
  joint_state_msgs_.header.frame_id = "link00";
  joint_state_msgs_.position.assign(arm_joint_names_.size(), 0);
  joint_state_msgs_.effort.assign(arm_joint_names_.size(), 0);
  joint_state_msgs_.velocity.assign(arm_joint_names_.size(), 0);
  joint_state_msgs_.name = arm_joint_names_;
  cmd_joint_state_msgs_.header.frame_id = "link00";
  cmd_joint_state_msgs_.position.assign(arm_joint_names_.size(), 0);
  cmd_joint_state_msgs_.effort.assign(arm_joint_names_.size(), 0);
  cmd_joint_state_msgs_.velocity.assign(arm_joint_names_.size(), 0);
  cmd_joint_state_msgs_.name = arm_joint_names_;
  ee_pose_msg_.header.frame_id = "link00";
  arm_joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
  arm_cmd_joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("/cmd_joint_states", 1);
  ee_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/end_effector_pose", 1);
  process_pub_ = nh_.advertise<std_msgs::Float64>("/execute_process", 1);
  center_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/arm_controller/center_point", 1);
  target_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/target_poses", 1);
  poses_out1_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/poses_out1", 1);
  poses_out2_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/poses_out2", 1);
  poses_mid_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/poses_mid", 1);
  poses_half1_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/poses_half1", 1);
  poses_half2_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/poses_half2", 1);
  poses_mid_all_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/poses_mid_all", 1);
  transformed_input_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/arm_controller/transformed_input_pose", 1);
  camera_transformed_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/camera_transformed_pose", 1);
  // 初始化直线可视化发布器
  line_mid_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/line_mid", 1);
  line_out1_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/line_out1", 1);
  line_out2_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/line_out2", 1);
  line_half1_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/line_half1", 1);
  line_half2_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/line_half2", 1);
  reference_points_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/reference_points", 1);

  // 初始化交叉模式的直线可视化发布器
  cross_line_mid_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/cross_line_mid", 1);
  cross_line_left1_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/cross_line_left1", 1);
  cross_line_left2_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/cross_line_left2", 1);
  cross_line_top1_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/cross_line_top1", 1);
  cross_line_top2_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/cross_line_top2", 1);
  cross_reference_points_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/cross_reference_points", 1);
  cross_center_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/arm_controller/cross_center", 1);
  cross_rotation_transform_left_pub = nh_.advertise<geometry_msgs::PoseStamped>("/arm_controller/cross_rotation_transform_left", 1);
  cross_rotation_transform_top_pub = nh_.advertise<geometry_msgs::PoseStamped>("/arm_controller/cross_rotation_transform_top", 1);
  imu_sub_ = nh_.subscribe("/aliengo/imu", 1, &ArmController::imuCallback, this);
}

void ArmController::initServers() {
  plan_server_ = nh_.advertiseService("plan", &ArmController::planServer, this);
  back2home_server_ = nh_.advertiseService("back_to_home", &ArmController::back2HomeServer, this);
  plan_to_default_server_ = nh_.advertiseService("plan_to_default", &ArmController::planToDefaultServer, this);
  plan_to_horizon_server_ = nh_.advertiseService("plan_to_horizon", &ArmController::planToHorizonServer, this);
  plan_to_horizon_height_server_ = nh_.advertiseService("plan_to_horizon_height", &ArmController::planToHorizonHeightServer, this);
  check_pose_in_workspace_server_ = nh_.advertiseService("check_pose_in_workspace", &ArmController::isInWorkspaceServer, this);
  search_plan_server_ = nh_.advertiseService("search_plan", &ArmController::searchPlanServer, this);
  js_control_server_ = nh_.advertiseService("joy_stick_control", &ArmController::jsControlServer, this);
  plan_and_gripper_control_server_ = nh_.advertiseService("plan_and_gripper_control", &ArmController::planAndGripperControlServer, this);
  get_goal_and_angle_server_ = nh_.advertiseService("get_goal_and_angle", &ArmController::getGoalAndAngleServer, this);
  zed_link_to_link00_server_ = nh_.advertiseService("zed_link_to_link00", &ArmController::zedLinkToLink00Server, this);
  camera_to_link00_server_ = nh_.advertiseService("camera_to_link00", &ArmController::cameraToLink00Server, this);
  cross_get_goal_and_angle_server_ = nh_.advertiseService("cross_get_goal_and_angle", &ArmController::getCrossGoalAndAngleServer, this);
  // 订阅执行控制信号（从机械臂控制器获取状态）
  execute_process_sub_ = nh_.subscribe<std_msgs::Float64>("/arm_controller/execute_process", 1, &ArmController::executeProcessCallback, this);

  // plan_action_server_ = std::make_unique<
  //     actionlib::SimpleActionServer<arm_controller::PlanAction>>(
  //     nh_, "plan_action",
  //     boost::bind(&ArmController::planActionServer, this, _1), false);
}

void ArmController::publishStates() {
  ros::Time timestamp = ros::Time::now();
  joint_state_msgs_.header.stamp = timestamp;
  cmd_joint_state_msgs_.header.stamp = timestamp;
  ee_pose_msg_.header.stamp = timestamp;
  for (int i{0}; i < 6; ++i) {
    joint_state_msgs_.position[i] = low_state_.q[i];
    joint_state_msgs_.velocity[i] = low_state_.dq[i];
    joint_state_msgs_.effort[i] = low_state_.tau[i];
    cmd_joint_state_msgs_.position[i] = low_cmd_.q[i];
    cmd_joint_state_msgs_.velocity[i] = low_cmd_.dq[i];
    cmd_joint_state_msgs_.effort[i] = 25.6 * low_cmd_.kp[i] * (low_cmd_.q[i] - low_state_.q[i]) + 0.0128 * low_cmd_.kd[i] * (low_cmd_.dq[i] - low_state_.dq[i]) + low_cmd_.tau[i];
  }
  joint_state_msgs_.position[6] = low_state_.getGripperQ();
  joint_state_msgs_.velocity[6] = low_state_.getGripperQd();
  joint_state_msgs_.effort[6] = low_state_.getGripperTau();
  cmd_joint_state_msgs_.position[6] = low_state_.getGripperQ();
  cmd_joint_state_msgs_.velocity[6] = low_state_.getGripperQd();
  cmd_joint_state_msgs_.effort[6] = low_state_.getGripperTau();

  ee_pose_msg_.pose.position.x = low_state_.endPosture[3];
  ee_pose_msg_.pose.position.y = low_state_.endPosture[4];
  ee_pose_msg_.pose.position.z = low_state_.endPosture[5];
  Eigen::Vector4d ee_quat;
  rpyToQuat(low_state_.endPosture.head<3>(), ee_quat);
  ee_pose_msg_.pose.orientation.w = ee_quat[0];
  ee_pose_msg_.pose.orientation.x = ee_quat[1];
  ee_pose_msg_.pose.orientation.y = ee_quat[2];
  ee_pose_msg_.pose.orientation.z = ee_quat[3];
  process_msgs_.data = process_;
  arm_joint_states_pub_.publish(joint_state_msgs_);
  arm_cmd_joint_states_pub_.publish(cmd_joint_state_msgs_);
  ee_pose_pub_.publish(ee_pose_msg_);
  process_pub_.publish(process_msgs_);
}

void ArmController::controlStep() {
  checkArmMotorSafe();
  // std::cout << "arm control fsm: "
  //           << static_cast<typename
  //           std::underlying_type<ArmControlFsm>::type>(
  //                  arm_control_fsm_)
  //           << std::endl;
  // std::cout << arm_model_->_gravity.transpose() << std::endl;
  switch (arm_control_fsm_) {
    case ArmControlFsm::Invalid: {
      setControlCmd(0, 300.0, false);
      break;
    }
    case ArmControlFsm::Home: {
      arm_control_joint_pos_.setZero();
      arm_control_joint_vel_.setZero();
      // ee_pose_goal_ = arm_model_->forwardKinematics(low_state_.getQ());
      // arm_joint_goal_ = low_state_.getQ();
      setControlCmd(3, 400.0, true);
      break;
    }
    case ArmControlFsm::Back2Home: {
      if (arm_control_tick_ == joint_pos_trajectory_.size()) {
        setArmControlFsm(ArmControlFsm::Home);
        arm_control_joint_vel_.setZero();
        process_ = 1.0;
      } else {
        arm_control_joint_pos_ = joint_pos_trajectory_[arm_control_tick_];
        arm_control_joint_vel_ = joint_vel_trajectory_[arm_control_tick_];
        process_ = static_cast<double>(arm_control_tick_) / plan_max_tick_;
      }
      setControlCmd(default_kp_, default_kd_);
      // 控制夹爪到目标位置
      data_mutex_.lock();
      low_cmd_.setGripperQ(gripper_goal_);
      low_cmd_.setGripperQd(0.0);
      data_mutex_.unlock();
      break;
    }
    case ArmControlFsm::Arrived: {
      arm_control_joint_vel_.setZero();
      setControlCmd(default_kp_, default_kd_);
      // 保持夹爪位置
      data_mutex_.lock();
      low_cmd_.setGripperQ(gripper_goal_);
      low_cmd_.setGripperQd(0.0);
      // low_cmd_.setGripperGain(15.0, 0.0);  // 设置夹爪增益
      data_mutex_.unlock();
      break;
    }
    case ArmControlFsm::PlanMove: {
      if (arm_control_tick_ == joint_pos_trajectory_.size()) {
        setArmControlFsm(ArmControlFsm::Arrived);
        arm_control_joint_vel_.setZero();
        process_ = 1.0;
      } else {
        // 设置机械臂关节目标
        arm_control_joint_pos_ = joint_pos_trajectory_[arm_control_tick_];
        arm_control_joint_vel_ = joint_vel_trajectory_[arm_control_tick_];
        process_ = static_cast<double>(arm_control_tick_) / plan_max_tick_;
      }
      setControlCmd(default_kp_, default_kd_);
      // 保持夹爪位置
      data_mutex_.lock();
      low_cmd_.setGripperQ(gripper_goal_);
      low_cmd_.setGripperQd(0.0);
      // low_cmd_.setGripperGain(15.0, 0.0);  // 设置夹爪增益
      data_mutex_.unlock();
      break;
    }
    case ArmControlFsm::JoyStickControl: {
      updateArmCmdByJs();
      break;
    }
    default:
      break;
  }
  ++arm_control_tick_;
}

void ArmController::setArmControlFsm(ArmControlFsm control_fsm) {
  if (arm_control_fsm_ != control_fsm) {
    arm_control_fsm_ = control_fsm;
    arm_control_tick_ = 0;
  }
}

void ArmController::updateArmCmdByJs() {
  js_api_->getState(js_state_);
  bool find_ik{false}, singular_pose{false};
  double rot_twist_scaling = 0.2, linear_twist_scaling = 0.1;
  Eigen::Matrix4d current_pose = arm_model_->forwardKinematics(low_state_.getQ());
  Eigen::Matrix4d target_pose = ee_pose_goal_;
  Eigen::Matrix<double, 6, 1> target_joint_pos;
  Eigen::Matrix<double, 6, 1> twist_E = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Vector3d delta_rpy_E = Eigen::Vector3d::Zero(), delta_pos_E = Eigen::Vector3d::Zero();
  // angular part: rpy
  twist_E[0] = 0.0;
  twist_E[1] = js_state_.rasY();
  twist_E[2] = -js_state_.rasX();
  // linear part
  twist_E[3] = -js_state_.lasY();
  twist_E[4] = -js_state_.lasX();
  if (js_state_.Up().pressed) {
    twist_E[5] = 1.0;
  } else if (js_state_.Down().pressed) {
    twist_E[5] = -1.0;
  }
  twist_E.head<3>() *= rot_twist_scaling;
  twist_E.tail<3>() *= linear_twist_scaling;
  delta_rpy_E = twist_E.head<3>() * control_period_;
  delta_pos_E = twist_E.tail<3>() * control_period_;
  target_pose.block<3, 3>(0, 0) = ee_pose_goal_.block<3, 3>(0, 0) * rpyToRot(delta_rpy_E);
  target_pose.block<3, 1>(0, 3) += current_pose.block<3, 3>(0, 0) * delta_pos_E;
  find_ik = arm_model_->inverseKinematics(target_pose, low_state_.getQ(), target_joint_pos, true);
  singular_pose = find_ik ? arm_model_->checkInSingularity(target_joint_pos) : arm_model_->checkInSingularity(low_state_.getQ());
  // std::cout << "FindIK: " << find_ik << " Singularity: " << singular_pose
  //           << "\nTwistE: " << twist_E.transpose()
  //           << "\nDeltaRotE: " << delta_rpy_E.transpose()
  //           << "\nDeltaPosE: " << delta_pos_E.transpose() <<
  //           "\nTargetPose:\n"
  //           << target_pose
  //           << "\nTargetPosition: " << target_joint_pos.transpose()
  //           << std::endl;
  if (find_ik && !singular_pose) {
    ee_pose_goal_ = target_pose;
    arm_joint_goal_ = target_joint_pos;
    arm_control_joint_pos_ = arm_joint_goal_;
    arm_control_joint_vel_.setZero();
    arm_model_->solveQP(twist_E, low_state_.getQ(), arm_control_joint_vel_, control_period_);
  } else {
    arm_control_joint_vel_.setZero();
  }
  setControlCmd(default_kp_, default_kd_, true);
}

void ArmController::lazyPlan(const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& start, const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& goal, unsigned long ticks) {
  process_ = 0.0;
  joint_pos_trajectory_.clear();
  joint_vel_trajectory_.clear();
  joint_pos_trajectory_.push_back(start);
  joint_vel_trajectory_.push_back(Eigen::Matrix<double, 6, 1>::Zero());
  joint_interp_fn_.setPolyInterpolationKernel(ticks * control_period_, start, goal, ticks);
  for (long unsigned int i{1}; i < ticks; ++i) {
    joint_pos_trajectory_.push_back(joint_interp_fn_.step());
    joint_vel_trajectory_.push_back(joint_interp_fn_.d(i * control_period_));
  }
  joint_pos_trajectory_.push_back(goal);
  joint_vel_trajectory_.push_back(Eigen::Matrix<double, 6, 1>::Zero());
}

void ArmController::setControlCmd(double kp, double kd, bool enable_feedfoward_control) {
  for (int i{0}; i < 6; ++i) {
    low_cmd_.kp[i] = kp;
    low_cmd_.kd[i] = kd;
    low_cmd_.q[i] = arm_control_joint_pos_[i];
    low_cmd_.dq[i] = arm_control_joint_vel_[i];
  }
  // low_cmd_.setZeroDq();
  if (enable_feedfoward_control) {
    Eigen::Matrix<double, 6, 1> payload, tau_bias;
    payload << 0, 0, 0, 0, 0, 1.2;
    tau_bias = arm_model_->inverseDynamics(low_state_.getQ(), low_state_.getQd(), Eigen::Matrix<double, 6, 1>::Zero(), payload);
    // std::cout << tau_bias.transpose() << std::endl;
    low_cmd_.setTau(tau_bias);
  } else {
    low_cmd_.setZeroTau();
  }
}

void ArmController::setControlCmd(std::vector<double> kp, std::vector<double> kd, bool enable_feedfoward_control) {
  for (int i{0}; i < 6; ++i) {
    low_cmd_.kp[i] = kp[i];
    low_cmd_.kd[i] = kd[i];
    low_cmd_.q[i] = arm_control_joint_pos_[i];
    low_cmd_.dq[i] = arm_control_joint_vel_[i];
  }
  // low_cmd_.setZeroDq();
  if (enable_feedfoward_control) {
    Eigen::Matrix<double, 6, 1> payload, tau_bias;
    payload << 0, 0, 0, 0, 0, 1.2;
    tau_bias = arm_model_->inverseDynamics(arm_control_joint_pos_, arm_control_joint_vel_, Eigen::Matrix<double, 6, 1>::Zero(), payload);
    low_cmd_.setTau(tau_bias);
  } else {
    low_cmd_.setZeroTau();
  }
}

void ArmController::checkArmMotorSafe() {
  for (long unsigned int i = 0; i < low_state_.errorstate.size(); ++i) {
    uint8_t arm_state = low_state_.errorstate[i];
    if (arm_state == 0x01 || arm_state == 0x02 || arm_state == 0x04 || arm_state == 0x20) {
      arm_motor_safe_ = false;
      std::cout << "Arm motor is not safe! Set arm invalid!" << std::endl;
      setArmControlFsm(ArmControlFsm::Invalid);
    }
  }
}

bool ArmController::isInWorkspaceServer(arm_controller_srvs::CheckPoseInWorkspace::Request& req, arm_controller_srvs::CheckPoseInWorkspace::Response& res) {
  Eigen::Matrix4d target_pose, camera_target_pose;
  Eigen::Matrix<double, 6, 1> target_joint_pos;
  arm_controller::geometryMsgsPose2Pose(req.target_pose, camera_target_pose);
  target_pose = camera_target_pose;
  // target_pose.block<3, 1>(0, 3) = camera_target_pose.block<3, 1>(0, 3) - camera_target_pose.block<3, 3>(0, 0) * kCameraPosBias_E_;
  target_pose.block<3, 1>(0, 3) += Z1Arm_PosBias_E_;
  res.is_in_workspace = arm_model_->inverseKinematics(target_pose, Eigen::Matrix<double, 6, 1>::Zero(), target_joint_pos, true);
  return true;
}

bool ArmController::planServer(arm_controller_srvs::Plan::Request& req, arm_controller_srvs::Plan::Response& res) {
  res.call_success = false;
  // // 保存夹爪目标值
  // double gripper_goal = req.gripper_pos;
  if (arm_control_fsm_ == ArmControlFsm::Home || arm_control_fsm_ == ArmControlFsm::Arrived) {
    Eigen::Matrix4d start_ee_pose = arm_model_->forwardKinematics(low_state_.getQ());
    Eigen::Matrix<double, 6, 1> start_joint_pos = low_state_.getQ();
    Eigen::Matrix4d camera_target_pose, target_pose;
    Eigen::Matrix<double, 6, 1> target_joint_pos;
    bool find_ik{false};
    arm_controller::geometryMsgsPose2Pose(req.target_pose, camera_target_pose);
    target_pose = camera_target_pose;
    // target_pose.block<3, 1>(0, 3) =
    //     camera_target_pose.block<3, 1>(0, 3) -
    //     camera_target_pose.block<3, 3>(0, 0) * kCameraPosBias_E_;
    target_pose.block<3, 1>(0, 3) += Z1Arm_PosBias_E_;
    find_ik = arm_model_->inverseKinematics(target_pose, start_joint_pos, target_joint_pos, true);
    // std::cout << "StartEEPose: \n"
    //           << start_ee_pose << "\nGoalEEPose: \n"
    //           << target_pose
    //           << "\nStartJointPos: " << start_joint_pos.transpose()
    //           << "\nEndJointPos: " << target_joint_pos.transpose()
    //           << "\nFindIk: " << find_ik << std::endl;
    // std::cout << "JointMin: ";
    // for (int i{0}; i < 6; ++i) {
    //   std::cout << arm_model_->_jointQMin[i] << ", ";
    // }
    // std::cout << std::endl;
    // std::cout << "JointMax: ";
    // for (int i{0}; i < 6; ++i) {
    //   std::cout << arm_model_->_jointQMax[i] << ", ";
    // }
    // std::cout << std::endl;
    if (arm_motor_safe_ && find_ik) {
      if ((target_joint_pos - start_joint_pos).norm() <= 0.042) {
        res.call_success = true;
        return true;
      }
      ee_pose_goal_ = target_pose;
      arm_joint_goal_ = target_joint_pos;
      plan_max_tick_ = static_cast<long unsigned int>((ee_pose_goal_ - start_ee_pose).block<3, 1>(0, 3).norm() / average_move_speed_ / control_period_);
      plan_max_tick_ = std::max(100uL, plan_max_tick_);
      lazyPlan(start_joint_pos, arm_joint_goal_, plan_max_tick_);

      // // 同时规划夹爪轨迹（从当前位置到目标位置）
      // double gripper_current = low_state_.getGripperQ();
      // // 可以用线性插值或者直接设置目标值
      // gripper_goal_ = gripper_goal;

      setArmControlFsm(ArmControlFsm::PlanMove);
      res.call_success = true;
    }
  }
  return true;
}

bool ArmController::searchPlanServer(arm_controller_srvs::Plan::Request& req, arm_controller_srvs::Plan::Response& res) {
  res.call_success = false;
  // // 保存夹爪目标值
  // double gripper_goal = req.gripper_pos;
  if (arm_control_fsm_ == ArmControlFsm::Home || arm_control_fsm_ == ArmControlFsm::Arrived) {
    Eigen::Matrix4d start_ee_pose = arm_model_->forwardKinematics(low_state_.getQ());
    Eigen::Matrix<double, 6, 1> start_joint_pos = low_state_.getQ();
    Eigen::Matrix<double, 6, 1> target_joint_pos;
    bool find_ik{false};
    Eigen::Matrix4d camera_target_pose, search_pose, target_pose;
    arm_controller::geometryMsgsPose2Pose(req.target_pose, camera_target_pose);
    int max_search_num{30};
    Eigen::Vector3d search_start_pos_T{0.1, 0, 0}, search_interval{0.01, 0, 0};
    for (int i{1}; i <= max_search_num; ++i) {
      search_pose.setIdentity();
      search_pose.block<3, 1>(0, 3) = camera_target_pose.block<3, 3>(0, 0) * (search_start_pos_T + search_interval * i) + camera_target_pose.block<3, 1>(0, 3);
      search_pose.block<3, 1>(0, 0) = -camera_target_pose.block<3, 1>(0, 0);
      search_pose.block<3, 1>(0, 1) = -camera_target_pose.block<3, 1>(0, 1);
      target_pose = search_pose;
      target_pose.block<3, 1>(0, 3) = search_pose.block<3, 1>(0, 3) - search_pose.block<3, 3>(0, 0) * kCameraPosBias_E_;
      target_pose.block<3, 1>(0, 3) += Z1Arm_PosBias_E_;
      find_ik = arm_model_->inverseKinematics(target_pose, start_joint_pos, target_joint_pos, true);
      // std::cout << "StartEEPose: \n"
      //           << start_ee_pose << "\nGoalEEPose:\n"
      //           << target_pose << "\nSearchCameraPose:\n"
      //           << search_pose << "\nCameraTargetPose:\n"
      //           << camera_target_pose
      //           << "\nStartJointPos: " << start_joint_pos.transpose()
      //           << "\nEndJointPos: " << target_joint_pos.transpose()
      //           << "\nFindIk: " << find_ik << std::endl;
      if (find_ik) {
        break;
      }
    }
    if (arm_motor_safe_ && find_ik) {
      if ((target_joint_pos - start_joint_pos).norm() <= 0.042) {
        res.call_success = true;
        return true;
      }
      ee_pose_goal_ = target_pose;
      arm_joint_goal_ = target_joint_pos;
      plan_max_tick_ = static_cast<long unsigned int>((ee_pose_goal_ - start_ee_pose).block<3, 1>(0, 3).norm() / average_move_speed_ / control_period_);
      plan_max_tick_ = std::max(100uL, plan_max_tick_);
      lazyPlan(start_joint_pos, arm_joint_goal_, plan_max_tick_);
      setArmControlFsm(ArmControlFsm::PlanMove);

      // // 同时规划夹爪轨迹（从当前位置到目标位置）
      // double gripper_current = low_state_.getGripperQ();
      // // 可以用线性插值或者直接设置目标值
      // gripper_goal_ = gripper_goal;

      res.call_success = true;
    }
  }
  return true;
}

bool ArmController::IsPlanServer(arm_controller_srvs::Plan::Request& req, arm_controller_srvs::Plan::Response& res) {
  res.call_success = false;

  geometry_msgs::Pose probe_pose = req.target_pose;

  // IsPlan 内部已经包含了 pinocchio_ik_ 的初始化检查和 FSM 状态检查
  bool plan_feasible = IsPlan(probe_pose);

  if (plan_feasible && arm_motor_safe_) {
    res.call_success = true;

    // 打印调试信息：对比一下“你想去的”和“实际能去的”
    // 因为我们忽略了 Roll，probe_pose 里的四元数可能和 req 里的不一样
    // ROS_INFO_THROTTLE(1.0, "[IsPlanServer] Check OK. Request: (%.2f, %.2f, %.2f) -> Feasible: (%.2f, %.2f, %.2f)",
    //   req.target_pose.position.x, req.target_pose.position.y, req.target_pose.position.z,
    //   probe_pose.position.x, probe_pose.position.y, probe_pose.position.z);
  } else {
    res.call_success = false;
  }

  return true;
}

bool ArmController::IsPlan(geometry_msgs::Pose& target_pose) {
  // 1. 基础检查
  if (!pinocchio_ik_) {
    ROS_ERROR("[IsPlan] PinocchioIK is NOT initialized!");
    return false;
  }

  // 状态检查：只允许在静止或空闲状态下查询
  if (arm_control_fsm_ != ArmControlFsm::Home && arm_control_fsm_ != ArmControlFsm::Arrived) {
    // ROS_WARN("[IsPlan] Rejected. Arm is moving.");
    // 可根据需求决定是否允许运动中查询，通常建议允许，所以这里只做警告或直接通过
  }

  // 2. 四元数归一化 (防止非法输入)
  Eigen::Quaterniond q_check(target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);
  if (std::abs(q_check.norm() - 1.0) > 1e-3) q_check.normalize();

  // 3. 准备 7 维状态
  // 即使只是查询，也需要当前的关节角作为 IK 的“种子(Seed)”，这样算出来的解离当前位置最近
  Eigen::VectorXd start_state_7d = Eigen::VectorXd::Zero(7);
  start_state_7d.head<6>() = low_state_.getQ();
  start_state_7d[6] = low_state_.getGripperQ();

  Eigen::VectorXd target_state_7d = Eigen::VectorXd::Zero(7);

  // 4. 准备目标位姿矩阵
  Eigen::Matrix4d target_pose_eigen;
  arm_controller::geometryMsgsPose2Pose(target_pose, target_pose_eigen);
  target_pose_eigen.block<3, 3>(0, 0) = q_check.toRotationMatrix();

  // 【重要】移除手动偏移
  // 如果 pinocchio_ik_ 初始化时用的是 "camera_optical_frame"，则不需要加 Bias
  // target_pose_eigen.block<3, 1>(0, 3) += Pinocchio_PosBias_E_; // DELETE THIS

  // 5. 设置 IK 权重 (保持与 planToTargetPose 一致)
  // Look-At 模式：忽略 Roll (Local Z 轴旋转)，大幅提高可达性
  Eigen::Matrix<double, 6, 1> weights;
  weights << 1, 1, 1, 0, 1, 1;

  // 6. 执行 IK 求解
  // 这是一个纯数学计算，不会控制机械臂运动
  bool find_ik = pinocchio_ik_->inverseKinematics(target_pose_eigen, start_state_7d, target_state_7d, weights);

  // 7. 结果处理
  if (arm_motor_safe_ && find_ik) {
    // =========================== 关键步骤 ===========================
    // IK 算出来了，但因为我们忽略了 Roll，或者有微小误差
    // 我们需要知道：如果机械臂走到这个角度，【实际】会在哪里？

    Eigen::Matrix4d actual_pose_eigen;
    // 使用算出来的 7 维角度计算正运动学 (FK)
    pinocchio_ik_->forwardKinematics(target_state_7d, actual_pose_eigen);

    // 将实际位姿回写给 target_pose (引用传递)
    // 这样外部调用者就知道：“哦，我让你去 A，你实际上只能去 A'”
    Eigen::Vector3d trans = actual_pose_eigen.block<3, 1>(0, 3);
    Eigen::Matrix3d rot = actual_pose_eigen.block<3, 3>(0, 0);
    Eigen::Quaterniond q_actual(rot);
    q_actual.normalize();

    target_pose.position.x = trans.x();
    target_pose.position.y = trans.y();
    target_pose.position.z = trans.z();
    target_pose.orientation.x = q_actual.x();
    target_pose.orientation.y = q_actual.y();
    target_pose.orientation.z = q_actual.z();
    target_pose.orientation.w = q_actual.w();

    return true;
  }

  return false;
}

bool ArmController::back2HomeServer(arm_controller_srvs::BackToHome::Request& req, arm_controller_srvs::BackToHome::Response& res) {
  res.call_success = false;
  Eigen::Matrix4d start_ee_pose = arm_model_->forwardKinematics(low_state_.getQ());
  Eigen::Matrix<double, 6, 1> start_joint_pos = low_state_.getQ();
  if (arm_motor_safe_) {
    ee_pose_goal_ = kEePoseHome_;
    arm_joint_goal_ = KJointHome_;
    plan_max_tick_ = static_cast<long unsigned int>((kEePoseHome_ - start_ee_pose).block<3, 1>(0, 3).norm() / average_move_speed_ / control_period_);
    plan_max_tick_ = std::max(100uL, plan_max_tick_);
    // std::cout << "StartEEPose:\n"
    //           << start_ee_pose
    //           << "\nStartJointPos: " << start_joint_pos.transpose()
    //           << "\nHomePose:\n"
    //           << kEePoseHome_ << "\nKJointHome: " << KJointHome_
    //           << "\nPlanTicks: " << plan_max_tick_ << std::endl;
    lazyPlan(start_joint_pos, KJointHome_, plan_max_tick_);
    // 设置夹爪复位到 Home 位置（完全打开）
    gripper_goal_ = 0.0;  // 夹爪 Home 位置（与 planToDefaultServer 保持一致）
    setArmControlFsm(ArmControlFsm::Back2Home);
    // res.call_success = true;
  }
  // 等待机械臂执行到位
  ros::Rate rate(1.0 / control_period_);
  double timeout = (plan_max_tick_ * control_period_) + 5.0;  // 预计时间 + 5秒超时
  ros::Time start_time = ros::Time::now();
  while (ros::ok()) {
    // 检查是否超时
    if ((ros::Time::now() - start_time).toSec() > timeout) {
      ROS_WARN("[Back2Home] Timeout waiting for arm to reach target position");
      res.call_success = false;
      return false;
    }

    // 检查是否到位
    if (arm_control_fsm_ == ArmControlFsm::Home) {
      ROS_INFO("[Back2Home] Arm reached target position and stabilized");
      res.call_success = true;
      return true;
    }

    ros::spinOnce();
    rate.sleep();
  }
  return true;
}

bool ArmController::planToDefaultServer(arm_controller_srvs::PlanToDefault::Request& req, arm_controller_srvs::PlanToDefault::Response& res) {
  res.call_success = false;
  // // 保存夹爪目标值
  // double gripper_goal = req.gripper_pos;
  if (arm_control_fsm_ == ArmControlFsm::Home || arm_control_fsm_ == ArmControlFsm::Arrived) {
    Eigen::Matrix4d start_ee_pose = arm_model_->forwardKinematics(low_state_.getQ());
    Eigen::Matrix<double, 6, 1> start_joint_pos = low_state_.getQ();
    // Eigen::Matrix4d camera_target_pose, target_pose;
    Eigen::Matrix<double, 6, 1> target_joint_pos;
    bool find_ik{false};
    target_joint_pos = arm_control_default_joint_pos_;
    if (arm_motor_safe_) {
      if ((target_joint_pos - start_joint_pos).norm() <= 0.042) {
        res.call_success = true;
        return true;
      }
      ee_pose_goal_ = arm_model_->forwardKinematics(arm_control_default_joint_pos_);
      arm_joint_goal_ = target_joint_pos;
      plan_max_tick_ = static_cast<long unsigned int>((ee_pose_goal_ - start_ee_pose).block<3, 1>(0, 3).norm() / average_move_speed_ / control_period_);
      plan_max_tick_ = std::max(100uL, plan_max_tick_);
      lazyPlan(start_joint_pos, arm_joint_goal_, plan_max_tick_);

      // 同时规划夹爪轨迹（从当前位置到目标位置）
      double gripper_goal = 0.0;
      double gripper_current = low_state_.getGripperQ();
      // 可以用线性插值或者直接设置目标值
      gripper_goal_ = gripper_goal;

      setArmControlFsm(ArmControlFsm::PlanMove);
      res.call_success = true;
    }
  }

  // 等待机械臂执行到位
  ros::Rate rate(1.0 / control_period_);
  double timeout = (plan_max_tick_ * control_period_) + 5.0;  // 预计时间 + 5秒超时
  ros::Time start_time = ros::Time::now();
  while (ros::ok()) {
    // 检查是否超时
    if ((ros::Time::now() - start_time).toSec() > timeout) {
      ROS_WARN("[PlanToDefault] Timeout waiting for arm to reach target position");
      res.call_success = false;
      return false;
    }

    // 检查是否到位
    if (arm_control_fsm_ == ArmControlFsm::Arrived) {
      ROS_INFO("[PlanToDefault] Arm reached target position and stabilized");
      res.call_success = true;
      return true;
    }

    ros::spinOnce();
    rate.sleep();
  }
  return true;
}

bool ArmController::planToHorizonServer(arm_controller_srvs::PlanToHorizon::Request& req, arm_controller_srvs::PlanToHorizon::Response& res) {
  res.call_success = false;
  // // 保存夹爪目标值
  // double gripper_goal = req.gripper_pos;
  if (arm_control_fsm_ == ArmControlFsm::Home || arm_control_fsm_ == ArmControlFsm::Arrived) {
    Eigen::Matrix4d start_ee_pose = arm_model_->forwardKinematics(low_state_.getQ());
    Eigen::Matrix<double, 6, 1> start_joint_pos = low_state_.getQ();
    // Eigen::Matrix4d camera_target_pose, target_pose;
    Eigen::Matrix<double, 6, 1> target_joint_pos;
    bool find_ik{false};
    target_joint_pos = arm_control_horizon_joint_pos_;
    if (arm_motor_safe_) {
      if ((target_joint_pos - start_joint_pos).norm() <= 0.042) {
        res.call_success = true;
        return true;
      }
      ee_pose_goal_ = arm_model_->forwardKinematics(arm_control_horizon_joint_pos_);
      arm_joint_goal_ = target_joint_pos;
      plan_max_tick_ = static_cast<long unsigned int>((ee_pose_goal_ - start_ee_pose).block<3, 1>(0, 3).norm() / average_move_speed_ / control_period_);
      plan_max_tick_ = std::max(100uL, plan_max_tick_);
      lazyPlan(start_joint_pos, arm_joint_goal_, plan_max_tick_);

      // 同时规划夹爪轨迹（从当前位置到目标位置）
      double gripper_goal = 0.0;
      double gripper_current = low_state_.getGripperQ();
      // 可以用线性插值或者直接设置目标值
      gripper_goal_ = gripper_goal;

      setArmControlFsm(ArmControlFsm::PlanMove);
      res.call_success = true;
    }
  }

  // 等待机械臂执行到位
  ros::Rate rate(1.0 / control_period_);
  double timeout = (plan_max_tick_ * control_period_) + 5.0;  // 预计时间 + 5秒超时
  ros::Time start_time = ros::Time::now();
  while (ros::ok()) {
    // 检查是否超时
    if ((ros::Time::now() - start_time).toSec() > timeout) {
      ROS_WARN("[PlanToDefault] Timeout waiting for arm to reach target position");
      res.call_success = false;
      return false;
    }

    // 检查是否到位
    if (arm_control_fsm_ == ArmControlFsm::Arrived) {
      ROS_INFO("[PlanToDefault] Arm reached target position and stabilized");
      res.call_success = true;
      return true;
    }

    ros::spinOnce();
    rate.sleep();
  }
  return true;
}

bool ArmController::planToHorizonHeightServer(arm_controller_srvs::PlanToHorizonHeight::Request& req, arm_controller_srvs::PlanToHorizonHeight::Response& res) {
  res.call_success = false;
  
  if (arm_control_fsm_ == ArmControlFsm::Home || arm_control_fsm_ == ArmControlFsm::Arrived) {
    Eigen::Matrix4d start_ee_pose = arm_model_->forwardKinematics(low_state_.getQ());
    Eigen::Matrix<double, 6, 1> start_joint_pos = low_state_.getQ();
    Eigen::Matrix<double, 6, 1> target_joint_pos;
    
    // 如果传入了高度参数且不为0，则使用传入的高度调整joint_pos_3
    // 否则使用默认的 arm_control_horizon_height_joint_pos_
    if (req.height != 0.0) {
      // 使用传入的高度参数来调整关节位置
      // 这里假设高度主要由 joint_pos_2 和 joint_pos_3 控制
      // 你可以根据实际机械臂的运动学特性调整这个映射关系
      target_joint_pos = arm_control_horizon_height_joint_pos_;
      // 示例：简单地将高度映射到 joint_pos_3
      // 你可能需要根据实际的运动学关系调整这个公式
      target_joint_pos[3] = arm_control_horizon_height_joint_pos_[3] + req.height;
      ROS_INFO("[PlanToHorizonHeight] Using custom height: %f, adjusted joint3: %f", req.height, target_joint_pos[3]);
    } else {
      // 使用默认配置
      target_joint_pos = arm_control_horizon_height_joint_pos_;
      ROS_INFO("[PlanToHorizonHeight] Using default horizon height position");
    }
    
    if (arm_motor_safe_) {
      // 检查是否已经很接近目标位置
      if ((target_joint_pos - start_joint_pos).norm() <= 0.042) {
        ROS_INFO("[PlanToHorizonHeight] Already at target position");
        res.call_success = true;
        return true;
      }
      
      ee_pose_goal_ = arm_model_->forwardKinematics(target_joint_pos);
      arm_joint_goal_ = target_joint_pos;
      plan_max_tick_ = static_cast<long unsigned int>((ee_pose_goal_ - start_ee_pose).block<3, 1>(0, 3).norm() / average_move_speed_ / control_period_);
      plan_max_tick_ = std::max(100uL, plan_max_tick_);
      lazyPlan(start_joint_pos, arm_joint_goal_, plan_max_tick_);

      // 同时规划夹爪轨迹（保持当前位置或设为0）
      double gripper_goal = 0.0;
      gripper_goal_ = gripper_goal;

      setArmControlFsm(ArmControlFsm::PlanMove);
      res.call_success = true;
      
      ROS_INFO("[PlanToHorizonHeight] Motion planned, max_tick: %lu", plan_max_tick_);
    } else {
      ROS_WARN("[PlanToHorizonHeight] Arm motor not safe, cannot execute motion");
    }
  } else {
    ROS_WARN("[PlanToHorizonHeight] Arm not in Home or Arrived state, current state: %d", static_cast<int>(arm_control_fsm_));
  }

  // 等待机械臂执行到位
  ros::Rate rate(1.0 / control_period_);
  double timeout = (plan_max_tick_ * control_period_) + 5.0;  // 预计时间 + 5秒超时
  ros::Time start_time = ros::Time::now();
  while (ros::ok()) {
    // 检查是否超时
    if ((ros::Time::now() - start_time).toSec() > timeout) {
      ROS_WARN("[PlanToHorizonHeight] Timeout waiting for arm to reach target position");
      res.call_success = false;
      return false;
    }

    // 检查是否到位
    if (arm_control_fsm_ == ArmControlFsm::Arrived) {
      ROS_INFO("[PlanToHorizonHeight] Arm reached target position and stabilized");
      res.call_success = true;
      return true;
    }

    ros::spinOnce();
    rate.sleep();
  }
  return true;
}

bool ArmController::jsControlServer(arm_controller_srvs::JoyStickControlRequest& req, arm_controller_srvs::JoyStickControlResponse& res) {
  res.call_success = false;
  if (req.enable && arm_control_fsm_ == ArmControlFsm::Arrived && (!arm_model_->checkInSingularity(low_state_.getQ()))) {
    setArmControlFsm(ArmControlFsm::JoyStickControl);
    res.call_success = true;
  } else if (!req.enable && arm_control_fsm_ == ArmControlFsm::JoyStickControl) {
    arm_controller_srvs::BackToHomeRequest reset_req;
    arm_controller_srvs::BackToHomeResponse reset_res;
    reset_req.back_to_home = true;
    back2HomeServer(reset_req, reset_res);
    res.call_success = reset_res.call_success;
  }
  return true;
}

bool ArmController::cameraToLink00Server(arm_controller_srvs::CameraToLink00::Request& req, arm_controller_srvs::CameraToLink00::Response& res) {
  try {
    // 1. 尝试获取相机坐标系到 link00 的变换
    geometry_msgs::TransformStamped transform_stamped;
    bool has_camera_frame = false;

    try {
      transform_stamped = tf_buffer_.lookupTransform("link00", "camera_link", ros::Time(0), ros::Duration(0.5));
      has_camera_frame = true;
      ROS_INFO("[CameraToLink00] Found camera frame in TF tree");
    } catch (tf2::TransformException& ex) {
      ROS_WARN(
          "[CameraToLink00] No camera frame found: %s. Using link00 frame "
          "directly.",
          ex.what());
      has_camera_frame = false;
    }

    // 2. 根据是否有 camera 坐标系来处理
    geometry_msgs::PoseStamped result_pose_stamped;
    result_pose_stamped.header.frame_id = "link00";
    result_pose_stamped.header.stamp = ros::Time::now();

    if (has_camera_frame) {
      // 有 camera 坐标系:创建相机坐标系下的偏移位姿并转换
      geometry_msgs::PoseStamped camera_offset_pose;
      camera_offset_pose.header.frame_id = "camera_link";
      camera_offset_pose.header.stamp = ros::Time::now();
      camera_offset_pose.pose.position.x = req.dx;
      camera_offset_pose.pose.position.y = req.dy;
      camera_offset_pose.pose.position.z = 0.0;  // 假设偏移在相机平面上
      camera_offset_pose.pose.orientation.w = 1.0;
      camera_offset_pose.pose.orientation.x = 0.0;
      camera_offset_pose.pose.orientation.y = 0.0;
      camera_offset_pose.pose.orientation.z = 0.0;

      // 将相机坐标系下的偏移位姿转换到 link00 坐标系
      geometry_msgs::PoseStamped link00_pose;
      tf2::doTransform(camera_offset_pose, link00_pose, transform_stamped);
      res.target_pose = link00_pose.pose;
      res.success = true;  // 转换成功

      // 设置发布的位姿
      result_pose_stamped.pose = link00_pose.pose;

      ROS_INFO(
          "[CameraToLink00] Offset (dx=%.3f, dy=%.3f) in camera frame -> "
          "Position (%.3f, %.3f, %.3f) in link00 frame (with TF transform)",
          req.dx, req.dy, res.target_pose.position.x, res.target_pose.position.y, res.target_pose.position.z);
    } else {
      // 没有 camera 坐标系:直接在 link00 坐标系下应用偏移
      res.target_pose.position.x = req.dx;
      res.target_pose.position.y = req.dy;
      res.target_pose.position.z = 0.0;
      res.target_pose.orientation.w = 1.0;
      res.target_pose.orientation.x = 0.0;
      res.target_pose.orientation.y = 0.0;
      res.target_pose.orientation.z = 0.0;
      res.success = false;  // 未使用 TF 转换

      // 设置发布的位姿
      result_pose_stamped.pose = res.target_pose;

      ROS_WARN(
          "[CameraToLink00] No camera frame. Using offset (dx=%.3f, "
          "dy=%.3f) directly in link00 frame (no TF transform)",
          req.dx, req.dy);
    }

    // 3. 发布转换后的位姿到话题
    // camera_transformed_pose_pub_.publish(result_pose_stamped);
    // ROS_DEBUG(
    //     "[CameraToLink00] Published transformed pose to "
    //     "/arm_controller/camera_transformed_pose");

    return true;
  } catch (const std::exception& e) {
    ROS_ERROR("[CameraToLink00] Exception: %s", e.what());
    return false;
  }
}

bool ArmController::planAndGripperControlServer(arm_controller_srvs::planandgrippercontrol::Request& req, arm_controller_srvs::planandgrippercontrol::Response& res) {
  // res.call_success = false;

  // 从 ROS 参数服务器读取默认目标位姿
  geometry_msgs::Pose planandgrippercontrol_target_pose = req.target_pose;
  float pitch = req.gripper_pos;
  float roll = req.joint6_pos;

  bool success_flag = executeMotionToTarget(planandgrippercontrol_target_pose, pitch, roll, 10);
  res.call_success = success_flag;
  return true;
}

bool ArmController::getGoalAndAngleServer(arm_controller_srvs::getgoalandangle::Request& req, arm_controller_srvs::getgoalandangle::Response& res) {
  ROS_INFO("[GetGoalAndAngle] Service called, computing target poses...");
  res.call_success = false;

  // 清空输出
  res.target_poses.clear();
  res.pitch_angles.clear();
  res.roll_angles.clear();
  res.pose_names.clear();

  // 读取参数
  double mid_sample_start, mid_sample_end, sample_start, sample_end;
  int mid_num_samples, num_samples;
  double angle_step_deg;
  double half_offset_distance = -0.14;
  double mid_offset_distance = -0.15;

  nh_.param("test/mid_sample_start", mid_sample_start, -1.0);
  nh_.param("test/mid_sample_end", mid_sample_end, 0.5);
  nh_.param("test/mid_num_samples", mid_num_samples, 50);
  nh_.param("test/angle_step_deg", angle_step_deg, 45.0);
  nh_.param("test/num_samples", num_samples, 50);
  nh_.param("test/sample_start", sample_start, -1.0);
  nh_.param("test/sample_end", sample_end, 0.0);
  // 发布输入位姿到 TF
  if (static_br_ptr_) {
    geometry_msgs::TransformStamped t_input_msg;

    t_input_msg.header.stamp = ros::Time::now();
    t_input_msg.header.frame_id = req.target_pose.header.frame_id;  // 父: "link00"
    t_input_msg.child_frame_id = "input_frame";
    t_input_msg.transform.translation.x = req.target_pose.pose.position.x;
    t_input_msg.transform.translation.y = req.target_pose.pose.position.y;
    t_input_msg.transform.translation.z = req.target_pose.pose.position.z;
    t_input_msg.transform.rotation = req.target_pose.pose.orientation;

    static_br_ptr_->sendTransform(t_input_msg);
  } else {
    ROS_ERROR("[GetGoalAndAngle] TF Broadcaster is not initialized!");
    return true;
  }
  double center_x, center_y, center_z;
  double center_roll, center_pitch, center_yaw;
  tf2::Quaternion quat_transformed;
  //==========================================================================
  // 步骤1：读取参数和解析输入位姿
  //==========================================================================
  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform("link00", "combine_frame", ros::Time(0), ros::Duration(1.0));

    center_x = transform_stamped.transform.translation.x;
    center_y = transform_stamped.transform.translation.y;
    center_z = transform_stamped.transform.translation.z;

    // 四元数变换
    tf2::fromMsg(transform_stamped.transform.rotation, quat_transformed);

  } catch (tf2::TransformException& ex) {
    ROS_WARN("Could not get transform: %s", ex.what());
    return true;
  }
  tf2::Matrix3x3 mat(quat_transformed);
  mat.getRPY(center_roll, center_pitch, center_yaw);

  // 生成三个方向向量
  tf2::Vector3 x_axis_in_combine_frame(1.0, 0.0, 0.0);
  tf2::Vector3 y_axis_in_combine_frame(0.0, 1.0, 0.0);
  tf2::Vector3 z_axis_in_combine_frame(0.0, 0.0, 1.0);
  tf2::Vector3 rotation_axis_combine_x_in_link00 = tf2::quatRotate(quat_transformed, x_axis_in_combine_frame);
  tf2::Vector3 rotation_axis_combine_y_in_link00 = tf2::quatRotate(quat_transformed, y_axis_in_combine_frame);
  tf2::Vector3 rotation_axis_combine_z_in_link00 = tf2::quatRotate(quat_transformed, z_axis_in_combine_frame);
  tf2::Vector3 origin_direction = rotation_axis_combine_x_in_link00.normalized();
  tf2::Vector3 translation_direction = rotation_axis_combine_x_in_link00.normalized();
  double line_x = center_x;
  double line_y = center_y;
  double line_z = center_z;

  //==========================================================================
  // 步骤2：生成5条直线
  //==========================================================================
  Line3D original_line;
  original_line.point = Eigen::Vector3d(center_x, center_y, center_z);
  original_line.direction = Eigen::Vector3d(origin_direction.x(), origin_direction.y(), origin_direction.z()).normalized();

  Eigen::Vector3d rotation_center(center_x, center_y, center_z);
  Eigen::Vector3d rotation_axis(rotation_axis_combine_z_in_link00.x(), rotation_axis_combine_z_in_link00.y(),
                                rotation_axis_combine_z_in_link00.z());  // 左右旋转轴
  Eigen::Vector3d translation_axis(rotation_axis_combine_y_in_link00.x(), rotation_axis_combine_y_in_link00.y(),
                                   rotation_axis_combine_y_in_link00.z());  // 平移轴
  Eigen::Vector3d reference_point_mid = original_line.point + mid_offset_distance * original_line.direction;

  Line3D line_mid = original_line;
  double angle_step_rad_1 = angle_step_deg * M_PI / 180.0;
  Line3D line_half1 = rotateLine(original_line, rotation_center, rotation_axis, angle_step_rad_1);
  double angle_step_rad_2 = (360.0 - angle_step_deg) * M_PI / 180.0;
  Line3D line_half2 = rotateLine(original_line, rotation_center, rotation_axis, angle_step_rad_2);
  double distance_1 = 0.3;
  Line3D line_out1 = translateLineGeometry(original_line, translation_axis, distance_1);
  double distance_2 = -0.3;
  Line3D line_out2 = translateLineGeometry(original_line, translation_axis, distance_2);

  //==========================================================================
  // 步骤3：计算参考点并检查顺序
  //==========================================================================
  Eigen::Vector3d mid_direction = line_mid.direction.normalized();
  Eigen::Vector3d half_direction_1 = line_half1.direction.normalized();
  Eigen::Vector3d half_direction_2 = line_half2.direction.normalized();
  double square_size = -0.05;
  Eigen::Vector3d reference_point_out1 = original_line.point + translation_axis * distance_1 + square_size * original_line.direction;
  Eigen::Vector3d reference_point_out2 = original_line.point + translation_axis * distance_2 + square_size * original_line.direction;
  Eigen::Vector3d reference_point_half1 = original_line.point + half_offset_distance * half_direction_1;
  Eigen::Vector3d reference_point_half2 = original_line.point + half_offset_distance * half_direction_2;

  // 检查并调整顺序
  Eigen::Vector3d vec_mid_to_out1 = reference_point_out1 - reference_point_mid;
  Eigen::Vector3d vec_mid_to_half1 = reference_point_half1 - reference_point_mid;
  Eigen::Vector3d vec_mid_to_half2 = reference_point_half2 - reference_point_mid;

  double dot_out1_half1 = vec_mid_to_out1.dot(vec_mid_to_half1);
  double dot_out1_half2 = vec_mid_to_out1.dot(vec_mid_to_half2);

  bool need_swap = (dot_out1_half2 > dot_out1_half1);
  if (need_swap) {
    std::swap(reference_point_half1, reference_point_half2);
    std::swap(half_direction_1, half_direction_2);
    std::swap(line_half1, line_half2);
  }

  // 创建以参考点为起点的直线
  Line3D line_mid_sampled;
  line_mid_sampled.point = reference_point_mid;
  line_mid_sampled.direction = mid_direction;

  Line3D line_half1_sampled;
  line_half1_sampled.point = reference_point_half1;
  line_half1_sampled.direction = half_direction_1;

  Line3D line_half2_sampled;
  line_half2_sampled.point = reference_point_half2;
  line_half2_sampled.direction = half_direction_2;

  Line3D line_out1_sampled;
  line_out1_sampled.point = reference_point_out1;
  line_out1_sampled.direction = mid_direction;

  Line3D line_out2_sampled;
  line_out2_sampled.point = reference_point_out2;
  line_out2_sampled.direction = mid_direction;

  //==========================================================================
  // 步骤4：采样并检测可达性
  //==========================================================================
  double pitch_0 = 0.0, roll_0 = 0.0;
  double pitch_1 = 0.0, roll_1 = 0.0;
  double pitch_2 = 0.0, roll_2 = 0.0;
  double pitch_half1 = 0.0, roll_half1 = 0.0;
  double pitch_half2 = 0.0, roll_half2 = 0.0;

  geometry_msgs::Pose debug_camera_poses_demo;
  debug_camera_poses_msg.header.frame_id = "gripperMover";  // gripperMover
  debug_camera_poses_msg.header.stamp = ros::Time::now();

  std::cout << "[1/5] Sampling MID line..." << std::endl;
  std::vector<geometry_msgs::PoseStamped> reachable_poses_mid = sampleAndCheckReachability(line_mid_sampled, mid_sample_start, mid_sample_end, mid_num_samples, pitch_0, roll_0, debug_camera_poses_demo);
  // debug_camera_poses_msg.poses.push_back(debug_camera_poses_demo);
  std::cout << "[2/5] Sampling HALF1 line..." << std::endl;
  std::vector<geometry_msgs::PoseStamped> reachable_poses_half1 = sampleAndCheckReachability(line_half1_sampled, sample_start, sample_end, num_samples, pitch_half1, roll_half1, debug_camera_poses_demo);
  // debug_camera_poses_msg.poses.push_back(debug_camera_poses_demo);
  std::cout << "[3/5] Sampling HALF2 line..." << std::endl;
  std::vector<geometry_msgs::PoseStamped> reachable_poses_half2 = sampleAndCheckReachability(line_half2_sampled, sample_start, sample_end, num_samples, pitch_half2, roll_half2, debug_camera_poses_demo);
  // debug_camera_poses_msg.poses.push_back(debug_camera_poses_demo);
  std::cout << "[4/5] Sampling OUT1 line..." << std::endl;
  std::vector<geometry_msgs::PoseStamped> reachable_poses_out1 = sampleAndCheckReachability(line_out1_sampled, sample_start, sample_end, num_samples, pitch_1, roll_1, debug_camera_poses_demo);
  // debug_camera_poses_msg.poses.push_back(debug_camera_poses_demo);
  std::cout << "[5/5] Sampling OUT2 line..." << std::endl;
  std::vector<geometry_msgs::PoseStamped> reachable_poses_out2 = sampleAndCheckReachability(line_out2_sampled, sample_start, sample_end, num_samples, pitch_2, roll_2, debug_camera_poses_demo);
  // debug_camera_poses_msg.poses.push_back(debug_camera_poses_demo);
  // if (!debug_camera_poses_msg.poses.empty()) {
  //   camera_transformed_pose_pub_.publish(debug_camera_poses_msg);
  //   std::cout << "[Viz] Published " << debug_camera_poses_msg.poses.size() << " camera poses in link00 frame." << std::endl;
  // }
  ROS_INFO(
      "[GetGoalAndAngle] Reachable poses: MID=%zu, HALF1=%zu, HALF2=%zu, "
      "OUT1=%zu, OUT2=%zu",
      reachable_poses_mid.size(), reachable_poses_half1.size(), reachable_poses_half2.size(), reachable_poses_out1.size(), reachable_poses_out2.size());

  // 排序
  double target_distance, mid_target_distance;
  nh_.param("test/target_distance", target_distance, 0.2);
  nh_.param("test/mid_target_distance", mid_target_distance, 0.2);

  reachable_poses_out1 = sortPosesByDistanceToPoint(reachable_poses_out1, reference_point_out1, mid_direction, target_distance);
  reachable_poses_out2 = sortPosesByDistanceToPoint(reachable_poses_out2, reference_point_out2, mid_direction, target_distance);
  reachable_poses_mid = sortPosesByDistanceToPoint(reachable_poses_mid, reference_point_mid, mid_direction, mid_target_distance);
  reachable_poses_half1 = sortPosesByDistanceToPoint(reachable_poses_half1, reference_point_half1, half_direction_1, target_distance);
  reachable_poses_half2 = sortPosesByDistanceToPoint(reachable_poses_half2, reference_point_half2, half_direction_2, target_distance);

  //==========================================================================
  // 构造响应：按顺序 [OUT1, HALF1, MID, HALF2, OUT2]
  //==========================================================================
  geometry_msgs::PoseArray target_poses_msg;
  target_poses_msg.header.frame_id = "link00";
  target_poses_msg.header.stamp = ros::Time::now();

  if (!reachable_poses_out1.empty()) {
    res.target_poses.push_back(reachable_poses_out1[0].pose);
    target_poses_msg.poses.push_back(reachable_poses_out1[0].pose);
    geometry_msgs::Pose& pose = reachable_poses_out1[0].pose;
    res.pitch_angles.push_back(pitch_1);
    res.roll_angles.push_back(roll_1);
    res.pose_names.push_back("OUT1");
  }

  if (!reachable_poses_half1.empty()) {
    res.target_poses.push_back(reachable_poses_half1[0].pose);
    target_poses_msg.poses.push_back(reachable_poses_half1[0].pose);
    geometry_msgs::Pose& pose = reachable_poses_half1[0].pose;
    res.pitch_angles.push_back(pitch_half1);
    res.roll_angles.push_back(roll_half1);
    res.pose_names.push_back("HALF1");
  }

  if (!reachable_poses_mid.empty()) {
    res.target_poses.push_back(reachable_poses_mid[0].pose);
    target_poses_msg.poses.push_back(reachable_poses_mid[0].pose);
    geometry_msgs::Pose& pose = reachable_poses_mid[0].pose;
    res.pitch_angles.push_back(pitch_0);
    res.roll_angles.push_back(roll_0);
    res.pose_names.push_back("MID");
  }

  if (!reachable_poses_half2.empty()) {
    res.target_poses.push_back(reachable_poses_half2[0].pose);
    target_poses_msg.poses.push_back(reachable_poses_half2[0].pose);
    geometry_msgs::Pose& pose = reachable_poses_half2[0].pose;
    res.pitch_angles.push_back(pitch_half2);
    res.roll_angles.push_back(roll_half2);
    res.pose_names.push_back("HALF2");
  }

  if (!reachable_poses_out2.empty()) {
    res.target_poses.push_back(reachable_poses_out2[0].pose);
    target_poses_msg.poses.push_back(reachable_poses_out2[0].pose);
    geometry_msgs::Pose& pose = reachable_poses_out2[0].pose;
    res.pitch_angles.push_back(pitch_2);
    res.roll_angles.push_back(roll_2);
    res.pose_names.push_back("OUT2");
  }
  // 发布目标点位
  target_poses_pub_.publish(target_poses_msg);
  ROS_INFO("[GetGoalAndAngle] Returning %zu target poses", res.target_poses.size());

  //==========================================================================
  // 步骤5：发布可视化数据到RViz
  //==========================================================================
  int viz_points = 100;  // 可视化点数

  // 发布 MID 直线
  line_mid_pub_.publish(createLineVisualization(line_mid_sampled, mid_sample_start, mid_sample_end, viz_points));
  ROS_INFO("[COMBINE_MODE_VIZ] Published MID line");

  // 发布 HALF1 直线
  line_half1_pub_.publish(createLineVisualization(line_half1_sampled, sample_start, sample_end, viz_points));
  ROS_INFO("[COMBINE_MODE_VIZ] Published HALF1 line");

  // 发布 HALF2 直线
  line_half2_pub_.publish(createLineVisualization(line_half2_sampled, sample_start, sample_end, viz_points));
  ROS_INFO("[COMBINE_MODE_VIZ] Published HALF2 line");

  // 发布 OUT1 直线
  line_out1_pub_.publish(createLineVisualization(line_out1_sampled, sample_start, sample_end, viz_points));
  ROS_INFO("[COMBINE_MODE_VIZ] Published OUT1 line");

  // 发布 OUT2 直线
  line_out2_pub_.publish(createLineVisualization(line_out2_sampled, sample_start, sample_end, viz_points));
  ROS_INFO("[COMBINE_MODE_VIZ] Published OUT2 line");

  // 发布参考点
  geometry_msgs::PoseArray reference_points_msg;
  reference_points_msg.header.frame_id = "link00";
  reference_points_msg.header.stamp = ros::Time::now();

  // 添加五个参考点
  geometry_msgs::Pose ref_pose;
  // MID 参考点
  ref_pose = Line3DToPose(line_mid_sampled);
  reference_points_msg.poses.push_back(ref_pose);

  // HALF1 参考点
  ref_pose = Line3DToPose(line_half1_sampled);
  reference_points_msg.poses.push_back(ref_pose);

  // HALF2 参考点
  ref_pose = Line3DToPose(line_half2_sampled);
  reference_points_msg.poses.push_back(ref_pose);
  // OUT1 参考点
  ref_pose = Line3DToPose(line_out1_sampled);
  reference_points_msg.poses.push_back(ref_pose);

  // OUT2 参考点
  ref_pose = Line3DToPose(line_out2_sampled);
  reference_points_msg.poses.push_back(ref_pose);

  cross_reference_points_pub_.publish(reference_points_msg);
  ROS_INFO("[COMBINE_MODE_VIZ] Published %zu reference points", reference_points_msg.poses.size());
  res.call_success = (res.target_poses.size() > 0);
  return true;
}

bool ArmController::getCrossGoalAndAngleServer(arm_controller_srvs::getgoalandangle::Request& req, arm_controller_srvs::getgoalandangle::Response& res) {
  ROS_INFO("[CROSS_MODE_GET_GOAL_AND_ANGLE] Service called, computing target poses...");
  res.call_success = false;
  // 清空输出
  res.target_poses.clear();
  res.pitch_angles.clear();
  res.roll_angles.clear();
  res.pose_names.clear();
  // 读取参数
  double mid_sample_start, mid_sample_end, sample_start, sample_end;
  int mid_num_samples, num_samples;
  double angle_step_deg;
  double half_offset_distance = -0.0707;
  double mid_offset_distance = -0.10;

  nh_.param("test/mid_sample_start", mid_sample_start, -1.0);
  nh_.param("test/mid_sample_end", mid_sample_end, 0.5);
  nh_.param("test/mid_num_samples", mid_num_samples, 50);
  nh_.param("test/angle_step_deg", angle_step_deg, 45.0);
  nh_.param("test/num_samples", num_samples, 50);
  nh_.param("test/sample_start", sample_start, -1.0);
  nh_.param("test/sample_end", sample_end, 0.0);
  // 发布输入位姿到 TF
  if (static_br_ptr_) {
    geometry_msgs::TransformStamped t_input_msg;

    t_input_msg.header.stamp = ros::Time::now();
    t_input_msg.header.frame_id = req.target_pose.header.frame_id;  // 父: "link00"
    t_input_msg.child_frame_id = "input_frame";
    t_input_msg.transform.translation.x = req.target_pose.pose.position.x;
    t_input_msg.transform.translation.y = req.target_pose.pose.position.y;
    t_input_msg.transform.translation.z = req.target_pose.pose.position.z;
    t_input_msg.transform.rotation = req.target_pose.pose.orientation;

    static_br_ptr_->sendTransform(t_input_msg);
  } else {
    ROS_ERROR("[GetCrossGoalAndAngle] TF Broadcaster is not initialized!");
    return true;
  }
  double center_x, center_y, center_z;
  double center_roll, center_pitch, center_yaw;
  tf2::Quaternion quat_transformed;
  //==========================================================================
  // 步骤1：读取参数和解析输入位姿
  //==========================================================================
  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform("link00", "flower_frame", ros::Time(0), ros::Duration(1.0));

    center_x = transform_stamped.transform.translation.x;
    center_y = transform_stamped.transform.translation.y;
    center_z = transform_stamped.transform.translation.z;

    // 四元数变换
    tf2::fromMsg(transform_stamped.transform.rotation, quat_transformed);

  } catch (tf2::TransformException& ex) {
    ROS_WARN("Could not get transform: %s", ex.what());
    return true;
  }

  tf2::Matrix3x3 mat(quat_transformed);
  mat.getRPY(center_roll, center_pitch, center_yaw);

  // 生成三个方向向量
  tf2::Vector3 x_axis_in_flower_frame(1.0, 0.0, 0.0);
  tf2::Vector3 y_axis_in_flower_frame(0.0, 1.0, 0.0);
  tf2::Vector3 z_axis_in_flower_frame(0.0, 0.0, 1.0);
  tf2::Vector3 rotation_axis_flower_x_in_link00 = tf2::quatRotate(quat_transformed, x_axis_in_flower_frame);
  tf2::Vector3 rotation_axis_flower_y_in_link00 = tf2::quatRotate(quat_transformed, y_axis_in_flower_frame);
  tf2::Vector3 rotation_axis_flower_z_in_link00 = tf2::quatRotate(quat_transformed, z_axis_in_flower_frame);
  tf2::Vector3 origin_direction = rotation_axis_flower_x_in_link00.normalized();
  //==========================================================================
  // 步骤2：生成5条直线
  //==========================================================================
  Line3D original_line;
  original_line.point = Eigen::Vector3d(center_x, center_y, center_z);
  original_line.direction = Eigen::Vector3d(origin_direction.x(), origin_direction.y(), origin_direction.z()).normalized();

  Eigen::Vector3d rotation_center(center_x, center_y, center_z);
  Eigen::Vector3d rotation_axis_left_eigen(rotation_axis_flower_z_in_link00.x(), rotation_axis_flower_z_in_link00.y(), rotation_axis_flower_z_in_link00.z());
  Eigen::Vector3d rotation_axis_top_eigen(rotation_axis_flower_y_in_link00.x(), rotation_axis_flower_y_in_link00.y(), rotation_axis_flower_y_in_link00.z());
  Eigen::Vector3d reference_point_mid = original_line.point + mid_offset_distance * original_line.direction;

  // bebug：旋转轴
  geometry_msgs::PoseStamped axes_poses_msg;
  axes_poses_msg.header.frame_id = "link00";
  axes_poses_msg.header.stamp = ros::Time::now();

  Line3D axis_line_left;  // 左右旋转轴
  axis_line_left.point = rotation_center;
  axis_line_left.direction = rotation_axis_left_eigen.normalized();

  Line3D axis_line_top;  // 上下旋转轴
  axis_line_top.point = rotation_center;
  axis_line_top.direction = rotation_axis_top_eigen.normalized();

  // 4. 发布
  cross_rotation_transform_left_pub.publish(Line3DToPoseStamped(axis_line_left));
  cross_rotation_transform_top_pub.publish(Line3DToPoseStamped(axis_line_top));

  double angle_step_rad_1_45 = angle_step_deg * M_PI / 180.0;
  double angle_step_rad_1_90 = 2 * angle_step_deg * M_PI / 180.0;  // 作为平移轴
  double angle_step_rad_2_45 = (360.0 - angle_step_deg) * M_PI / 180.0;
  double angle_step_rad_2_90 = (360.0 - 2 * angle_step_deg) * M_PI / 180.0;  // 作为平移轴
  Line3D line_mid = original_line;
  Line3D line_left1 = rotateLine(original_line, rotation_center, rotation_axis_left_eigen, angle_step_rad_1_45);
  Line3D line_left1_90 = rotateLine(original_line, rotation_center, rotation_axis_left_eigen, angle_step_rad_1_90);
  Line3D line_left2 = rotateLine(original_line, rotation_center, rotation_axis_left_eigen, angle_step_rad_2_45);
  Line3D line_left2_90 = rotateLine(original_line, rotation_center, rotation_axis_left_eigen, angle_step_rad_2_90);
  Line3D line_top1 = rotateLine(original_line, rotation_center, rotation_axis_top_eigen, angle_step_rad_1_45);
  Line3D line_top1_90 = rotateLine(original_line, rotation_center, rotation_axis_top_eigen, angle_step_rad_1_90);
  Line3D line_top2 = rotateLine(original_line, rotation_center, rotation_axis_top_eigen, angle_step_rad_2_45);
  Line3D line_top2_90 = rotateLine(original_line, rotation_center, rotation_axis_top_eigen, angle_step_rad_2_90);
  //==========================================================================
  // 步骤3：计算参考点并检查顺序
  //==========================================================================
  Eigen::Vector3d mid_direction = line_mid.direction.normalized();
  Eigen::Vector3d left_direction_1 = line_left1.direction.normalized();
  Eigen::Vector3d left_direction_2 = line_left2.direction.normalized();
  Eigen::Vector3d top_direction_1 = line_top1.direction.normalized();
  Eigen::Vector3d top_direction_2 = line_top2.direction.normalized();

  double half_square_size = -0.05;
  Eigen::Vector3d reference_point_left1 = original_line.point + line_left1_90.direction.normalized() * half_square_size + half_offset_distance * left_direction_1;
  Eigen::Vector3d reference_point_left2 = original_line.point + line_left2_90.direction.normalized() * half_square_size + half_offset_distance * left_direction_2;
  Eigen::Vector3d reference_point_top1 = original_line.point + line_top1_90.direction.normalized() * half_square_size + half_offset_distance * top_direction_1;
  Eigen::Vector3d reference_point_top2 = original_line.point + line_top2_90.direction.normalized() * half_square_size + half_offset_distance * top_direction_2;

  // 创建以参考点为起点的直线
  Line3D line_mid_sampled;
  line_mid_sampled.point = reference_point_mid;
  line_mid_sampled.direction = mid_direction;

  Line3D line_left1_sampled;
  line_left1_sampled.point = reference_point_left1;
  line_left1_sampled.direction = left_direction_1;

  Line3D line_left2_sampled;
  line_left2_sampled.point = reference_point_left2;
  line_left2_sampled.direction = left_direction_2;

  Line3D line_top1_sampled;
  line_top1_sampled.point = reference_point_top1;
  line_top1_sampled.direction = top_direction_1;

  Line3D line_top2_sampled;
  line_top2_sampled.point = reference_point_top2;
  line_top2_sampled.direction = top_direction_2;

  //==========================================================================
  // 步骤4：采样并检测可达性
  //==========================================================================
  double pitch_mid = 0.0, roll_mid = 0.0;
  double pitch_left1 = 0.0, roll_left1 = 0.0;
  double pitch_left2 = 0.0, roll_left2 = 0.0;
  double pitch_top1 = 0.0, roll_top1 = 0.0;
  double pitch_top2 = 0.0, roll_top2 = 0.0;
  geometry_msgs::Pose debug_camera_poses_demo;
  geometry_msgs::PoseArray debug_camera_poses_msg;
  debug_camera_poses_msg.header.frame_id = "gripperMover";  // gripperMover
  debug_camera_poses_msg.header.stamp = ros::Time::now();
  std::cout << "[1/5] Sampling MID line..." << std::endl;
  std::vector<geometry_msgs::PoseStamped> reachable_poses_mid = sampleAndCheckReachability(line_mid_sampled, mid_sample_start, mid_sample_end, mid_num_samples, pitch_mid, roll_mid, debug_camera_poses_demo);
  // debug_camera_poses_msg.poses.push_back(debug_camera_poses_demo);
  std::cout << "[2/5] Sampling LEFT1 line..." << std::endl;
  std::vector<geometry_msgs::PoseStamped> reachable_poses_left1 = sampleAndCheckReachability(line_left1_sampled, sample_start, sample_end, num_samples, pitch_left1, roll_left1, debug_camera_poses_demo);
  // debug_camera_poses_msg.poses.push_back(debug_camera_poses_demo);
  std::cout << "[3/5] Sampling LEFT2 line..." << std::endl;
  std::vector<geometry_msgs::PoseStamped> reachable_poses_left2 = sampleAndCheckReachability(line_left2_sampled, sample_start, sample_end, num_samples, pitch_left2, roll_left2, debug_camera_poses_demo);
  // debug_camera_poses_msg.poses.push_back(debug_camera_poses_demo);
  std::cout << "[4/5] Sampling TOP1 line..." << std::endl;
  std::vector<geometry_msgs::PoseStamped> reachable_poses_top1 = sampleAndCheckReachability(line_top1_sampled, sample_start, sample_end, num_samples, pitch_top1, roll_top1, debug_camera_poses_demo);
  // debug_camera_poses_msg.poses.push_back(debug_camera_poses_demo);
  std::cout << "[5/5] Sampling TOP2 line..." << std::endl;
  std::vector<geometry_msgs::PoseStamped> reachable_poses_top2 = sampleAndCheckReachability(line_top2_sampled, sample_start, sample_end, num_samples, pitch_top2, roll_top2, debug_camera_poses_demo);
  // debug_camera_poses_msg.poses.push_back(debug_camera_poses_demo);

  // if (!debug_camera_poses_msg.poses.empty()) {
  //   camera_transformed_pose_pub_.publish(debug_camera_poses_msg);
  //   std::cout << "[Viz] Published " << debug_camera_poses_msg.poses.size() << " camera poses in link00 frame." << std::endl;
  // }
  ROS_INFO(
      "[CROSS_MODE_GET_GOAL_AND_ANGLE] Reachable poses: MID=%zu, "
      "LEFT1=%zu, LEFT2=%zu,  TOP1=%zu, TOP2=%zu",
      reachable_poses_mid.size(), reachable_poses_left1.size(), reachable_poses_left2.size(), reachable_poses_top1.size(), reachable_poses_top2.size());

  // 排序
  double target_distance, mid_target_distance;
  nh_.param("test/target_distance", target_distance, 0.2);
  nh_.param("test/mid_target_distance", mid_target_distance, 0.2);

  reachable_poses_mid = sortPosesByDistanceToPoint(reachable_poses_mid, reference_point_mid, mid_direction, mid_target_distance);
  reachable_poses_left1 = sortPosesByDistanceToPoint(reachable_poses_left1, reference_point_left1, left_direction_1, target_distance);
  reachable_poses_left2 = sortPosesByDistanceToPoint(reachable_poses_left2, reference_point_left2, left_direction_2, target_distance);
  reachable_poses_top1 = sortPosesByDistanceToPoint(reachable_poses_top1, reference_point_top1, top_direction_1, target_distance);
  reachable_poses_top2 = sortPosesByDistanceToPoint(reachable_poses_top2, reference_point_top2, top_direction_2, target_distance);

  //==========================================================================
  // 构造响应：按顺序 [LEFT1, TOP1, TOP2, LEFT2, MID]
  //==========================================================================
  geometry_msgs::PoseArray target_poses_msg;
  target_poses_msg.header.frame_id = "link00";
  target_poses_msg.header.stamp = ros::Time::now();

  if (!reachable_poses_left1.empty()) {
    res.target_poses.push_back(reachable_poses_left1[0].pose);
    target_poses_msg.poses.push_back(reachable_poses_left1[0].pose);
    res.pitch_angles.push_back(pitch_left1);
    res.roll_angles.push_back(roll_left1);
    res.pose_names.push_back("LEFT1");
  }

  if (!reachable_poses_top1.empty()) {
    res.target_poses.push_back(reachable_poses_top1[0].pose);
    target_poses_msg.poses.push_back(reachable_poses_top1[0].pose);
    res.pitch_angles.push_back(pitch_top1);
    res.roll_angles.push_back(roll_top1);
    res.pose_names.push_back("TOP1");
  }

  if (!reachable_poses_top2.empty()) {
    res.target_poses.push_back(reachable_poses_top2[0].pose);
    target_poses_msg.poses.push_back(reachable_poses_top2[0].pose);
    res.pitch_angles.push_back(pitch_top2);
    res.roll_angles.push_back(roll_top2);
    res.pose_names.push_back("TOP2");
  }

  if (!reachable_poses_left2.empty()) {
    res.target_poses.push_back(reachable_poses_left2[0].pose);
    target_poses_msg.poses.push_back(reachable_poses_left2[0].pose);
    res.pitch_angles.push_back(pitch_left2);
    res.roll_angles.push_back(roll_left2);
    res.pose_names.push_back("LEFT2");
  }

  if (!reachable_poses_mid.empty()) {
    res.target_poses.push_back(reachable_poses_mid[0].pose);
    target_poses_msg.poses.push_back(reachable_poses_mid[0].pose);
    res.pitch_angles.push_back(pitch_mid);
    res.roll_angles.push_back(roll_mid);
    res.pose_names.push_back("MID");
  }

  // 发布目标点位
  target_poses_pub_.publish(target_poses_msg);
  ROS_INFO("[CROSS_MODE_GET_GOAL_AND_ANGLE] Returning %zu target poses", res.target_poses.size());

  //==========================================================================
  // 步骤5：发布可视化数据到RViz
  //==========================================================================
  int viz_points = 100;  // 可视化点数

  // 发布 MID 直线
  cross_line_mid_pub_.publish(createLineVisualization(line_mid_sampled, mid_sample_start, mid_sample_end, viz_points));
  ROS_INFO("[CROSS_MODE_VIZ] Published MID line");

  // 发布 LEFT1 直线
  cross_line_left1_pub_.publish(createLineVisualization(line_left1_sampled, sample_start, sample_end, viz_points));
  ROS_INFO("[CROSS_MODE_VIZ] Published LEFT1 line");

  // 发布 LEFT2 直线
  cross_line_left2_pub_.publish(createLineVisualization(line_left2_sampled, sample_start, sample_end, viz_points));
  ROS_INFO("[CROSS_MODE_VIZ] Published LEFT2 line");

  // 发布 TOP1 直线
  cross_line_top1_pub_.publish(createLineVisualization(line_top1_sampled, sample_start, sample_end, viz_points));
  ROS_INFO("[CROSS_MODE_VIZ] Published TOP1 line");

  // 发布 TOP2 直线
  cross_line_top2_pub_.publish(createLineVisualization(line_top2_sampled, sample_start, sample_end, viz_points));
  ROS_INFO("[CROSS_MODE_VIZ] Published TOP2 line");

  // 发布参考点
  geometry_msgs::PoseArray reference_points_msg;
  reference_points_msg.header.frame_id = "link00";
  reference_points_msg.header.stamp = ros::Time::now();

  // 添加五个参考点
  geometry_msgs::Pose ref_pose;
  // MID 参考点
  ref_pose = Line3DToPose(line_mid_sampled);
  reference_points_msg.poses.push_back(ref_pose);

  // LEFT1 参考点
  ref_pose = Line3DToPose(line_left1_sampled);
  reference_points_msg.poses.push_back(ref_pose);

  // LEFT2 参考点
  ref_pose = Line3DToPose(line_left2_sampled);
  reference_points_msg.poses.push_back(ref_pose);
  // TOP1 参考点
  ref_pose = Line3DToPose(line_top1_sampled);
  reference_points_msg.poses.push_back(ref_pose);

  // TOP2 参考点
  ref_pose = Line3DToPose(line_top2_sampled);
  reference_points_msg.poses.push_back(ref_pose);

  cross_reference_points_pub_.publish(reference_points_msg);
  ROS_INFO("[CROSS_MODE_VIZ] Published %zu reference points", reference_points_msg.poses.size());
  res.call_success = (res.target_poses.size() > 0);
  return true;
}

bool ArmController::PlanTouchGoalAndAngleServer(arm_controller_srvs::getgoalandangle::Request& req, arm_controller_srvs::getgoalandangle::Response& res) {
  ROS_INFO(
      "[PlanTouchGoalAndAngleServer] Service called, computing target "
      "poses...");
  res.call_success = false;

  // 清空输出
  res.target_poses.clear();
  res.pitch_angles.clear();
  res.roll_angles.clear();
  res.pose_names.clear();

  //==========================================================================
  // 步骤0：获取 camera_link 到 link00 的 TF 变换
  //==========================================================================
  geometry_msgs::TransformStamped camera_to_link00_transform;
  Eigen::Vector3d pen_offset_in_link00(0.0, 0.0, 0.0);  // 默认无偏移

  try {
    // 查询 camera_link 到 link00 的变换
    camera_to_link00_transform = tf_buffer_.lookupTransform("link00", "camera_link", ros::Time(0), ros::Duration(1.0));

    // 定义笔相对于 camera_link 的偏移 (0.0, 0.05, 0.0)
    geometry_msgs::Vector3Stamped pen_offset_camera;
    pen_offset_camera.header.frame_id = "camera_link";
    pen_offset_camera.header.stamp = ros::Time::now();
    pen_offset_camera.vector.x = 0.0;
    pen_offset_camera.vector.y = -0.05;
    pen_offset_camera.vector.z = 0.0;

    // 将偏移向量从 camera_link 变换到 link00
    geometry_msgs::Vector3Stamped pen_offset_link00;
    tf2::doTransform(pen_offset_camera, pen_offset_link00, camera_to_link00_transform);

    // 转换为 Eigen 向量
    pen_offset_in_link00 = Eigen::Vector3d(pen_offset_link00.vector.x, pen_offset_link00.vector.y, pen_offset_link00.vector.z);

    ROS_INFO(
        "[PlanTouchGoalAndAngleServer] Pen offset in link00: [%.4f, %.4f, "
        "%.4f]",
        pen_offset_in_link00.x(), pen_offset_in_link00.y(), pen_offset_in_link00.z());

    // 发布 pen_offset_link 虚拟坐标系用于可视化
    // 该坐标系相对于 camera_link 偏移 (0.0, 0.05, 0.0)，方向与 camera_link 一致
    geometry_msgs::TransformStamped pen_offset_msg;

    pen_offset_msg.header.stamp = ros::Time::now();
    pen_offset_msg.header.frame_id = "camera_link";
    pen_offset_msg.child_frame_id = "pen_offset_link";

    pen_offset_msg.transform.translation.x = 0.0;
    pen_offset_msg.transform.translation.y = -0.05;
    pen_offset_msg.transform.translation.z = 0.0;

    pen_offset_msg.transform.rotation.x = 0.0;
    pen_offset_msg.transform.rotation.y = 0.0;
    pen_offset_msg.transform.rotation.z = 0.0;
    pen_offset_msg.transform.rotation.w = 1.0;

    tf_broadcaster_.sendTransform(pen_offset_msg);

    ROS_DEBUG(
        "[PlanTouchGoalAndAngleServer] Published TF frame "
        "'pen_offset_link' relative to 'camera_link'");

  } catch (tf2::TransformException& ex) {
    ROS_WARN(
        "[PlanTouchGoalAndAngleServer] Could not get camera_link to "
        "link00 transform: %s",
        ex.what());
    ROS_WARN(
        "[PlanTouchGoalAndAngleServer] Using default offset (0, -0.05, 0) "
        "in link00 frame");
    // 如果无法获取 TF，使用默认偏移（向下 0.05m）
    pen_offset_in_link00 = Eigen::Vector3d(0.0, -0.05, 0.0);
  }

  //==========================================================================
  // 步骤1：读取参数和解析输入位姿
  //==========================================================================
  double line_x = req.target_pose.pose.position.x;
  double line_y = req.target_pose.pose.position.y;
  double line_z = req.target_pose.pose.position.z;

  // 四元数变换
  tf2::Quaternion quat_input(req.target_pose.pose.orientation.x, req.target_pose.pose.orientation.y, req.target_pose.pose.orientation.z, req.target_pose.pose.orientation.w);

  tf2::Quaternion rot_y_inv;
  rot_y_inv.setRotation(tf2::Vector3(0, 1, 0), M_PI / 2.0);
  tf2::Quaternion rot_x_inv;
  rot_x_inv.setRotation(tf2::Vector3(1, 0, 0), M_PI / 2.0);
  tf2::Quaternion rot_y_180;
  rot_y_180.setRotation(tf2::Vector3(0, 1, 0), M_PI);
  tf2::Quaternion quat = quat_input * rot_y_inv * rot_x_inv * rot_y_180;

  double line_roll_link00, line_pitch_link00, line_yaw_link00;
  tf2::Matrix3x3 mat(quat);
  mat.getRPY(line_roll_link00, line_pitch_link00, line_yaw_link00);

  double line_pitch = line_pitch_link00;
  double line_yaw = line_yaw_link00;
  double line_roll = line_roll_link00;

  // 读取参数
  double mid_sample_start, mid_sample_end, sample_start, sample_end;
  int mid_num_samples, num_samples;
  double angle_step_deg;
  double half_offset_distance = -0.15;
  double mid_offset_distance = -0.15;

  nh_.param("test/mid_sample_start", mid_sample_start, -1.0);
  nh_.param("test/mid_sample_end", mid_sample_end, 0.5);
  nh_.param("test/mid_num_samples", mid_num_samples, 50);
  nh_.param("test/angle_step_deg", angle_step_deg, 45.0);
  nh_.param("test/num_samples", num_samples, 50);
  nh_.param("test/sample_start", sample_start, -1.0);
  nh_.param("test/sample_end", sample_end, 0.0);

  // 计算方向向量
  tf2::Vector3 x_axis(1.0, 0.0, 0.0);
  tf2::Vector3 rotated_direction = tf2::quatRotate(quat, x_axis);
  double origin_direction_x = rotated_direction.x();
  double origin_direction_y = rotated_direction.y();
  double origin_direction_z = rotated_direction.z();

  // 使用TF的四元数来实现90度坐标系旋转
  // 创建一个绕Y轴旋转90度的四元数
  tf2::Quaternion rotation_transform_left;
  rotation_transform_left.setRPY(0, M_PI / 2, 0);  // Roll=0, Pitch=90度, Yaw=0
  tf2::Quaternion rotation_transform_top;
  rotation_transform_top.setRPY(0, 0,
                                M_PI / 2);  // Roll=0, Pitch=0度, Yaw=90度

  // 应用变换得到旋转轴
  tf2::Vector3 rotation_axis_vec_left = tf2::quatRotate(rotation_transform_left, rotated_direction);
  double rotation_axis_left_x = rotation_axis_vec_left.x();
  double rotation_axis_left_y = rotation_axis_vec_left.y();
  double rotation_axis_left_z = rotation_axis_vec_left.z();
  tf2::Vector3 rotation_axis_vec_top = tf2::quatRotate(rotation_transform_top, rotated_direction);
  double rotation_axis_top_x = rotation_axis_vec_top.x();
  double rotation_axis_top_y = rotation_axis_vec_top.y();
  double rotation_axis_top_z = rotation_axis_vec_top.z();
  // 计算平移轴
  // Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(line_yaw,
  // Eigen::Vector3d::UnitZ()).toRotationMatrix(); Eigen::Matrix3d R_pitch =
  // Eigen::AngleAxisd(line_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
  // Eigen::Matrix3d R_roll = Eigen::AngleAxisd(line_roll,
  // Eigen::Vector3d::UnitX()).toRotationMatrix(); Eigen::Matrix3d R_total =
  // R_yaw * R_pitch * R_roll; Eigen::Vector3d original_y_axis(0.0, 1.0, 0.0);
  // Eigen::Vector3d translation_axis_vec = R_total * original_y_axis;

  //==========================================================================
  // 步骤2：生成5条直线
  //==========================================================================
  Line3D original_line;
  original_line.point = Eigen::Vector3d(line_x, line_y, line_z);
  original_line.direction = Eigen::Vector3d(origin_direction_x, origin_direction_y, origin_direction_z).normalized();

  Eigen::Vector3d rotation_center(line_x, line_y, line_z);
  Eigen::Vector3d rotation_axis_left(rotation_axis_left_x, rotation_axis_left_y, rotation_axis_left_z);
  Eigen::Vector3d rotation_axis_top(rotation_axis_top_x, rotation_axis_top_y, rotation_axis_top_z);
  Eigen::Vector3d base_point = original_line.point;
  Eigen::Vector3d reference_point_mid = base_point + mid_offset_distance * original_line.direction;

  Line3D line_mid = original_line;
  double angle_step_rad_1 = angle_step_deg * M_PI / 180.0;
  Line3D line_left1 = rotateLine(original_line, rotation_center, rotation_axis_left, angle_step_rad_1);
  double angle_step_rad_2 = (360.0 - angle_step_deg) * M_PI / 180.0;
  Line3D line_left2 = rotateLine(original_line, rotation_center, rotation_axis_top, angle_step_rad_2);
  Line3D line_top1 = rotateLine(original_line, rotation_center, rotation_axis_left, angle_step_rad_1);
  Line3D line_top2 = rotateLine(original_line, rotation_center, rotation_axis_top, angle_step_rad_2);

  //==========================================================================
  // 步骤3：计算参考点并检查顺序
  //==========================================================================
  Eigen::Vector3d mid_direction = line_mid.direction.normalized();
  Eigen::Vector3d left_direction_1 = line_left1.direction.normalized();
  Eigen::Vector3d left_direction_2 = line_left2.direction.normalized();
  Eigen::Vector3d top_direction_1 = line_top1.direction.normalized();
  Eigen::Vector3d top_direction_2 = line_top2.direction.normalized();

  // Eigen::Vector3d reference_point_out1 = base_point + translation_axis_vec *
  // distance_1; Eigen::Vector3d reference_point_out2 = base_point +
  // translation_axis_vec * distance_2;

  // 计算参考点（沿直线方向偏移）
  Eigen::Vector3d reference_point_left1 = base_point + half_offset_distance * left_direction_1;
  Eigen::Vector3d reference_point_left2 = base_point + half_offset_distance * left_direction_2;
  Eigen::Vector3d reference_point_top1 = base_point + half_offset_distance * top_direction_1;
  Eigen::Vector3d reference_point_top2 = base_point + half_offset_distance * top_direction_2;

  // 根据笔的位置调整参考点位置（使用 TF 变换得到的偏移）
  reference_point_left1 -= pen_offset_in_link00;
  reference_point_left2 -= pen_offset_in_link00;
  reference_point_top1 -= pen_offset_in_link00;
  reference_point_top2 -= pen_offset_in_link00;

  ROS_DEBUG(
      "[PlanTouchGoalAndAngleServer] Reference points after pen offset "
      "adjustment:");
  ROS_DEBUG("  LEFT1: [%.4f, %.4f, %.4f]", reference_point_left1.x(), reference_point_left1.y(), reference_point_left1.z());
  ROS_DEBUG("  LEFT2: [%.4f, %.4f, %.4f]", reference_point_left2.x(), reference_point_left2.y(), reference_point_left2.z());
  ROS_DEBUG("  top1: [%.4f, %.4f, %.4f]", reference_point_top1.x(), reference_point_top1.y(), reference_point_top1.z());
  ROS_DEBUG("  top2: [%.4f, %.4f, %.4f]", reference_point_top2.x(), reference_point_top2.y(), reference_point_top2.z());

  // 创建以参考点为起点的直线
  Line3D line_mid_sampled;
  line_mid_sampled.point = reference_point_mid;
  line_mid_sampled.direction = mid_direction;

  Line3D line_left1_sampled;
  line_left1_sampled.point = reference_point_left1;
  line_left1_sampled.direction = left_direction_1;

  Line3D line_left2_sampled;
  line_left2_sampled.point = reference_point_left2;
  line_left2_sampled.direction = left_direction_2;

  Line3D line_top1_sampled;
  line_top1_sampled.point = reference_point_top1;
  line_top1_sampled.direction = top_direction_1;

  Line3D line_top2_sampled;
  line_top2_sampled.point = reference_point_top2;
  line_top2_sampled.direction = top_direction_2;

  //==========================================================================
  // 步骤4：采样并检测可达性
  //==========================================================================
  double pitch_mid = 0.0, roll_mid = 0.0;
  double pitch_left1 = 0.0, roll_left1 = 0.0;
  double pitch_left2 = 0.0, roll_left2 = 0.0;
  double pitch_top1 = 0.0, roll_top1 = 0.0;
  double pitch_top2 = 0.0, roll_top2 = 0.0;
  geometry_msgs::Pose debug_camera_poses_demo;
  std::vector<geometry_msgs::PoseStamped> reachable_poses_mid = sampleAndCheckReachability(line_mid_sampled, mid_sample_start, mid_sample_end, mid_num_samples, pitch_mid, roll_mid, debug_camera_poses_demo);

  std::vector<geometry_msgs::PoseStamped> reachable_poses_left1 = sampleAndCheckReachability(line_left1_sampled, sample_start, sample_end, num_samples, pitch_left1, roll_left1, debug_camera_poses_demo);

  std::vector<geometry_msgs::PoseStamped> reachable_poses_left2 = sampleAndCheckReachability(line_left2_sampled, sample_start, sample_end, num_samples, pitch_left2, roll_left2, debug_camera_poses_demo);

  std::vector<geometry_msgs::PoseStamped> reachable_poses_top1 = sampleAndCheckReachability(line_top1_sampled, sample_start, sample_end, num_samples, pitch_top1, roll_top1, debug_camera_poses_demo);

  std::vector<geometry_msgs::PoseStamped> reachable_poses_top2 = sampleAndCheckReachability(line_top2_sampled, sample_start, sample_end, num_samples, pitch_top2, roll_top2, debug_camera_poses_demo);

  ROS_INFO(
      "[CROSS_MODE_GET_GOAL_AND_ANGLE] Reachable poses: MID=%zu, "
      "LEFT1=%zu, LEFT2=%zu, top1=%zu, top2=%zu",
      reachable_poses_mid.size(), reachable_poses_left1.size(), reachable_poses_left2.size(), reachable_poses_top1.size(), reachable_poses_top2.size());

  // 排序
  double target_distance_for_touch = 0.06;
  nh_.param("test/target_distance_for_touch", target_distance_for_touch, 0.2);

  reachable_poses_mid = sortPosesByDistanceToPoint(reachable_poses_mid, reference_point_mid, mid_direction, target_distance_for_touch);
  reachable_poses_left1 = sortPosesByDistanceToPoint(reachable_poses_left1, reference_point_left1, left_direction_1, target_distance_for_touch);
  reachable_poses_left2 = sortPosesByDistanceToPoint(reachable_poses_left2, reference_point_left2, left_direction_2, target_distance_for_touch);
  reachable_poses_top1 = sortPosesByDistanceToPoint(reachable_poses_top1, reference_point_top1, top_direction_1, target_distance_for_touch);
  reachable_poses_top2 = sortPosesByDistanceToPoint(reachable_poses_top2, reference_point_top2, top_direction_2, target_distance_for_touch);

  //==========================================================================
  // 构造响应：按顺序 [MID, LEFT1, top1, top2, LEFT2]
  //==========================================================================
  if (!reachable_poses_mid.empty()) {
    res.target_poses.push_back(reachable_poses_mid[0].pose);
    res.pitch_angles.push_back(pitch_mid);
    res.roll_angles.push_back(roll_mid);
    res.pose_names.push_back("MID");
  }

  if (!reachable_poses_left1.empty()) {
    res.target_poses.push_back(reachable_poses_left1[0].pose);
    res.pitch_angles.push_back(pitch_left1);
    res.roll_angles.push_back(roll_left1);
    res.pose_names.push_back("LEFT1");
  }

  if (!reachable_poses_top1.empty()) {
    res.target_poses.push_back(reachable_poses_top1[0].pose);
    res.pitch_angles.push_back(pitch_top1);
    res.roll_angles.push_back(roll_top1);
    res.pose_names.push_back("top1");
  }

  if (!reachable_poses_top2.empty()) {
    res.target_poses.push_back(reachable_poses_top2[0].pose);
    res.pitch_angles.push_back(pitch_top2);
    res.roll_angles.push_back(roll_top2);
    res.pose_names.push_back("top2");
  }

  if (!reachable_poses_left2.empty()) {
    res.target_poses.push_back(reachable_poses_left2[0].pose);
    res.pitch_angles.push_back(pitch_left2);
    res.roll_angles.push_back(roll_left2);
    res.pose_names.push_back("LEFT2");
  }

  ROS_INFO("[PlanTouchGoalAndAngleServer] Returning %zu target poses", res.target_poses.size());

  res.call_success = (res.target_poses.size() > 0);
  return true;
}

geometry_msgs::PoseArray ArmController::createLineVisualization(Line3D& line, double t_start, double t_end, int num_points) {
  geometry_msgs::PoseArray line_msg;
  line_msg.header.frame_id = "link00";
  line_msg.header.stamp = ros::Time::now();

  for (int i = 0; i < num_points; ++i) {
    double t = t_start + (t_end - t_start) * i / (num_points - 1);
    Eigen::Vector3d point = line.point + t * line.direction;

    geometry_msgs::Pose pose = Line3DToPose(line);
    pose.position.x = point.x();
    pose.position.y = point.y();
    pose.position.z = point.z();
    // pose.orientation.w = 1.0;

    line_msg.poses.push_back(pose);
  }
  return line_msg;
};

bool ArmController::planToTargetPose(const geometry_msgs::Pose& target_pose, const double& joint6_pos, const bool& use_manual_joint6) {
  ROS_INFO("================ [Pinocchio 7-DOF SINGLE PLAN] ================");

  // --------------------------------------------------------
  // 1. 全局配置与预处理
  // --------------------------------------------------------
  if (!pinocchio_ik_) {
    ROS_ERROR("[Critical] PinocchioIK not initialized!");
    return false;
  }

  // 状态检查
  if (arm_control_fsm_ != ArmControlFsm::Home && arm_control_fsm_ != ArmControlFsm::Arrived) {
    ROS_WARN("[State Error] Arm moving. State: %d", static_cast<int>(arm_control_fsm_));
    return false;
  }

  // 四元数归一化
  Eigen::Quaterniond q_check(target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);
  if (std::abs(q_check.norm() - 1.0) > 1e-3) q_check.normalize();

  // 转换目标位姿
  Eigen::Matrix4d target_pose_eigen;
  arm_controller::geometryMsgsPose2Pose(target_pose, target_pose_eigen);
  target_pose_eigen.block<3, 3>(0, 0) = q_check.toRotationMatrix();

  // 设置权重 (Look-At模式: 忽略 Roll)
  Eigen::Matrix<double, 6, 1> weights;
  weights << 1, 1, 1, 0, 1, 1;  // [x, y, z, roll(ignore), pitch, yaw]

  const double TARGET_TOLERANCE = 0.01;  // 目标精度 1cm

  // --------------------------------------------------------
  // 2. 执行核心逻辑 (单次执行)
  // --------------------------------------------------------

  // A. 获取当前真实状态
  Eigen::VectorXd start_state_7d = Eigen::VectorXd::Zero(7);
  start_state_7d.head<6>() = low_state_.getQ();
  start_state_7d[6] = low_state_.getGripperQ();

  Eigen::VectorXd target_state_7d = Eigen::VectorXd::Zero(7);

  // B. 执行 IK
  ros::Time t_start = ros::Time::now();
  bool find_ik = pinocchio_ik_->inverseKinematics(target_pose_eigen, start_state_7d, target_state_7d, weights, 2000);

  if (!find_ik) {
    ROS_WARN("[PlanToTarget] IK Failed to find solution.");
    return false;
  }

  // C. 提取结果
  Eigen::Matrix<double, 6, 1> arm_target_joints = target_state_7d.head<6>();
  double gripper_target_val = target_state_7d[6];

  // D. 判断是否需要运动
  double joint_diff = (arm_target_joints - start_state_7d.head<6>()).norm();
  bool needs_motion = true;

  if (joint_diff < 0.001 && std::abs(gripper_target_val - start_state_7d[6]) < 0.001) {
    ROS_INFO("[PlanToTarget] IK suggests no movement needed. Skipping execution...");
    needs_motion = false;
  }

  // E. 规划与执行
  if (needs_motion) {
    gripper_goal_ = gripper_target_val;
    ee_pose_goal_ = target_pose_eigen;

    arm_joint_goal_ = arm_target_joints;

    // 计算时间
    double max_joint_delta = (arm_target_joints - start_state_7d.head<6>()).cwiseAbs().maxCoeff();
    double duration = max_joint_delta / 1.0;  // 1.0 rad/s
    duration += 0.4;                          // 缓冲
    plan_max_tick_ = std::max(50uL, static_cast<long unsigned int>(duration / control_period_));

    lazyPlan(start_state_7d.head<6>(), arm_joint_goal_, plan_max_tick_);
    setArmControlFsm(ArmControlFsm::PlanMove);

    // F. 等待运动完成
    ros::Rate rate(1.0 / control_period_);
    double timeout = (plan_max_tick_ * control_period_) + 3.0;
    ros::Time wait_start = ros::Time::now();
    bool motion_finished = false;

    while (ros::ok()) {
      if ((ros::Time::now() - wait_start).toSec() > timeout) {
        ROS_WARN("[PlanToTarget] Motion Timeout.");
        break;
      }
      if (arm_control_fsm_ == ArmControlFsm::Arrived) {
        ros::Duration(0.5).sleep();  // 等待稳定
        motion_finished = true;
        break;
      }
      rate.sleep();
    }

    if (!motion_finished) return false;
  }

  // --------------------------------------------------------
  // G. 物理误差检查
  // --------------------------------------------------------

  // 获取最新状态
  Eigen::VectorXd real_q(7);
  real_q.head<6>() = low_state_.getQ();
  real_q[6] = low_state_.getGripperQ();

  // 打印详细关节对比 (调试用)
  std::stringstream ss_target, ss_real, ss_diff;
  ss_target << std::fixed << std::setprecision(3);
  ss_real << std::fixed << std::setprecision(3);
  ss_diff << std::fixed << std::setprecision(3);

  for (int i = 0; i < 7; ++i) {
    double diff = real_q[i] - target_state_7d[i];
    ss_target << target_state_7d[i] << (i < 6 ? ", " : "");
    ss_real << real_q[i] << (i < 6 ? ", " : "");
    ss_diff << diff << (i < 6 ? ", " : "");
  }

  ROS_INFO("---------------------------------------------------------");
  ROS_INFO("[Joint Diff] Target Q: [%s]", ss_target.str().c_str());
  ROS_INFO("[Joint Diff] Real   Q: [%s]", ss_real.str().c_str());
  ROS_INFO("[Joint Diff] Diff   Q: [%s]", ss_diff.str().c_str());
  ROS_INFO("---------------------------------------------------------");

  // 计算 FK 误差
  Eigen::Matrix4d real_pose;
  pinocchio_ik_->forwardKinematics(real_q, real_pose);

  double real_err = (real_pose.block<3, 1>(0, 3) - target_pose_eigen.block<3, 1>(0, 3)).norm();

  // 最终判断
  if (real_err < TARGET_TOLERANCE) {
    ROS_INFO(">>> SUCCESS: Target Reached! Final Precision: %.5f m", real_err);
    return true;
  } else {
    // 即使误差略大，动作也已经执行完毕，通常应视为"执行完成"但带有警告
    ROS_WARN(">>> FINISHED: Motion done but error %.5fm > threshold (Gravity Sag?).", real_err);
    return true;
  }
}
void ArmController::imuCallback(const sensor_msgs::Imu::ConstPtr& imu) {
  // tf2::Vector3 gravity_world(0, 0, -9.81);
  // tf2::Quaternion orientation;
  // tf2::fromMsg(imu->orientation, orientation);
  // tf2::Vector3 gravity_imu = tf2::quatRotate(orientation.inverse(),
  // gravity_world); arm_model_->_gravity[0] = gravity_imu.x();
  // arm_model_->_gravity[1] = gravity_imu.y();
  // arm_model_->_gravity[2] = gravity_imu.z();
}

void ArmController::executeProcessCallback(const std_msgs::Float64::ConstPtr& msg) {
  execute_process_ = msg->data;
  // ROS_INFO("[ExecuteProcess] Control signal updated: %.1f (%s)",
  //          execute_process_, execute_process_ >= 1.0 ? "ENABLED" :
  //          "DISABLED");
}

std::vector<geometry_msgs::PoseStamped> ArmController::sortPosesByDistanceToPoint(const std::vector<geometry_msgs::PoseStamped>& poses, const Eigen::Vector3d& reference_point, const Eigen::Vector3d& line_direction,
                                                                                  double target_distance) {
  // 复制输入列表
  std::vector<geometry_msgs::PoseStamped> sorted_poses = poses;

  // 按照到参考点的距离与 target_distance 的差值排序
  // 差值越小，说明距离越接近 target_distance
  std::sort(sorted_poses.begin(), sorted_poses.end(), [&reference_point, target_distance](const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b) {
    // 将 pose 转换为 Eigen 向量
    Eigen::Vector3d point_a(a.pose.position.x, a.pose.position.y, a.pose.position.z);
    Eigen::Vector3d point_b(b.pose.position.x, b.pose.position.y, b.pose.position.z);

    // 计算到参考点的距离
    double dist_a = (point_a - reference_point).norm();
    double dist_b = (point_b - reference_point).norm();

    // 计算与目标距离的差值（绝对值）
    double diff_a = std::abs(dist_a - target_distance);
    double diff_b = std::abs(dist_b - target_distance);

    // 差值较小的排在前面（即距离更接近 target_distance 的排在前面）
    return diff_a < diff_b;
  });

  ROS_INFO(
      "[SortPoses] Sorted %zu poses by distance to reference point (%.3f, "
      "%.3f, %.3f)",
      sorted_poses.size(), reference_point.x(), reference_point.y(), reference_point.z());
  ROS_INFO("[SortPoses] Target distance from reference: %.3f m", target_distance);

  // 输出前几个点的距离信息
  for (size_t i = 0; i < std::min(size_t(5), sorted_poses.size()); ++i) {
    Eigen::Vector3d point(sorted_poses[i].pose.position.x, sorted_poses[i].pose.position.y, sorted_poses[i].pose.position.z);
    double dist_to_ref = (point - reference_point).norm();
    double diff = std::abs(dist_to_ref - target_distance);

    ROS_INFO(
        "  [%zu] Pose at (%.3f, %.3f, %.3f), distance to ref: %.3f m, "
        "diff from target: %.3f m",
        i, point.x(), point.y(), point.z(), dist_to_ref, diff);
  }

  return sorted_poses;
}

bool ArmController::goToDefaultPoint() {
  ROS_INFO("[GoToDefaultPoint] Moving to default point for safe transition...");

  // 调用 planToDefaultServer
  arm_controller_srvs::PlanToDefault::Request req;
  arm_controller_srvs::PlanToDefault::Response res;
  req.plan_to_default = true;

  bool success = planToDefaultServer(req, res);

  if (!success || !res.call_success) {
    ROS_ERROR("[GoToDefaultPoint] Failed to plan to default point");
    return false;
  }

  // 等待到达默认点
  ros::Rate rate(10);  // 10Hz
  int timeout_count = 0;
  const int max_timeout = 100;  // 10秒超时

  while (ros::ok() && arm_control_fsm_ != ArmControlFsm::Arrived && timeout_count < max_timeout) {
    rate.sleep();
    timeout_count++;
  }

  if (timeout_count >= max_timeout) {
    ROS_WARN("[GoToDefaultPoint] Timeout waiting for arrival at default point");
    return false;
  }

  ROS_INFO("[GoToDefaultPoint] Arrived at default point");
  ros::Duration(0.5).sleep();  // 稍微停顿
  return true;
}

bool ArmController::executeMotionToTarget(const geometry_msgs::Pose& target_pose, double pitch, double roll, double timeout_seconds) {
  // ROS_INFO("[ExecuteMotionToTarget] Starting motion to target position:
  // (%.3f, %.3f, %.3f)",
  //          target_pose.position.x, target_pose.position.y,
  //          target_pose.position.z);

  // 1. 规划到目标位姿
  // 移动加爪
  ros::Rate rate(50);  // 10Hz
  double gripper_pos = pitch;

  if (gripper_pos < -0.85 || gripper_pos > 0.0) {
    ROS_WARN(
        "[executeMotionToTarget] Gripper position %.3f is out of "
        "typical range (0.0 to -0.85)",
        gripper_pos);
  }
  double curr_gripper_pos = low_state_.getGripperQ();
  for (int i{0}; i < 50; ++i) {
    gripper_goal_ = curr_gripper_pos + static_cast<double>(i) / 50 * (pitch - curr_gripper_pos);
    rate.sleep();
  }
  if (roll < -3.14 || roll > 3.14) {
    ROS_ERROR(
        "[executeMotionToTarget] Invalid Joint6 position: %.3f rad "
        "(valid range: -3.14 to 3.14)",
        roll);
    return false;
  }

  bool success_plan = planToTargetPose(target_pose, roll, false);

  if (!success_plan) {
    // ROS_ERROR("[ExecuteMotionToTarget] Failed to plan motion to target
    // pose");
    return false;
  }

  return true;
}

std::vector<Eigen::Vector3d> Line3D::samplePoints(double t_start, double t_end, int num_samples) const {
  std::vector<Eigen::Vector3d> samples;
  if (num_samples <= 0) {
    return samples;
  }

  if (num_samples == 1) {
    samples.push_back(point + t_start * direction);
    return samples;
  }

  double dt = (t_end - t_start) / (num_samples - 1);
  for (int i = 0; i < num_samples; ++i) {
    double t = t_start + i * dt;
    samples.push_back(point + t * direction);
  }
  return samples;
}

Line3D ArmController::rotateLine(const Line3D& line, const Eigen::Vector3d& rotation_center, const Eigen::Vector3d& rotation_axis, double angle_rad) const {
  Line3D rotated_line;

  // 使用 Eigen 的 AngleAxis 创建旋转矩阵
  Eigen::AngleAxisd rotation(angle_rad, rotation_axis.normalized());
  Eigen::Matrix3d R = rotation.toRotationMatrix();

  // 旋转直线上的点（相对于旋转中心）
  Eigen::Vector3d relative_point = line.point - rotation_center;
  rotated_line.point = R * relative_point + rotation_center;

  // 旋转方向向量
  rotated_line.direction = (R * line.direction).normalized();

  return rotated_line;
}

Line3D ArmController::translateLineGeometry(const Line3D& line, const Eigen::Vector3d& direction, double distance) const {
  Line3D translated_line;

  // 计算平移向量：方向单位化 * 距离
  Eigen::Vector3d translation = direction.normalized() * distance;

  // 平移直线上的点
  translated_line.point = line.point + translation;

  // 保持方向向量不变
  translated_line.direction = line.direction;

  return translated_line;
}

std::vector<geometry_msgs::PoseStamped> ArmController::sampleAndCheckReachability(Line3D& line, double t_start, double t_end, int num_samples, double& pitch, double& roll, geometry_msgs::Pose& debug_camera_poses) {
  std::vector<geometry_msgs::PoseStamped> reachable_poses;
  // 【新增】用于收集这一轮采样中所有计算出的相机全局位姿
  geometry_msgs::PoseArray debug_camera_poses_msg;
  debug_camera_poses_msg.header.frame_id = "gripperMover";  // gripperMover
  debug_camera_poses_msg.header.stamp = ros::Time::now();
  // 初始化输出参数
  pitch = 0.0;
  roll = 0.0;
  bool first_second_try = true, success_second_try = false;

  // 检查是否有逆运动学解
  arm_controller_srvs::Plan::Request req;
  // arm_controller_srvs::Plan::Response res;
  // 在直线上采样点
  std::vector<Eigen::Vector3d> sampled_points = line.samplePoints(t_start, t_end, num_samples);

  std::cout << "[Sample & Check] Sampling " << num_samples << " points on line from t=" << t_start << " to t=" << t_end << std::endl;

  for (size_t i = 0; i < sampled_points.size(); ++i) {
    // 1.不借助相机上的夹爪，直接喂给六轴机械臂姿态
    geometry_msgs::Pose pose = Line3DToPose(line);
    pose.position.x = sampled_points[i].x();
    pose.position.y = sampled_points[i].y();
    pose.position.z = sampled_points[i].z();
    req.target_pose = pose;
    if (IsPlan(req.target_pose)) {
      first_second_try = true;
      std::cout << "  Point_First_Try[" << i << "]: (" << req.target_pose.position.x << ", " << req.target_pose.position.y << ", " << req.target_pose.position.z << ", " << req.target_pose.orientation.x << ", "
                << req.target_pose.orientation.y << ", " << req.target_pose.orientation.z << ", " << req.target_pose.orientation.w << ") is OK" << std::endl;
      // pitch = -0.7854;
      // roll = 0.0;
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = ros::Time::now();
      pose_stamped.header.frame_id = "link00";
      pose_stamped.pose = req.target_pose;
      reachable_poses.push_back(pose_stamped);
      std::cout << "[First_try]Calculated camera orientation: pitch=" << pitch << ", roll=" << roll << std::endl;
      continue;
    }
  }
  std::cout << "  Total " << reachable_poses.size() << " reachable poses found." << std::endl;
  return reachable_poses;
}

Line3D ArmController::translateLine(const Line3D& line, const Eigen::Vector3d& direction, double distance, double t_start, double t_end, int num_samples, std::vector<geometry_msgs::PoseStamped>& reachable_poses,
                                    double& pitch, double& roll) {
  std::cout << "[Line Translation] Generating " << num_samples << " translated lines in direction: " << direction.transpose() << " with distance: " << distance << " m" << std::endl;

  // 1. 纯几何变换：平移直线
  Line3D translated_line = translateLineGeometry(line, direction, distance);

  // 2. 采样并检测可达性
  geometry_msgs::Pose debug_camera_poses_demo;
  reachable_poses = sampleAndCheckReachability(translated_line, t_start, t_end, num_samples, pitch, roll, debug_camera_poses_demo);

  // 输出成功信息
  std::cout << "translateline,distance:" << distance << ": SUCCESS\n" << std::endl;

  return translated_line;
}

bool ArmController::calculateCameraOrientation(const Eigen::Vector3d& camera_direction, double& pitch, double& roll, double& yaw, geometry_msgs::Pose& out_cam_pose) const {
  // 归一化输入方向
  Eigen::Vector3d d = camera_direction.normalized();
  // 显式初始化位置
  out_cam_pose.position.x = 0.0;
  out_cam_pose.position.y = 0.0;
  out_cam_pose.position.z = 0.0;
  // 显式初始化四元数（重要！w必须为1.0代表无旋转）
  out_cam_pose.orientation.x = 0.0;
  out_cam_pose.orientation.y = 0.0;
  out_cam_pose.orientation.z = 0.0;
  out_cam_pose.orientation.w = 1.0;
  // =============================================================================
  // 暴力搜索方法：遍历所有 pitch 和 roll 组合
  // =============================================================================

  // 相机相对末端的固定位置偏移（从 URDF）
  const Eigen::Vector3d CAM_POS_OFFSET(0.0389, 0, -0.0389);
  const double CAM_PITCH_OFFSET = 0.7854;       // 45° 相机固定姿态偏移
  const double radius = CAM_POS_OFFSET.norm();  // 相机到夹爪的距离
  // 步骤 1: 计算目标坐标（单位向量 × 半径）
  Eigen::Vector3d target_pos = d * radius;

  // 步骤 2: 暴力搜索最佳 pitch 和 roll
  double best_pitch = 0.0, best_roll = 0.0;
  double min_error = 1e10;
  double epsilon = 1e-2;  // 遍历误差阈值

  // 搜索精度：1度
  const double step = 1.0 * M_PI / 180.0;
  const double pitch_range[] = {-M_PI / 2, 0.0};
  const double roll_range[] = {-M_PI / 2, M_PI / 2};

  // 遍历 pitch
  for (double p = pitch_range[0]; p <= pitch_range[1] + epsilon; p += step) {
    // 遍历 roll
    for (double r = roll_range[0]; r <= roll_range[1] + epsilon; r += step) {
      // 完整旋转矩阵：R = Rz(yaw) * Ry(pitch) * Rx(roll)
      Eigen::Matrix3d full_rotation = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *  // 固定 Yaw
                                       Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *    // 搜索 Pitch
                                       Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))     // 搜索 Roll
                                          .toRotationMatrix();

      // 计算相机位置
      Eigen::Vector3d cam_pos = full_rotation * CAM_POS_OFFSET;

      // 计算误差
      double error = (cam_pos - target_pos).norm();

      // 更新最佳解
      if (error < min_error) {
        min_error = error;
        best_pitch = p;
        best_roll = r;
      }
    }
  }

  pitch = best_pitch;
  roll = best_roll;

  // 重建最佳旋转矩阵 (用于计算位置和可视化姿态)
  Eigen::Matrix3d final_rotation = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(best_pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(best_roll, Eigen::Vector3d::UnitX())).toRotationMatrix();

  Eigen::Vector3d actual_cam_pos = final_rotation * CAM_POS_OFFSET;
  double final_error = (actual_cam_pos - target_pos).norm();

  if (final_error > 0.003) {  // 3mm 误差
    std::cout << "[Camera Position - Brute Force Search]" << std::endl;
    std::cout << "\n  [Warning] Large error! Target may not be achievable." << std::endl;
    return false;
  } else {
    // 1. 填充位置 (相对于夹爪的固定偏移)
    // out_cam_pose.header.frame_id = "link00";  // gripperMover
    // out_cam_pose.header.stamp = ros::Time::now();
    out_cam_pose.position.x = actual_cam_pos.x();  // target_pos.x()
    out_cam_pose.position.y = actual_cam_pos.y();  // target_pos.y()
    out_cam_pose.position.z = actual_cam_pos.z();  // target_pos.z()

    // 2. 填充姿态 (相对于夹爪的旋转)
    Eigen::Quaterniond q_final(final_rotation);
    q_final.normalize();
    out_cam_pose.orientation.x = q_final.x();
    out_cam_pose.orientation.y = q_final.y();
    out_cam_pose.orientation.z = q_final.z();
    out_cam_pose.orientation.w = q_final.w();
    // 输出结果
    std::cout << "[Camera Position - Brute Force Search]" << std::endl;
    std::cout << "  Target position:     " << target_pos.transpose() << std::endl;
    std::cout << "  Calculated position: " << actual_cam_pos.transpose() << std::endl;
    std::cout << "  Error: " << final_error << " m" << "||" << min_error << " m" << std::endl;
    std::cout << "  => End-effector Pitch: " << (pitch * 180.0 / M_PI) << " deg"
              << "==" << pitch << std::endl;
    std::cout << "  => End-effector Roll:  " << (roll * 180.0 / M_PI) << " deg"
              << "==" << roll << std::endl;
    // std::cout << "  [DEBUG] Final Output Pose ->"
    //           << " Angel: [Pitch:" << pitch << ", Roll:" << roll << "]"
    //           << " Pos: [" << out_cam_pose.position.x << ", " << out_cam_pose.position.y << ", " << out_cam_pose.position.z << "]"
    //           << " Ori: [" << out_cam_pose.orientation.x << ", " << out_cam_pose.orientation.y << ", " << out_cam_pose.orientation.z << ", " << out_cam_pose.orientation.w << "]" << std::endl;
    return true;
  }

  return true;
}

std::vector<std::pair<Line3D, std::vector<Eigen::Vector3d>>> ArmController::generateRotatedLinesWithSamples(const Line3D& line, const Eigen::Vector3d& rotation_center, const Eigen::Vector3d& rotation_axis, int num_rotations,
                                                                                                            double angle_step, double t_start, double t_end, int num_samples,
                                                                                                            std::vector<geometry_msgs::PoseStamped>& reachable_poses, double& pitch, double& roll,
                                                                                                            ros::Publisher* line_publisher, int viz_points) {
  std::vector<std::pair<Line3D, std::vector<Eigen::Vector3d>>> results;

  // 清空输出参数
  reachable_poses.clear();
  pitch = 0.0;
  roll = 0.0;

  std::cout << "[Line Rotation] Generating " << num_rotations << " rotated lines around axis: " << rotation_axis.transpose() << " with angle step: " << (angle_step * 180.0 / M_PI) << " deg" << std::endl;

  for (int i = 1; i <= num_rotations; i++) {
    double angle = i * angle_step;

    // 1. 纯几何变换：旋转直线
    Line3D rotated_line = rotateLine(line, rotation_center, rotation_axis, angle);

    // 2. 采样并检测可达性
    geometry_msgs::Pose debug_camera_poses_demo;
    std::vector<geometry_msgs::PoseStamped> current_reachable_poses = sampleAndCheckReachability(rotated_line, t_start, t_end, num_samples, pitch, roll, debug_camera_poses_demo);

    // 将当前旋转角度的可达点添加到总列表
    reachable_poses.insert(reachable_poses.end(), current_reachable_poses.begin(), current_reachable_poses.end());

    // 获取采样点用于返回结果
    std::vector<Eigen::Vector3d> sampled_points = rotated_line.samplePoints(t_start, t_end, num_samples);

    // 保存结果
    results.push_back(std::make_pair(rotated_line, sampled_points));

    // 输出调试信息
    std::cout << "  [" << i << "] Angle: " << (angle * 180.0 / M_PI) << " deg, "
              << "Line point: " << rotated_line.point.transpose() << ", "
              << "\nDirection: " << rotated_line.direction.transpose() << ", "
              << "\nSamples: " << sampled_points.size() << std::endl;

    // 输出可达点数量
    std::cout << "  Total " << reachable_poses.size() << " reachable poses found." << std::endl;

    // 输出成功信息
    std::cout << "rotated_line,angle_step:" << angle_step << ": SUCCESS\n" << std::endl;

    // 如果提供了发布者，则发布直线可视化
    if (line_publisher != nullptr) {
      geometry_msgs::PoseArray line_msg = createLineVisualization(rotated_line, t_start, t_end, viz_points);
      line_publisher->publish(line_msg);
      std::cout << "  [VIZ] Published line visualization (" << reachable_poses.size() << " reachable poses)" << std::endl;
    }
  }

  return results;
}

bool ArmController::zedLinkToLink00Server(arm_controller_srvs::zedlinktolink00::Request& req, arm_controller_srvs::zedlinktolink00::Response& res) {
  res.call_success = false;

  if (req.enable) {
    ROS_INFO("[ZedLinkToLink00] Querying TF transform: link00 -> estimated_object");

    try {
      // 查询 TF 变换：从 link00 到 estimated_object
      geometry_msgs::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_.lookupTransform("link00",            // 目标坐标系（相对于这个坐标系表示）
                                                     "estimated_object",  // 源坐标系（要查询的坐标系）
                                                     ros::Time(0),        // 获取最新的变换
                                                     ros::Duration(1.0)   // 超时时间：1秒
      );

      // 提取并输出坐标信息
      double pos_x = transform_stamped.transform.translation.x;
      double pos_y = transform_stamped.transform.translation.y;
      double pos_z = transform_stamped.transform.translation.z;
      double ori_x = transform_stamped.transform.rotation.x;
      double ori_y = transform_stamped.transform.rotation.y;
      double ori_z = transform_stamped.transform.rotation.z;
      double ori_w = transform_stamped.transform.rotation.w;

      ROS_INFO("[ZedLinkToLink00] Transform found:");
      ROS_INFO("  Position: x=%.4f, y=%.4f, z=%.4f", pos_x, pos_y, pos_z);
      ROS_INFO("  Orientation (quaternion): x=%.4f, y=%.4f, z=%.4f, w=%.4f", ori_x, ori_y, ori_z, ori_w);

      // 转换为 RPY 角度（可选）
      tf2::Quaternion quat(ori_x, ori_y, ori_z, ori_w);
      tf2::Matrix3x3 mat(quat);
      double roll, pitch, yaw;
      mat.getRPY(roll, pitch, yaw);

      ROS_INFO(
          "  Orientation (RPY): roll=%.4f (%.2f°), pitch=%.4f (%.2f°), "
          "yaw=%.4f (%.2f°)",
          roll, roll * 180.0 / M_PI, pitch, pitch * 180.0 / M_PI, yaw, yaw * 180.0 / M_PI);

      // 填充响应中的 Pose 信息
      res.transformed_pose.position.x = pos_x;
      res.transformed_pose.position.y = pos_y;
      res.transformed_pose.position.z = pos_z;
      res.transformed_pose.orientation.x = ori_x;
      res.transformed_pose.orientation.y = ori_y;
      res.transformed_pose.orientation.z = ori_z;
      res.transformed_pose.orientation.w = ori_w;

      res.call_success = true;

    } catch (tf2::TransformException& ex) {
      ROS_ERROR("[ZedLinkToLink00] TF lookup failed: %s", ex.what());
      ROS_ERROR("  Make sure both 'link00' and 'estimated_object' frames exist");
      ROS_ERROR(
          "  You can check available frames with: rosrun tf tf_echo "
          "link00 estimated_object");
      res.call_success = false;
    }

  } else {
    ROS_INFO("[ZedLinkToLink00] Service disabled (enable=false)");
    res.call_success = true;
  }

  return true;
}

/**
 * @brief line3d to PoseStamped.
 */
geometry_msgs::PoseStamped ArmController::Line3DToPoseStamped(Line3D& line3d_posestamped) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = "link00";
  pose_msg.header.stamp = ros::Time::now();

  pose_msg.pose.position.x = line3d_posestamped.point.x();
  pose_msg.pose.position.y = line3d_posestamped.point.y();
  pose_msg.pose.position.z = line3d_posestamped.point.z();

  Eigen::Vector3d default_dir = Eigen::Vector3d::UnitX();
  Eigen::Vector3d target_dir = line3d_posestamped.direction.normalized();

  Eigen::Quaterniond q_rot;
  q_rot.setFromTwoVectors(default_dir, target_dir);

  pose_msg.pose.orientation.x = q_rot.x();
  pose_msg.pose.orientation.y = q_rot.y();
  pose_msg.pose.orientation.z = q_rot.z();
  pose_msg.pose.orientation.w = q_rot.w();

  return pose_msg;
}

/**
 * @brief line3d to Pose.
 */
geometry_msgs::Pose ArmController::Line3DToPose(Line3D& line3d_pose) {
  geometry_msgs::Pose pose_msg;
  // pose_msg.header.frame_id = "link00";
  // pose_msg.header.stamp = ros::Time::now();

  pose_msg.position.x = line3d_pose.point.x();
  pose_msg.position.y = line3d_pose.point.y();
  pose_msg.position.z = line3d_pose.point.z();

  Eigen::Vector3d default_dir = Eigen::Vector3d::UnitX();
  Eigen::Vector3d target_dir = line3d_pose.direction.normalized();

  Eigen::Quaterniond q_rot;
  q_rot.setFromTwoVectors(default_dir, target_dir);

  pose_msg.orientation.x = q_rot.x();
  pose_msg.orientation.y = q_rot.y();
  pose_msg.orientation.z = q_rot.z();
  pose_msg.orientation.w = q_rot.w();

  return pose_msg;
}

}  // namespace arm_controller
