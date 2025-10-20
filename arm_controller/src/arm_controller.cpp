#include "arm_controller/arm_controller.h"

#include "arm_controller/geometry_utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace arm_controller {

ArmController::ArmController(const ros::NodeHandle& nh) : nh_(nh), tf_listener_(tf_buffer_) {
  arm_api_ = std::make_unique<ArmApi>();
  arm_model_ = std::make_unique<Z1ArmModel>();
  
  // 修改关节限制：限制 Joint[2] 最小角度以防打到相机
  arm_model_->setJointQMin(2, -1.9);  // Joint[2] (index 2) 最小角度 -2.0 rad (-115°)
  // arm_model_->setJointQMin(2, -1.5);  // Joint[3] (index 3) 最小角度 -2.0 rad (-115°)
  ROS_INFO("Joint[2] min limit set to: -1.5 rad (-86.2°)");
  // ROS_INFO("Joint[3] min limit set to: -2.0 rad (-115 deg)");
  
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
  low_cmd_.setGripperGain();  // 使用默认增益
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

    //test - 从 ROS 参数服务器读取配置
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

    //将line_pitch, line_yaw, line_roll转换为笛卡尔坐标系
    line_pitch = -line_pitch_link00;
    line_yaw = line_yaw_link00;
    line_roll = line_roll_link00-1.5708;
    std::cout << "Line direction: " << line_pitch << ", " << line_yaw << ", " << line_roll << std::endl;
    // 计算方向向量(假设沿着姿态的X轴方向)
    origin_direction_x = cos(line_yaw) * cos(line_pitch);
    origin_direction_y = sin(line_yaw) * cos(line_pitch);
    origin_direction_z = sin(line_pitch);
    std::cout << "Origin direction: " << origin_direction_x << ", " << origin_direction_y << ", " << origin_direction_z << std::endl;

    //将origin_direction在XOZ平面内旋转90度得到rotation_axis
    rotation_axis_x = -origin_direction_z;
    rotation_axis_y = origin_direction_y;
    rotation_axis_z = origin_direction_x;
    std::cout << "Rotation axis: " << rotation_axis_x << ", " << rotation_axis_y << ", " << rotation_axis_z << std::endl;

    // 计算平移轴：通过 pitch、yaw、roll 旋转 (0, 1, 0) 向量
    // 使用 Eigen 的 AngleAxis 构建旋转矩阵
    Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(line_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d R_pitch = Eigen::AngleAxisd(line_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d R_roll = Eigen::AngleAxisd(line_roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    // 组合旋转矩阵：R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Eigen::Matrix3d R_total = R_yaw * R_pitch * R_roll;
    // 对 (0, 1, 0) 向量进行旋转
    Eigen::Vector3d original_y_axis(0.0, 1.0, 0.0);
    Eigen::Vector3d translation_axis_vec = R_total * original_y_axis;   
    std::cout << "Translation axis: " << translation_axis_vec[0] << ", " << translation_axis_vec[1] << ", " << translation_axis_vec[2] << std::endl;

    // 创建原始直线
    Line3D original_line;
    original_line.point = Eigen::Vector3d(line_x, line_y, line_z);
    original_line.direction = Eigen::Vector3d(origin_direction_x, origin_direction_y, origin_direction_z).normalized();

    // 定义旋转参数
    Eigen::Vector3d rotation_center(center_x, center_y, center_z);
    Eigen::Vector3d rotation_axis(rotation_axis_x, rotation_axis_y, rotation_axis_z);  // 绕 Z 轴旋转
    double rotation_angle_rad = rotation_angle_deg * M_PI / 180.0;

    // 方法: 批量旋转并采样
    //原直线
    // std::cout<<"------------MID LINE------------"<<std::endl;
    // double angle_step_rad = 0.0;
    // auto results = generateRotatedLinesWithSamples(
    //     original_line,
    //     rotation_center,
    //     rotation_axis,
    //     num_rotations,
    //     angle_step_rad,
    //     sample_start, sample_end,
    //     num_samples
    // );

    // //正转angle_step_deg度
    // std::cout<<"------------HALF LINE------------"<<std::endl;
    // double angle_step_rad_1 = angle_step_deg * M_PI / 180.0;
    // auto results_1 = generateRotatedLinesWithSamples(
    //     original_line,
    //     rotation_center,
    //     rotation_axis,
    //     num_rotations,
    //     angle_step_rad_1,
    //     sample_start, sample_end,
    //     num_samples
    // );


    // //反转angle_step_deg度
    // std::cout<<"------------HALF2 LINE------------"<<std::endl;
    // double angle_step_rad_2 = (360.0-angle_step_deg) * M_PI / 180.0;
    // auto results_2 = generateRotatedLinesWithSamples(
    //     original_line,
    //     rotation_center,
    //     rotation_axis,
    //     num_rotations,
    //     angle_step_rad_2,
    //     sample_start, sample_end,
    //     num_samples
    // );

    // //平移distance_1
    // std::cout<<"------------OUT LINE------------"<<std::endl;
    // double distance_1 = 0.3;
    // auto results_3 = translateLine(
    //     original_line, translation_axis_vec, 
    //     distance_1, 
    //     sample_start, sample_end, 
    //     num_samples);

    // //平移distance_2
    // std::cout<<"------------OUT2 LINE------------"<<std::endl;
    // double distance_2 = -0.3;
    // auto results_4 = translateLine(
    //     original_line, translation_axis_vec, 
    //     distance_2, 
    //     sample_start, sample_end, 
    //     num_samples);
    
    // auto start_time = std::chrono::system_clock::now();
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
    }
  });
}

void ArmController::initSubsAndPubs() {
  joint_state_msgs_.header.frame_id = "link00";
  joint_state_msgs_.position.assign(6, 0);
  joint_state_msgs_.effort.assign(6, 0);
  joint_state_msgs_.velocity.assign(6, 0);
  joint_state_msgs_.name = arm_joint_names_;
  cmd_joint_state_msgs_.header.frame_id = "link00";
  cmd_joint_state_msgs_.position.assign(6, 0);
  cmd_joint_state_msgs_.effort.assign(6, 0);
  cmd_joint_state_msgs_.velocity.assign(6, 0);
  cmd_joint_state_msgs_.name = arm_joint_names_;
  ee_pose_msg_.header.frame_id = "link00";
  arm_joint_states_pub_ =
      nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
  arm_cmd_joint_states_pub_ =
      nh_.advertise<sensor_msgs::JointState>("/cmd_joint_states", 1);
  ee_pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/end_effector_pose", 1);
  process_pub_ = nh_.advertise<std_msgs::Float64>("/execute_process", 1);
  center_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/arm_controller/center_point", 1);
  target_poses_pub_ = 
      nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/target_poses", 1);
  poses_out1_pub_ = 
      nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/poses_out1", 1);
  poses_out2_pub_ = 
      nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/poses_out2", 1);
  poses_mid_pub_ = 
      nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/poses_mid", 1);
  poses_half1_pub_ = 
      nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/poses_half1", 1);
  poses_half2_pub_ = 
      nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/poses_half2", 1);
  poses_mid_all_pub_ = 
      nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/poses_mid_all", 1);
  transformed_input_pub_ = 
      nh_.advertise<geometry_msgs::PoseStamped>("/arm_controller/transformed_input_pose", 1);
  camera_transformed_pose_pub_ = 
      nh_.advertise<geometry_msgs::PoseStamped>("/arm_controller/camera_transformed_pose", 1);
  // 初始化直线可视化发布器
  line_mid_pub_ = 
      nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/line_mid", 1);
  line_out1_pub_ = 
      nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/line_out1", 1);
  line_out2_pub_ = 
      nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/line_out2", 1);
  line_half1_pub_ = 
      nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/line_half1", 1);
  line_half2_pub_ = 
      nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/line_half2", 1);
  reference_points_pub_ = 
      nh_.advertise<geometry_msgs::PoseArray>("/arm_controller/reference_points", 1);
  imu_sub_ =
      nh_.subscribe("/aliengo/imu", 1, &ArmController::imuCallback, this);
}

void ArmController::initServers() {
  plan_server_ = nh_.advertiseService("plan", &ArmController::planServer, this);
  back2home_server_ = nh_.advertiseService(
      "back_to_home", &ArmController::back2HomeServer, this);
  plan_to_default_server_ = nh_.advertiseService(
      "plan_to_default", &ArmController::planToDefaultServer, this);
  check_pose_in_workspace_server_ = nh_.advertiseService(
      "check_pose_in_workspace", &ArmController::isInWorkspaceServer, this);
  search_plan_server_ = nh_.advertiseService(
      "search_plan", &ArmController::searchPlanServer, this);
  js_control_server_ = nh_.advertiseService(
      "joy_stick_control", &ArmController::jsControlServer, this);
  gripper_control_server_ = nh_.advertiseService(
      "gripper_control", &ArmController::gripperControlServer, this);
  plan_to_five_point_server_ = nh_.advertiseService(
      "plan_to_five_point", &ArmController::planToFivePointServer, this);
  plan_and_gripper_control_server_ = nh_.advertiseService(
      "plan_and_gripper_control", &ArmController::planAndGripperControlServer, this);
  get_goal_and_angle_server_ = nh_.advertiseService(
      "get_goal_and_angle", &ArmController::getGoalAndAngleServer, this);
  zed_link_to_link00_server_ = nh_.advertiseService(
      "zed_link_to_link00", &ArmController::zedLinkToLink00Server, this);
  camera_to_link00_server_ = nh_.advertiseService(
    "camera_to_link00", &ArmController::cameraToLink00Server, this);
  
  // 订阅执行控制信号（从机械臂控制器获取状态）
  execute_process_sub_ = nh_.subscribe<std_msgs::Float64>(
      "/arm_controller/execute_process", 1, &ArmController::executeProcessCallback, this);
  
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
    cmd_joint_state_msgs_.effort[i] =
        25.6 * low_cmd_.kp[i] * (low_cmd_.q[i] - low_state_.q[i]) +
        0.0128 * low_cmd_.kd[i] * (low_cmd_.dq[i] - low_state_.dq[i]) +
        low_cmd_.tau[i];
  }
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
  // std::cout << "Position Tau: [";
  // for (int i{0}; i < 6; ++i) {
  //   std::cout << 25.6 * low_cmd_.kp[i] * (low_cmd_.q[i] - low_state_.q[i])
  //             << ", ";
  // }
  // std::cout << "]" << std::endl;
  // std::cout << "Velocity Tau: [";
  // for (int i{0}; i < 6; ++i) {
  //   std::cout << 0.0128 * low_cmd_.kd[i] * (low_cmd_.dq[i] -
  //   low_state_.dq[i])
  //             << ", ";
  // }
  // std::cout << "]" << std::endl;
  // std::cout << "Tau: [";
  // for (int i{0}; i < 6; ++i) {
  //   std::cout << low_cmd_.tau[i] << ", ";
  // }
  // std::cout << "]" << std::endl;
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
      setControlCmd(0, 400.0, true);
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
      break;
    }
    case ArmControlFsm::Arrived: {
      arm_control_joint_vel_.setZero();
      setControlCmd(default_kp_, default_kd_);
      // 保持夹爪位置
      data_mutex_.lock();
      low_cmd_.setGripperQ(gripper_goal_);
      low_cmd_.setGripperQd(0.0);
      low_cmd_.setGripperGain(15.0,0.0);  // 设置夹爪增益
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
      low_cmd_.setGripperGain(15.0,0.0);  // 设置夹爪增益
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
  Eigen::Matrix4d current_pose =
      arm_model_->forwardKinematics(low_state_.getQ());
  Eigen::Matrix4d target_pose = ee_pose_goal_;
  Eigen::Matrix<double, 6, 1> target_joint_pos;
  Eigen::Matrix<double, 6, 1> twist_E = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Vector3d delta_rpy_E = Eigen::Vector3d::Zero(),
                  delta_pos_E = Eigen::Vector3d::Zero();
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
  target_pose.block<3, 3>(0, 0) =
      ee_pose_goal_.block<3, 3>(0, 0) * rpyToRot(delta_rpy_E);
  target_pose.block<3, 1>(0, 3) += current_pose.block<3, 3>(0, 0) * delta_pos_E;
  find_ik = arm_model_->inverseKinematics(target_pose, low_state_.getQ(),
                                          target_joint_pos, true);
  singular_pose = find_ik ? arm_model_->checkInSingularity(target_joint_pos)
                          : arm_model_->checkInSingularity(low_state_.getQ());
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
    arm_model_->solveQP(twist_E, low_state_.getQ(), arm_control_joint_vel_,
                        control_period_);
  } else {
    arm_control_joint_vel_.setZero();
  }
  setControlCmd(default_kp_, default_kd_, true);
}

void ArmController::lazyPlan(
    const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& start,
    const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& goal,
    unsigned long ticks) {
  process_ = 0.0;
  joint_pos_trajectory_.clear();
  joint_vel_trajectory_.clear();
  joint_pos_trajectory_.push_back(start);
  joint_vel_trajectory_.push_back(Eigen::Matrix<double, 6, 1>::Zero());
  joint_interp_fn_.setPolyInterpolationKernel(ticks * control_period_, start,
                                              goal, ticks);
  for (long unsigned int i{1}; i < ticks; ++i) {
    joint_pos_trajectory_.push_back(joint_interp_fn_.step());
    joint_vel_trajectory_.push_back(joint_interp_fn_.d(i * control_period_));
  }
  joint_pos_trajectory_.push_back(goal);
  joint_vel_trajectory_.push_back(Eigen::Matrix<double, 6, 1>::Zero());
}

void ArmController::setControlCmd(double kp, double kd,
                                  bool enable_feedfoward_control) {
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
    tau_bias = arm_model_->inverseDynamics(
        low_state_.getQ(), low_state_.getQd(),
        Eigen::Matrix<double, 6, 1>::Zero(), payload);
    // std::cout << tau_bias.transpose() << std::endl;
    low_cmd_.setTau(tau_bias);
  } else {
    low_cmd_.setZeroTau();
  }
}

void ArmController::setControlCmd(std::vector<double> kp,
                                  std::vector<double> kd,
                                  bool enable_feedfoward_control) {
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
    tau_bias = arm_model_->inverseDynamics(
        arm_control_joint_pos_, arm_control_joint_vel_,
        Eigen::Matrix<double, 6, 1>::Zero(), payload);
    low_cmd_.setTau(tau_bias);
  } else {
    low_cmd_.setZeroTau();
  }
}

void ArmController::checkArmMotorSafe() {
  for (long unsigned int i = 0; i < low_state_.errorstate.size(); ++i) {
    uint8_t arm_state = low_state_.errorstate[i];
    if (arm_state == 0x01 || arm_state == 0x02 || arm_state == 0x04 ||
        arm_state == 0x20) {
      arm_motor_safe_ = false;
      std::cout << "Arm motor is not safe! Set arm invalid!" << std::endl;
      setArmControlFsm(ArmControlFsm::Invalid);
    }
  }
}

bool ArmController::isInWorkspaceServer(
    arm_controller_srvs::CheckPoseInWorkspace::Request& req,
    arm_controller_srvs::CheckPoseInWorkspace::Response& res) {
  Eigen::Matrix4d target_pose, camera_target_pose;
  Eigen::Matrix<double, 6, 1> target_joint_pos;
  arm_controller::geometryMsgsPose2Pose(req.target_pose, camera_target_pose);
  target_pose = camera_target_pose;
  target_pose.block<3, 1>(0, 3) =
      camera_target_pose.block<3, 1>(0, 3) -
      camera_target_pose.block<3, 3>(0, 0) * kCameraPosBias_E_;
  res.is_in_workspace = arm_model_->inverseKinematics(
      target_pose, Eigen::Matrix<double, 6, 1>::Zero(), target_joint_pos, true);
  return true;
}

bool ArmController::planServer(arm_controller_srvs::Plan::Request& req,
                               arm_controller_srvs::Plan::Response& res) {
  res.call_success = false;
  // // 保存夹爪目标值
  // double gripper_goal = req.gripper_pos;
  if (arm_control_fsm_ == ArmControlFsm::Home ||
      arm_control_fsm_ == ArmControlFsm::Arrived) {
    Eigen::Matrix4d start_ee_pose =
        arm_model_->forwardKinematics(low_state_.getQ());
    Eigen::Matrix<double, 6, 1> start_joint_pos = low_state_.getQ();
    Eigen::Matrix4d camera_target_pose, target_pose;
    Eigen::Matrix<double, 6, 1> target_joint_pos;
    bool find_ik{false};
    arm_controller::geometryMsgsPose2Pose(req.target_pose, camera_target_pose);
    target_pose = camera_target_pose;
    // target_pose.block<3, 1>(0, 3) =
    //     camera_target_pose.block<3, 1>(0, 3) -
    //     camera_target_pose.block<3, 3>(0, 0) * kCameraPosBias_E_;
    find_ik = arm_model_->inverseKinematics(target_pose, start_joint_pos,
                                            target_joint_pos, true);
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
      plan_max_tick_ = static_cast<long unsigned int>(
          (ee_pose_goal_ - start_ee_pose).block<3, 1>(0, 3).norm() /
          average_move_speed_ / control_period_);
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

bool ArmController::searchPlanServer(arm_controller_srvs::Plan::Request& req,
                                     arm_controller_srvs::Plan::Response& res) {
  res.call_success = false;
  // // 保存夹爪目标值
  // double gripper_goal = req.gripper_pos;
  if (arm_control_fsm_ == ArmControlFsm::Home ||
      arm_control_fsm_ == ArmControlFsm::Arrived) {
    Eigen::Matrix4d start_ee_pose =
        arm_model_->forwardKinematics(low_state_.getQ());
    Eigen::Matrix<double, 6, 1> start_joint_pos = low_state_.getQ();
    Eigen::Matrix<double, 6, 1> target_joint_pos;
    bool find_ik{false};
    Eigen::Matrix4d camera_target_pose, search_pose, target_pose;
    arm_controller::geometryMsgsPose2Pose(req.target_pose, camera_target_pose);
    int max_search_num{30};
    Eigen::Vector3d search_start_pos_T{0.1, 0, 0}, search_interval{0.01, 0, 0};
    for (int i{1}; i <= max_search_num; ++i) {
      search_pose.setIdentity();
      search_pose.block<3, 1>(0, 3) =
          camera_target_pose.block<3, 3>(0, 0) *
              (search_start_pos_T + search_interval * i) +
          camera_target_pose.block<3, 1>(0, 3);
      search_pose.block<3, 1>(0, 0) = -camera_target_pose.block<3, 1>(0, 0);
      search_pose.block<3, 1>(0, 1) = -camera_target_pose.block<3, 1>(0, 1);
      target_pose = search_pose;
      target_pose.block<3, 1>(0, 3) =
          search_pose.block<3, 1>(0, 3) -
          search_pose.block<3, 3>(0, 0) * kCameraPosBias_E_;
      find_ik = arm_model_->inverseKinematics(target_pose, start_joint_pos,
                                              target_joint_pos, true);
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
      plan_max_tick_ = static_cast<long unsigned int>(
          (ee_pose_goal_ - start_ee_pose).block<3, 1>(0, 3).norm() /
          average_move_speed_ / control_period_);
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


bool ArmController::IsPlanServer(arm_controller_srvs::Plan::Request& req,
                                 arm_controller_srvs::Plan::Response& res) {
  res.call_success = false;
  
  if (arm_control_fsm_ == ArmControlFsm::Home ||
      arm_control_fsm_ == ArmControlFsm::Arrived) {
    Eigen::Matrix4d start_ee_pose =
        arm_model_->forwardKinematics(low_state_.getQ());
    Eigen::Matrix<double, 6, 1> start_joint_pos = low_state_.getQ();
    Eigen::Matrix4d camera_target_pose, target_pose;
    Eigen::Matrix<double, 6, 1> target_joint_pos;
    bool find_ik{false};
    
    arm_controller::geometryMsgsPose2Pose(req.target_pose, camera_target_pose);
    target_pose = camera_target_pose;
    target_pose.block<3, 1>(0, 3) =
        camera_target_pose.block<3, 1>(0, 3) -
        camera_target_pose.block<3, 3>(0, 0) * kCameraPosBias_E_;
    
    find_ik = arm_model_->inverseKinematics(target_pose, start_joint_pos,
                                            target_joint_pos, true);
    
    if (find_ik && target_joint_pos[2] < -1.5) {
      ROS_DEBUG("[IsPlanServer] Joint[2]=%.3f rad (%.1f deg) violates limit -1.5 rad",
                target_joint_pos[2], target_joint_pos[2] * 180.0 / M_PI);
      find_ik = false;  // 强制标记为失败
    }
    
    if (arm_motor_safe_ && find_ik) {
      res.call_success = true;
    }
  }
  return true;
}


bool ArmController::back2HomeServer(
    arm_controller_srvs::BackToHome::Request& req,
    arm_controller_srvs::BackToHome::Response& res) {
  res.call_success = false;

  arm_controller_srvs::PlanToDefault::Request plan_to_default_req;
  arm_controller_srvs::PlanToDefault::Response plan_to_default_res;

  planToDefaultServer(plan_to_default_req, plan_to_default_res);
  if (!plan_to_default_res.call_success) {
    return false;
  }
  Eigen::Matrix4d start_ee_pose =
      arm_model_->forwardKinematics(low_state_.getQ());
  Eigen::Matrix<double, 6, 1> start_joint_pos = low_state_.getQ();
  if (arm_motor_safe_) {
    ee_pose_goal_ = kEePoseHome_;
    arm_joint_goal_ = KJointHome_;
    plan_max_tick_ = static_cast<long unsigned int>(
        (kEePoseHome_ - start_ee_pose).block<3, 1>(0, 3).norm() /
        average_move_speed_ / control_period_);
    plan_max_tick_ = std::max(100uL, plan_max_tick_);
    // std::cout << "StartEEPose:\n"
    //           << start_ee_pose
    //           << "\nStartJointPos: " << start_joint_pos.transpose()
    //           << "\nHomePose:\n"
    //           << kEePoseHome_ << "\nKJointHome: " << KJointHome_
    //           << "\nPlanTicks: " << plan_max_tick_ << std::endl;
    lazyPlan(start_joint_pos, KJointHome_, plan_max_tick_);
    setArmControlFsm(ArmControlFsm::Back2Home);
    res.call_success = true;
  }
  return true;
}

bool ArmController::planToDefaultServer(
    arm_controller_srvs::PlanToDefault::Request& req,
    arm_controller_srvs::PlanToDefault::Response& res) {
  res.call_success = false;
  
  // 从 ROS 参数服务器读取默认目标位姿
  geometry_msgs::Pose default_target_pose;
  nh_.param("test/default_target_pose_x", default_target_pose.position.x, 0.5);
  nh_.param("test/default_target_pose_y", default_target_pose.position.y, 0.0);
  nh_.param("test/default_target_pose_z", default_target_pose.position.z, 0.15);
  nh_.param("test/default_target_pose_orientation_x", default_target_pose.orientation.x, 0.0);
  nh_.param("test/default_target_pose_orientation_y", default_target_pose.orientation.y, 0.0);
  nh_.param("test/default_target_pose_orientation_z", default_target_pose.orientation.z, 0.0);
  nh_.param("test/default_target_pose_orientation_w", default_target_pose.orientation.w, 1.0);
  
  ROS_INFO("PlanToDefault: Point (%.3f, %.3f, %.3f)", 
           default_target_pose.position.x, 
           default_target_pose.position.y, 
           default_target_pose.position.z);
  
  bool success_flag = executeMotionToTarget(default_target_pose,0.0,0.0,10);

  res.call_success = success_flag;
  return success_flag;
}

bool ArmController::jsControlServer(
    arm_controller_srvs::JoyStickControlRequest& req,
    arm_controller_srvs::JoyStickControlResponse& res) {
  res.call_success = false;
  if (req.enable && arm_control_fsm_ == ArmControlFsm::Arrived &&
      (!arm_model_->checkInSingularity(low_state_.getQ()))) {
    setArmControlFsm(ArmControlFsm::JoyStickControl);
    res.call_success = true;
  } else if (!req.enable &&
             arm_control_fsm_ == ArmControlFsm::JoyStickControl) {
    arm_controller_srvs::BackToHomeRequest reset_req;
    arm_controller_srvs::BackToHomeResponse reset_res;
    reset_req.back_to_home = true;
    back2HomeServer(reset_req, reset_res);
    res.call_success = reset_res.call_success;
  }
  return true;
}

bool ArmController::cameraToLink00Server(
  arm_controller_srvs::CameraToLink00::Request& req,
  arm_controller_srvs::CameraToLink00::Response& res) {
  try {
    // 1. 尝试获取相机坐标系到 link00 的变换
    geometry_msgs::TransformStamped transform_stamped;
    bool has_camera_frame = false;
    
    try {
      transform_stamped = tf_buffer_.lookupTransform(
          "link00", "camera_optical_frame", ros::Time(0), ros::Duration(0.5));
      has_camera_frame = true;
      ROS_INFO("[CameraToLink00] Found camera frame in TF tree");
    } catch (tf2::TransformException& ex) {
      ROS_WARN("[CameraToLink00] No camera frame found: %s. Using link00 frame directly.", ex.what());
      has_camera_frame = false;
    }

    // 2. 根据是否有 camera 坐标系来处理
    geometry_msgs::PoseStamped result_pose_stamped;
    result_pose_stamped.header.frame_id = "link00";
    result_pose_stamped.header.stamp = ros::Time::now();

    if (has_camera_frame) {
      // 有 camera 坐标系:创建相机坐标系下的偏移位姿并转换
      geometry_msgs::PoseStamped camera_offset_pose;
      camera_offset_pose.header.frame_id = "camera_optical_frame";
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

      ROS_INFO("[CameraToLink00] Offset (dx=%.3f, dy=%.3f) in camera frame -> "
              "Position (%.3f, %.3f, %.3f) in link00 frame (with TF transform)",
              req.dx, req.dy,
              res.target_pose.position.x,
              res.target_pose.position.y,
              res.target_pose.position.z);
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

      ROS_WARN("[CameraToLink00] No camera frame. Using offset (dx=%.3f, dy=%.3f) directly in link00 frame (no TF transform)",
              req.dx, req.dy);
    }

    // 3. 发布转换后的位姿到话题
    camera_transformed_pose_pub_.publish(result_pose_stamped);
    ROS_DEBUG("[CameraToLink00] Published transformed pose to /arm_controller/camera_transformed_pose");

    return true;
  } catch (const std::exception& e) {
    ROS_ERROR("[CameraToLink00] Exception: %s", e.what());
    return false;
  }
}


bool ArmController::gripperControlServer(
  arm_controller_srvs::GripperControl::Request& req,
  arm_controller_srvs::GripperControl::Response& res) {
  res.call_success = false;

  // 检查 Joint6 角度范围
  // if (req.joint6_pos < -3.14 || req.joint6_pos > 3.14) {
  //   std::cout << "[Gripper Control] Invalid Joint6 position: " << req.joint6_pos
  //             << " rad (valid range: -3.14 to 3.14)" << std::endl;
  //   return true;
  // }

  // // 如果不在 Arrived 或 PlanMove 状态，需要初始化机械臂位置
  // if (arm_control_fsm_ != ArmControlFsm::Arrived && 
  //     arm_control_fsm_ != ArmControlFsm::PlanMove) {
  //   arm_control_joint_pos_ = low_state_.getQ();  // 保持当前位置
  //   arm_control_joint_vel_.setZero();
  // }

  // // 设置夹爪目标
  // gripper_goal_ = req.gripper_pos;

  // // 设置 Joint6 目标
  // arm_control_joint_pos_[5] = req.joint6_pos;  // Joint6 是索引 5

  // std::cout << "[Gripper Control] Setting Joint6 to: " << req.joint6_pos 
  //           << " rad (" << (req.joint6_pos * 180.0 / 3.14159) << " deg), "
  //           << "Gripper to: " << gripper_goal_ << std::endl;

  res.call_success = false;
  // setArmControlFsm(ArmControlFsm::Arrived);
  return true;
}

bool ArmController::planAndGripperControlServer(
    arm_controller_srvs::planandgrippercontrol::Request& req,
    arm_controller_srvs::planandgrippercontrol::Response& res) {
  res.call_success = false;
  
  // 从 ROS 参数服务器读取默认目标位姿
  geometry_msgs::Pose planandgrippercontrol_target_pose=req.target_pose;
  float pitch = req.gripper_pos;
  float roll = req.joint6_pos;
  
  bool success_flag = executeMotionToTarget(planandgrippercontrol_target_pose,pitch,roll,10);
  
  return true;
}

bool ArmController::getGoalAndAngleServer(
    arm_controller_srvs::getgoalandangle::Request& req,
    arm_controller_srvs::getgoalandangle::Response& res) {
  
  ROS_INFO("[GetGoalAndAngle] Service called, computing target poses...");
  res.call_success = false;
  
  // 清空输出
  res.target_poses.clear();
  res.pitch_angles.clear();
  res.roll_angles.clear();
  res.pose_names.clear();
  
  //==========================================================================
  // 步骤1：读取参数和解析输入位姿
  //==========================================================================
  double line_x = req.target_pose.pose.position.x;
  double line_y = req.target_pose.pose.position.y;
  double line_z = req.target_pose.pose.position.z;
  
  // 四元数变换
  tf::Quaternion quat_input(
    req.target_pose.pose.orientation.x,
    req.target_pose.pose.orientation.y,
    req.target_pose.pose.orientation.z,
    req.target_pose.pose.orientation.w
  );
  
  tf::Quaternion rot_y_inv;
  rot_y_inv.setRotation(tf::Vector3(0, 1, 0), M_PI / 2.0);
  tf::Quaternion rot_x_inv;
  rot_x_inv.setRotation(tf::Vector3(1, 0, 0), M_PI / 2.0);
  tf::Quaternion rot_y_180;
  rot_y_180.setRotation(tf::Vector3(0, 1, 0), M_PI);
  tf::Quaternion quat = quat_input * rot_y_inv * rot_x_inv * rot_y_180;
  
  double line_roll_link00, line_pitch_link00, line_yaw_link00;
  tf::Matrix3x3 mat(quat);
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
  tf::Vector3 x_axis(1.0, 0.0, 0.0);
  tf::Vector3 rotated_direction = tf::quatRotate(quat, x_axis);
  double origin_direction_x = rotated_direction.x();
  double origin_direction_y = rotated_direction.y();
  double origin_direction_z = rotated_direction.z();
  
  double rotation_axis_x = -origin_direction_z;
  double rotation_axis_y = origin_direction_y;
  double rotation_axis_z = origin_direction_x;
  
  // 计算平移轴
  Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(line_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Matrix3d R_pitch = Eigen::AngleAxisd(line_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
  Eigen::Matrix3d R_roll = Eigen::AngleAxisd(line_roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
  Eigen::Matrix3d R_total = R_yaw * R_pitch * R_roll;
  Eigen::Vector3d original_y_axis(0.0, 1.0, 0.0);
  Eigen::Vector3d translation_axis_vec = R_total * original_y_axis;
  
  //==========================================================================
  // 步骤2：生成5条直线
  //==========================================================================
  Line3D original_line;
  original_line.point = Eigen::Vector3d(line_x, line_y, line_z);
  original_line.direction = Eigen::Vector3d(origin_direction_x, origin_direction_y, origin_direction_z).normalized();
  
  Eigen::Vector3d rotation_center(line_x, line_y, line_z);
  Eigen::Vector3d rotation_axis(rotation_axis_x, rotation_axis_y, rotation_axis_z);
  Eigen::Vector3d base_point = original_line.point;
  Eigen::Vector3d reference_point_mid = base_point + mid_offset_distance * original_line.direction;
  
  Line3D line_mid = original_line;
  double angle_step_rad_1 = angle_step_deg * M_PI / 180.0;
  Line3D line_half1 = rotateLine(original_line, rotation_center, rotation_axis, angle_step_rad_1);
  double angle_step_rad_2 = (360.0 - angle_step_deg) * M_PI / 180.0;
  Line3D line_half2 = rotateLine(original_line, rotation_center, rotation_axis, angle_step_rad_2);
  double distance_1 = 0.3;
  Line3D line_out1 = translateLineGeometry(original_line, translation_axis_vec, distance_1);
  double distance_2 = -0.3;
  Line3D line_out2 = translateLineGeometry(original_line, translation_axis_vec, distance_2);
  
  //==========================================================================
  // 步骤3：计算参考点并检查顺序
  //==========================================================================
  Eigen::Vector3d mid_direction = line_mid.direction.normalized();
  Eigen::Vector3d half_direction_1 = line_half1.direction.normalized();
  Eigen::Vector3d half_direction_2 = line_half2.direction.normalized();
  
  Eigen::Vector3d reference_point_out1 = base_point + translation_axis_vec * distance_1;
  Eigen::Vector3d reference_point_out2 = base_point + translation_axis_vec * distance_2;
  Eigen::Vector3d reference_point_half1 = base_point + half_offset_distance * half_direction_1;
  Eigen::Vector3d reference_point_half2 = base_point + half_offset_distance * half_direction_2;
  
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
  
  std::vector<geometry_msgs::PoseStamped> reachable_poses_mid = 
      sampleAndCheckReachability(line_mid_sampled, mid_sample_start, mid_sample_end, mid_num_samples, pitch_0, roll_0);
  
  std::vector<geometry_msgs::PoseStamped> reachable_poses_half1 = 
      sampleAndCheckReachability(line_half1_sampled, sample_start, sample_end, num_samples, pitch_half1, roll_half1);
  
  std::vector<geometry_msgs::PoseStamped> reachable_poses_half2 = 
      sampleAndCheckReachability(line_half2_sampled, sample_start, sample_end, num_samples, pitch_half2, roll_half2);
  
  std::vector<geometry_msgs::PoseStamped> reachable_poses_out1 = 
      sampleAndCheckReachability(line_out1_sampled, sample_start, sample_end, num_samples, pitch_1, roll_1);
  
  std::vector<geometry_msgs::PoseStamped> reachable_poses_out2 = 
      sampleAndCheckReachability(line_out2_sampled, sample_start, sample_end, num_samples, pitch_2, roll_2);
  
  ROS_INFO("[GetGoalAndAngle] Reachable poses: MID=%zu, HALF1=%zu, HALF2=%zu, OUT1=%zu, OUT2=%zu",
           reachable_poses_mid.size(), reachable_poses_half1.size(), reachable_poses_half2.size(),
           reachable_poses_out1.size(), reachable_poses_out2.size());
  
  // 排序
  double target_distance, mid_target_distance;
  nh_.param("test/target_distance", target_distance, 0.2);
  nh_.param("test/mid_target_distance", mid_target_distance, 0.2);
  
  reachable_poses_out1 = sortPosesByDistanceToPoint(reachable_poses_out1, reference_point_out1, translation_axis_vec, target_distance);
  reachable_poses_out2 = sortPosesByDistanceToPoint(reachable_poses_out2, reference_point_out2, translation_axis_vec, target_distance);
  reachable_poses_mid = sortPosesByDistanceToPoint(reachable_poses_mid, reference_point_mid, mid_direction, mid_target_distance);
  reachable_poses_half1 = sortPosesByDistanceToPoint(reachable_poses_half1, reference_point_half1, half_direction_1, target_distance);
  reachable_poses_half2 = sortPosesByDistanceToPoint(reachable_poses_half2, reference_point_half2, half_direction_2, target_distance);
  
  //==========================================================================
  // 构造响应：按顺序 [MID, HALF1, HALF2, OUT1, OUT2]
  //==========================================================================
  if (!reachable_poses_mid.empty()) {
    res.target_poses.push_back(reachable_poses_mid[0].pose);
    res.pitch_angles.push_back(pitch_0);
    res.roll_angles.push_back(roll_0);
    res.pose_names.push_back("MID");
  }
  
  if (!reachable_poses_half1.empty()) {
    res.target_poses.push_back(reachable_poses_half1[0].pose);
    res.pitch_angles.push_back(pitch_half1);
    res.roll_angles.push_back(roll_half1);
    res.pose_names.push_back("HALF1");
  }
  
  if (!reachable_poses_half2.empty()) {
    res.target_poses.push_back(reachable_poses_half2[0].pose);
    res.pitch_angles.push_back(pitch_half2);
    res.roll_angles.push_back(roll_half2);
    res.pose_names.push_back("HALF2");
  }
  
  if (!reachable_poses_out1.empty()) {
    res.target_poses.push_back(reachable_poses_out1[0].pose);
    res.pitch_angles.push_back(pitch_1);
    res.roll_angles.push_back(roll_1);
    res.pose_names.push_back("OUT1");
  }
  
  if (!reachable_poses_out2.empty()) {
    res.target_poses.push_back(reachable_poses_out2[0].pose);
    res.pitch_angles.push_back(pitch_2);
    res.roll_angles.push_back(roll_2);
    res.pose_names.push_back("OUT2");
  }
  
  ROS_INFO("[GetGoalAndAngle] Returning %zu target poses", res.target_poses.size());
  
  res.call_success = (res.target_poses.size() > 0);
  return true;
}

geometry_msgs::PoseArray createLineVisualization(const Line3D& line, double t_start, double t_end, int num_points) {
  geometry_msgs::PoseArray line_msg;
  line_msg.header.frame_id = "link00";
  line_msg.header.stamp = ros::Time::now();
  
  for (int i = 0; i < num_points; ++i) {
    double t = t_start + (t_end - t_start) * i / (num_points - 1);
    Eigen::Vector3d point = line.point + t * line.direction;
    
    geometry_msgs::Pose pose;
    pose.position.x = point.x();
    pose.position.y = point.y();
    pose.position.z = point.z();
    pose.orientation.w = 1.0;
    
    line_msg.poses.push_back(pose);
  }
  return line_msg;
};

bool ArmController::planToFivePointServer(
    arm_controller_srvs::PlanTofivepoint::Request& req,
    arm_controller_srvs::PlanTofivepoint::Response& res) {
  res.call_success = false;
  
  //test - 从 ROS 参数服务器读取配置
  double line_x, line_y, line_z;
  double center_x, center_y, center_z;
  double line_pitch_link00, line_yaw_link00, line_roll_link00;
  double line_pitch_link01, line_yaw_link01, line_roll_link01;
  double line_pitch, line_yaw, line_roll;
  double rotation_angle_deg, sample_start, sample_end;
  double origin_direction_x, origin_direction_y, origin_direction_z;
  double rotation_axis_x, rotation_axis_y, rotation_axis_z;
  // 半侧点1和2：沿着直线轴线方向在x轴负方向上加入偏移量
  double half_offset_distance = -0.15, mid_offset_distance = -0.15;  // x轴负方向偏移量
  // double translation_axis_x, translation_axis_y, translation_axis_z;
  int num_samples, num_rotations;
  double angle_step_deg;
  // MID 线专用采样参数
  double mid_sample_start, mid_sample_end;
  int mid_num_samples;

  // 提取位置
  line_x = req.target_pose.pose.position.x;
  line_y = req.target_pose.pose.position.y;
  line_z = req.target_pose.pose.position.z;
  
  // 将四元数转换为 roll, pitch, yaw
  // 先读取传入的四元数
  tf::Quaternion quat_input(
    req.target_pose.pose.orientation.x,
    req.target_pose.pose.orientation.y,
    req.target_pose.pose.orientation.z,
    req.target_pose.pose.orientation.w
  );
  
  // 当前变换后的状态：原 X → 现 Z，原 Y → 现 X，原 Z → 现 Y
  // 需要逆变换恢复到原始状态
  
  // 逆变换：先绕Y轴逆时针旋转90度
  tf::Quaternion rot_y_inv;
  rot_y_inv.setRotation(tf::Vector3(0, 1, 0), M_PI / 2.0);  // 逆时针 90度 = π/2
  
  // 逆变换：再绕X轴逆时针旋转90度
  tf::Quaternion rot_x_inv;
  rot_x_inv.setRotation(tf::Vector3(1, 0, 0), M_PI / 2.0);  // 逆时针 90度 = π/2
  
  // 额外变换：绕Y轴旋转180度
  tf::Quaternion rot_y_180;
  rot_y_180.setRotation(tf::Vector3(0, 1, 0), M_PI);  // 180度 = π

  // 应用变换：先应用逆变换，再绕Y轴旋转180度
  tf::Quaternion quat = quat_input * rot_y_inv * rot_x_inv * rot_y_180;
  
  // tf::Quaternion quat = quat_input;

  // 发布变换后的姿态
  geometry_msgs::PoseStamped transformed_pose_msg;
  transformed_pose_msg.header.frame_id = "link00";
  transformed_pose_msg.header.stamp = ros::Time::now();
  transformed_pose_msg.pose.position.x = line_x;
  transformed_pose_msg.pose.position.y = line_y;
  transformed_pose_msg.pose.position.z = line_z;
  transformed_pose_msg.pose.orientation.w = quat.w();
  transformed_pose_msg.pose.orientation.x = quat.x();
  transformed_pose_msg.pose.orientation.y = quat.y();
  transformed_pose_msg.pose.orientation.z = quat.z();
  transformed_input_pub_.publish(transformed_pose_msg);
  
  // 通过 TF 广播变换后的坐标系
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(line_x, line_y, line_z));
  transform.setRotation(quat);
  tf_broadcaster_.sendTransform(
    tf::StampedTransform(transform, ros::Time::now(), "link00", "transformed_input_frame")
  );
  
  ROS_INFO("[PlanToFivePoint] Published transformed input pose and TF frame 'transformed_input_frame'");
  
  tf::Matrix3x3 mat(quat);
  mat.getRPY(line_roll_link00, line_pitch_link00, line_yaw_link00);
  
  ROS_INFO("[PlanToFivePoint] Received pose: pos=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f)",
           line_x, line_y, line_z, 
           line_roll_link00, line_pitch_link00, line_yaw_link00);
  
  // 从 ROS 参数服务器读取其他配置参数
  nh_.param("test/mid_sample_start", mid_sample_start, -1.0);
  nh_.param("test/mid_sample_end", mid_sample_end, 0.5);
  nh_.param("test/mid_num_samples", mid_num_samples, 50);
  nh_.param("test/angle_step_deg", angle_step_deg, 45.0);
  nh_.param("test/num_rotations", num_rotations, 1);
  nh_.param("test/num_samples", num_samples, 50);
  nh_.param("test/sample_start", sample_start, -1.0);
  nh_.param("test/sample_end", sample_end, 0.0);
  nh_.param("test/rotation_angle_deg", rotation_angle_deg, 45.0);
  
  // 旋转中心默认为直线起点
  center_x = line_x;
  center_y = line_y;
  center_z = line_z;

  // line_pitch_link00 = line_pitch_link00;
  // line_yaw_link00 = line_yaw_link00-1.5708;
  // line_roll_link00 = line_roll_link00+0.7854;
  //将line_pitch, line_yaw, line_roll转换为笛卡尔坐标系
  line_pitch = line_pitch_link00;
  line_yaw = line_yaw_link00;
  line_roll = line_roll_link00;
  // line_pitch = -line_pitch_link00;
  // line_yaw = line_yaw_link00;
  // line_roll = line_roll_link00-1.5708
  // line_pitch_link01 = line_roll_link00;
  // line_yaw_link01 = line_yaw_link00;
  // line_roll_link01 = -line_pitch_link00;


  // line_pitch = -line_pitch_link01;
  // line_yaw = line_yaw_link01;
  // line_roll = line_roll_link01-1.5708;
  std::cout << "Line direction (RPY): " << line_roll << ", " << line_pitch << ", " << line_yaw << std::endl;

  // std::cout << "Line direction: " << line_pitch << ", " << line_yaw << ", " << line_roll << std::endl;
  // // 计算方向向量(假设沿着姿态的X轴方向)
  // origin_direction_x = cos(line_yaw) * cos(line_pitch);
  // origin_direction_y = sin(line_yaw) * cos(line_pitch);
  // origin_direction_z = sin(line_pitch);
  // std::cout << "Origin direction: " << origin_direction_x << ", " << origin_direction_y << ", " << origin_direction_z << std::endl;
  
  // 使用四元数计算方向向量（假设沿着姿态的X轴方向）
  // 方法：将欧拉角转换为四元数，然后旋转单位X轴向量
  // tf::Quaternion q;
  // q.setRPY(line_roll, line_pitch, line_yaw);  // Roll, Pitch, Yaw 顺序
  
  // 定义初始方向向量（沿X轴）
  tf::Vector3 x_axis(1.0, 0.0, 0.0);
  
  // 使用四元数旋转向量
  tf::Vector3 rotated_direction = tf::quatRotate(quat, x_axis);
  
  // 提取旋转后的方向向量
  origin_direction_x = rotated_direction.x();
  origin_direction_y = rotated_direction.y();
  origin_direction_z = rotated_direction.z();
  
  std::cout << "Origin direction (via quaternion): " << origin_direction_x << ", " 
            << origin_direction_y << ", " << origin_direction_z << std::endl;

  //将origin_direction在XOZ平面内旋转90度得到rotation_axis
  rotation_axis_x = -origin_direction_z;
  rotation_axis_y = origin_direction_y;
  rotation_axis_z = origin_direction_x;
  std::cout << "Rotation axis: " << rotation_axis_x << ", " << rotation_axis_y << ", " << rotation_axis_z << std::endl;

  // 计算平移轴：通过 pitch、yaw、roll 旋转 (0, 1, 0) 向量
  // 使用 Eigen 的 AngleAxis 构建旋转矩阵
  Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(line_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Matrix3d R_pitch = Eigen::AngleAxisd(line_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
  Eigen::Matrix3d R_roll = Eigen::AngleAxisd(line_roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
  // 组合旋转矩阵：R = Rz(yaw) * Ry(pitch) * Rx(roll)
  Eigen::Matrix3d R_total = R_yaw * R_pitch * R_roll;
  // 对 (0, 1, 0) 向量进行旋转
  Eigen::Vector3d original_y_axis(0.0, 1.0, 0.0);
  Eigen::Vector3d translation_axis_vec = R_total * original_y_axis;   
  std::cout << "Translation axis: " << translation_axis_vec[0] << ", " << translation_axis_vec[1] << ", " << translation_axis_vec[2] << std::endl;

  // 创建原始直线
  Line3D original_line;
  original_line.point = Eigen::Vector3d(line_x, line_y, line_z);
  original_line.direction = Eigen::Vector3d(origin_direction_x, origin_direction_y, origin_direction_z).normalized();

  // 定义旋转参数
  Eigen::Vector3d rotation_center(center_x, center_y, center_z);
  Eigen::Vector3d rotation_axis(rotation_axis_x, rotation_axis_y, rotation_axis_z);  // 绕 Z 轴旋转
  double rotation_angle_rad = rotation_angle_deg * M_PI / 180.0;

  Eigen::Vector3d base_point = original_line.point;
  Eigen::Vector3d reference_point_mid = base_point + mid_offset_distance * original_line.direction;  // 管道中心点
  //1.生成5个轴线
  // 方法: 批量旋转并采样
  //==========================================================================
  // 第一步：生成5条直线（纯几何变换）
  //==========================================================================
  std::cout << "\n========== STEP 1: Generate 5 Lines (Geometry Only) ==========" << std::endl;
  
  // 1. MID 直线（原直线，不旋转）
  std::cout << "[1/5] MID line (original, no rotation)" << std::endl;
  Line3D line_mid = original_line;
  
  // 2. HALF1 直线（正转 angle_step_deg 度）
  std::cout << "[2/5] HALF1 line (rotate +" << angle_step_deg << " deg)" << std::endl;
  double angle_step_rad_1 = angle_step_deg * M_PI / 180.0;
  Line3D line_half1 = rotateLine(original_line, rotation_center, rotation_axis, angle_step_rad_1);
  
  // 3. HALF2 直线（反转 angle_step_deg 度）
  std::cout << "[3/5] HALF2 line (rotate -" << angle_step_deg << " deg)" << std::endl;
  double angle_step_rad_2 = (360.0 - angle_step_deg) * M_PI / 180.0;
  Line3D line_half2 = rotateLine(original_line, rotation_center, rotation_axis, angle_step_rad_2);
  
  // 4. OUT1 直线（平移 +0.3m）
  std::cout << "[4/5] OUT1 line (translate +0.3m)" << std::endl;
  double distance_1 = 0.3;
  Line3D line_out1 = translateLineGeometry(original_line, translation_axis_vec, distance_1);
  
  // 5. OUT2 直线（平移 -0.3m）
  std::cout << "[5/5] OUT2 line (translate -0.3m)" << std::endl;
  double distance_2 = -0.3;
  Line3D line_out2 = translateLineGeometry(original_line, translation_axis_vec, distance_2);
  
  //==========================================================================
  // 第二步：计算参考点
  //==========================================================================
  std::cout << "\n========== STEP 2: Calculate Reference Points ==========" << std::endl;
  
  // 提取各条直线的方向
  Eigen::Vector3d mid_direction = line_mid.direction.normalized();
  Eigen::Vector3d half_direction_1 = line_half1.direction.normalized();
  Eigen::Vector3d half_direction_2 = line_half2.direction.normalized();
  
  // 计算参考点
  Eigen::Vector3d reference_point_out1 = base_point + translation_axis_vec * distance_1;
  Eigen::Vector3d reference_point_out2 = base_point + translation_axis_vec * distance_2;
  Eigen::Vector3d reference_point_half1 = base_point + half_offset_distance * half_direction_1;
  Eigen::Vector3d reference_point_half2 = base_point + half_offset_distance * half_direction_2;
  
  std::cout << "  MID  reference: (" << reference_point_mid.transpose() << ")" << std::endl;
  std::cout << "  OUT1 reference: (" << reference_point_out1.transpose() << ")" << std::endl;
  std::cout << "  OUT2 reference: (" << reference_point_out2.transpose() << ")" << std::endl;
  std::cout << "  HALF1 reference: (" << reference_point_half1.transpose() << ")" << std::endl;
  std::cout << "  HALF2 reference: (" << reference_point_half2.transpose() << ")" << std::endl;
  
  //==========================================================================
  // 检查并调整顺序：确保 OUT1 和 HALF1 在同一侧，OUT2 和 HALF2 在另一侧
  //==========================================================================
  std::cout << "\n[Order Check] Checking spatial arrangement..." << std::endl;
  
  // 计算向量：从 MID 到各个点
  Eigen::Vector3d vec_mid_to_out1 = reference_point_out1 - reference_point_mid;
  Eigen::Vector3d vec_mid_to_out2 = reference_point_out2 - reference_point_mid;
  Eigen::Vector3d vec_mid_to_half1 = reference_point_half1 - reference_point_mid;
  Eigen::Vector3d vec_mid_to_half2 = reference_point_half2 - reference_point_mid;
  
  // 使用点积判断同侧性：如果点积 > 0，则在同一侧
  double dot_out1_half1 = vec_mid_to_out1.dot(vec_mid_to_half1);
  double dot_out1_half2 = vec_mid_to_out1.dot(vec_mid_to_half2);
  double dot_out2_half1 = vec_mid_to_out2.dot(vec_mid_to_half1);
  double dot_out2_half2 = vec_mid_to_out2.dot(vec_mid_to_half2);
  
  std::cout << "[Order Check] Dot products:" << std::endl;
  std::cout << "  OUT1 · HALF1: " << dot_out1_half1 << std::endl;
  std::cout << "  OUT1 · HALF2: " << dot_out1_half2 << std::endl;
  std::cout << "  OUT2 · HALF1: " << dot_out2_half1 << std::endl;
  std::cout << "  OUT2 · HALF2: " << dot_out2_half2 << std::endl;
  
  // 判断是否需要交换：
  // 如果 OUT1 与 HALF2 更接近（点积更大），则需要交换 HALF1 和 HALF2
  bool need_swap = (dot_out1_half2 > dot_out1_half1);
  
  if (need_swap) {
    std::cout << "[Order Check] OUT1 is closer to HALF2, swapping HALF1 and HALF2..." << std::endl;
    
    // 交换参考点
    std::swap(reference_point_half1, reference_point_half2);
    
    // 交换方向
    std::swap(half_direction_1, half_direction_2);
    
    // 交换直线
    std::swap(line_half1, line_half2);
    
    // 注意：pitch 和 roll 将在后面采样时自动计算，不需要交换
    
    std::cout << "[Order Check] Swapped! New arrangement:" << std::endl;
    std::cout << "  HALF1 reference: (" << reference_point_half1.transpose() << ")" << std::endl;
    std::cout << "  HALF2 reference: (" << reference_point_half2.transpose() << ")" << std::endl;
  } else {
    std::cout << "[Order Check] Order is correct, no swap needed." << std::endl;
  }
  
  // 发布五个参考点到 RViz
  geometry_msgs::PoseArray reference_points_msg;
  reference_points_msg.header.frame_id = "link00";
  reference_points_msg.header.stamp = ros::Time::now();
  
  std::vector<Eigen::Vector3d> ref_points = {
    reference_point_mid, reference_point_out1, reference_point_out2,
    reference_point_half1, reference_point_half2
  };
  
  for (const auto& point : ref_points) {
    geometry_msgs::Pose pose;
    pose.position.x = point[0];
    pose.position.y = point[1];
    pose.position.z = point[2];
    pose.orientation.w = 1.0;
    reference_points_msg.poses.push_back(pose);
  }
  
  reference_points_pub_.publish(reference_points_msg);
  std::cout << "[VIZ] Published 5 reference points" << std::endl;
  
  //==========================================================================
  // 第三步：以参考点为原点重新生成5条直线（方向不变）
  //==========================================================================
  std::cout << "\n========== STEP 3: Create New Lines with Reference Points as Origins ==========" << std::endl;
  
  // 创建以参考点为起点的新直线，方向保持不变
  Line3D line_mid_sampled;
  line_mid_sampled.point = reference_point_mid;
  line_mid_sampled.direction = mid_direction;
  std::cout << "  MID line: origin=(" << reference_point_mid.transpose() 
            << "), direction=(" << mid_direction.transpose() << ")" << std::endl;
  
  Line3D line_half1_sampled;
  line_half1_sampled.point = reference_point_half1;
  line_half1_sampled.direction = half_direction_1;
  std::cout << "  HALF1 line: origin=(" << reference_point_half1.transpose() 
            << "), direction=(" << half_direction_1.transpose() << ")" << std::endl;
  
  Line3D line_half2_sampled;
  line_half2_sampled.point = reference_point_half2;
  line_half2_sampled.direction = half_direction_2;
  std::cout << "  HALF2 line: origin=(" << reference_point_half2.transpose() 
            << "), direction=(" << half_direction_2.transpose() << ")" << std::endl;
  
  Line3D line_out1_sampled;
  line_out1_sampled.point = reference_point_out1;
  line_out1_sampled.direction = mid_direction;  // OUT1/OUT2 使用原始管道方向（与 MID 一致）
  std::cout << "  OUT1 line: origin=(" << reference_point_out1.transpose() 
            << "), direction=(" << mid_direction.transpose() << ")" << std::endl;
  
  Line3D line_out2_sampled;
  line_out2_sampled.point = reference_point_out2;
  line_out2_sampled.direction = mid_direction;  // OUT1/OUT2 使用原始管道方向（与 MID 一致）
  std::cout << "  OUT2 line: origin=(" << reference_point_out2.transpose() 
            << "), direction=(" << mid_direction.transpose() << ")" << std::endl;
  
  //==========================================================================
  // 第四步：采样并检测可达性
  //==========================================================================
  std::cout << "\n========== STEP 4: Sample and Check Reachability ==========" << std::endl;
  
  int viz_points = 50;
  double pitch_0 = 0.0, roll_0 = 0.0;
  double pitch_1 = 0.0, roll_1 = 0.0;
  double pitch_2 = 0.0, roll_2 = 0.0;
  double pitch_half1 = 0.0, roll_half1 = 0.0;
  double pitch_half2 = 0.0, roll_half2 = 0.0;
  
  // 1. MID 直线采样
  std::cout << "[1/5] Sampling MID line..." << std::endl;
  std::vector<geometry_msgs::PoseStamped> reachable_poses_mid = 
      sampleAndCheckReachability(line_mid_sampled, mid_sample_start, mid_sample_end, mid_num_samples, pitch_0, roll_0);
  line_mid_pub_.publish(createLineVisualization(line_mid_sampled, mid_sample_start, mid_sample_end, viz_points));
  
  // 2. HALF1 直线采样
  std::cout << "[2/5] Sampling HALF1 line..." << std::endl;
  std::vector<geometry_msgs::PoseStamped> reachable_poses_half1 = 
      sampleAndCheckReachability(line_half1_sampled, sample_start, sample_end, num_samples, pitch_half1, roll_half1);
  line_half1_pub_.publish(createLineVisualization(line_half1_sampled, sample_start, sample_end, viz_points));
  
  // 3. HALF2 直线采样
  std::cout << "[3/5] Sampling HALF2 line..." << std::endl;
  std::vector<geometry_msgs::PoseStamped> reachable_poses_half2 = 
      sampleAndCheckReachability(line_half2_sampled, sample_start, sample_end, num_samples, pitch_half2, roll_half2);
  line_half2_pub_.publish(createLineVisualization(line_half2_sampled, sample_start, sample_end, viz_points));
  
  // 4. OUT1 直线采样
  std::cout << "[4/5] Sampling OUT1 line..." << std::endl;
  std::vector<geometry_msgs::PoseStamped> reachable_poses_out1 = 
      sampleAndCheckReachability(line_out1_sampled, sample_start, sample_end, num_samples, pitch_1, roll_1);
  line_out1_pub_.publish(createLineVisualization(line_out1_sampled, sample_start, sample_end, viz_points));
  
  // 5. OUT2 直线采样
  std::cout << "[5/5] Sampling OUT2 line..." << std::endl;
  std::vector<geometry_msgs::PoseStamped> reachable_poses_out2 = 
      sampleAndCheckReachability(line_out2_sampled, sample_start, sample_end, num_samples, pitch_2, roll_2);
  line_out2_pub_.publish(createLineVisualization(line_out2_sampled, sample_start, sample_end, viz_points));
  
  std::cout << "[INFO] Sampling completed. Reachable poses: MID=" << reachable_poses_mid.size()
            << ", HALF1=" << reachable_poses_half1.size() << ", HALF2=" << reachable_poses_half2.size()
            << ", OUT1=" << reachable_poses_out1.size() << ", OUT2=" << reachable_poses_out2.size() << std::endl;
  
  //==========================================================================
  // 第五步：排序可达点
  //==========================================================================
  std::cout << "\n========== STEP 5: Sort Reachable Poses ==========" << std::endl;
  
  // 检查是否有可达点
  if (reachable_poses_half1.empty() || reachable_poses_half2.empty()) {
    ROS_WARN("[PlanToFivePoint] Some lines have no reachable poses");
  }
  
  // 从 ROS 参数服务器读取 target_distance
  double target_distance;
  nh_.param("test/target_distance", target_distance, 0.2);  // 默认值 0.2
  double mid_target_distance;
  nh_.param("test/mid_target_distance", mid_target_distance, 0.2);  // 默认值 0.2
  std::cout << "[INFO] Target distance for sorting: " << target_distance << " m" << std::endl;
  
  reachable_poses_out1 = sortPosesByDistanceToPoint(reachable_poses_out1, reference_point_out1, translation_axis_vec, target_distance);
  reachable_poses_out2 = sortPosesByDistanceToPoint(reachable_poses_out2, reference_point_out2, translation_axis_vec, target_distance);
  reachable_poses_mid = sortPosesByDistanceToPoint(reachable_poses_mid, reference_point_mid, mid_direction, mid_target_distance);
  reachable_poses_half1 = sortPosesByDistanceToPoint(reachable_poses_half1, reference_point_half1, half_direction_1, target_distance);
  reachable_poses_half2 = sortPosesByDistanceToPoint(reachable_poses_half2, reference_point_half2, half_direction_2, target_distance);

  bool success[5] = {false, false, false, false, false};
  
  // 4. 发布目标点位供 RViz 可视化
  geometry_msgs::PoseArray target_poses_msg;
  target_poses_msg.header.frame_id = "link00";
  target_poses_msg.header.stamp = ros::Time::now();
  
  // 添加所有目标点位
  if (!reachable_poses_mid.empty()) {
    target_poses_msg.poses.push_back(reachable_poses_mid[0].pose);
    ROS_INFO("[PlanToFivePoint] Target 1 (MID): (%.3f, %.3f, %.3f)",
             reachable_poses_mid[0].pose.position.x,
             reachable_poses_mid[0].pose.position.y,
             reachable_poses_mid[0].pose.position.z);
  }
  if (!reachable_poses_half1.empty()) {
    target_poses_msg.poses.push_back(reachable_poses_half1[0].pose);
    ROS_INFO("[PlanToFivePoint] Target 2 (HALF1): (%.3f, %.3f, %.3f)",
             reachable_poses_half1[0].pose.position.x,
             reachable_poses_half1[0].pose.position.y,
             reachable_poses_half1[0].pose.position.z);
  }
  if (!reachable_poses_half2.empty()) {
    target_poses_msg.poses.push_back(reachable_poses_half2[0].pose);
    ROS_INFO("[PlanToFivePoint] Target 3 (HALF2): (%.3f, %.3f, %.3f)",
             reachable_poses_half2[0].pose.position.x,
             reachable_poses_half2[0].pose.position.y,
             reachable_poses_half2[0].pose.position.z);
  }
  if (!reachable_poses_out1.empty()) {
    target_poses_msg.poses.push_back(reachable_poses_out1[0].pose);
    ROS_INFO("[PlanToFivePoint] Target 4 (OUT1): (%.3f, %.3f, %.3f)",
             reachable_poses_out1[0].pose.position.x,
             reachable_poses_out1[0].pose.position.y,
             reachable_poses_out1[0].pose.position.z);
  }
  if (!reachable_poses_out2.empty()) {
    target_poses_msg.poses.push_back(reachable_poses_out2[0].pose);
    ROS_INFO("[PlanToFivePoint] Target 5 (OUT2): (%.3f, %.3f, %.3f)",
             reachable_poses_out2[0].pose.position.x,
             reachable_poses_out2[0].pose.position.y,
             reachable_poses_out2[0].pose.position.z);
  }
  
  // 发布目标点位
  target_poses_pub_.publish(target_poses_msg);
  ROS_INFO("[PlanToFivePoint] Published %zu target poses to /arm_controller/target_poses",
           target_poses_msg.poses.size());
  
  // 发布各组可达点供 RViz 可视化
  geometry_msgs::PoseArray poses_out1_msg, poses_out2_msg, poses_mid_msg, poses_half1_msg, poses_half2_msg;
  poses_out1_msg.header.frame_id = "link00";
  poses_out1_msg.header.stamp = ros::Time::now();
  poses_out2_msg.header = poses_mid_msg.header = poses_half1_msg.header = poses_half2_msg.header = poses_out1_msg.header;
  
  // 填充各组点
  for (const auto& pose_stamped : reachable_poses_out1) {
    poses_out1_msg.poses.push_back(pose_stamped.pose);
  }
  for (const auto& pose_stamped : reachable_poses_out2) {
    poses_out2_msg.poses.push_back(pose_stamped.pose);
  }
  for (const auto& pose_stamped : reachable_poses_mid) {
    poses_mid_msg.poses.push_back(pose_stamped.pose);
  }
  for (const auto& pose_stamped : reachable_poses_half1) {
    poses_half1_msg.poses.push_back(pose_stamped.pose);
  }
  for (const auto& pose_stamped : reachable_poses_half2) {
    poses_half2_msg.poses.push_back(pose_stamped.pose);
  }
  
  // 发布各组点
  poses_out1_pub_.publish(poses_out1_msg);
  poses_out2_pub_.publish(poses_out2_msg);
  poses_mid_pub_.publish(poses_mid_msg);
  poses_half1_pub_.publish(poses_half1_msg);
  poses_half2_pub_.publish(poses_half2_msg);
  
  ROS_INFO("[PlanToFivePoint] Published reachable poses: OUT1=%zu, OUT2=%zu, MID=%zu, HALF1=%zu, HALF2=%zu",
           poses_out1_msg.poses.size(), poses_out2_msg.poses.size(), poses_mid_msg.poses.size(),
           poses_half1_msg.poses.size(), poses_half2_msg.poses.size());
  
  // 发布 MID 组的所有采样点（包括可达和不可达）
  geometry_msgs::PoseArray poses_mid_all_msg;
  poses_mid_all_msg.header.frame_id = "link00";
  poses_mid_all_msg.header.stamp = ros::Time::now();
  
  // 直接从 MID 直线采样所有点（使用以参考点为起点的直线）
  std::vector<Eigen::Vector3d> mid_all_sampled_points = 
      line_mid_sampled.samplePoints(mid_sample_start, mid_sample_end, mid_num_samples);
  
  for (const auto& point : mid_all_sampled_points) {
    geometry_msgs::Pose pose;
    pose.position.x = point.x();
    pose.position.y = point.y();
    pose.position.z = point.z();
    pose.orientation.w = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    poses_mid_all_msg.poses.push_back(pose);
  }
  
  poses_mid_all_pub_.publish(poses_mid_all_msg);
  ROS_INFO("[PlanToFivePoint] Published %zu total MID sampled points (reachable + unreachable)",
           poses_mid_all_msg.poses.size());
  geometry_msgs::PoseStamped center_poses_msg;
  center_poses_msg.header.frame_id = "link00";
  center_poses_msg.header.stamp = ros::Time::now();
  center_poses_msg.pose.position.x = center_x;
  center_poses_msg.pose.position.y = center_y;
  center_poses_msg.pose.position.z = center_z;
  
  // 将方向向量转换为四元数
  // 使用方向向量创建一个旋转，使得Z轴指向该方向
  Eigen::Vector3d z_axis(0, 0, 1);
  Eigen::Vector3d direction = original_line.direction.normalized();
  
  // 计算旋转轴和角度
  Eigen::Vector3d quat_rotation_axis = z_axis.cross(direction);
  double quat_rotation_angle = std::acos(z_axis.dot(direction));
  
  Eigen::Quaterniond eigen_quat;
  if (quat_rotation_axis.norm() < 1e-6) {
    // 方向向量与Z轴平行或反平行
    if (z_axis.dot(direction) > 0) {
      eigen_quat = Eigen::Quaterniond::Identity();  // 同向
    } else {
      eigen_quat = Eigen::Quaterniond(0, 1, 0, 0);  // 反向，绕X轴旋转180度
    }
  } else {
    quat_rotation_axis.normalize();
    eigen_quat = Eigen::Quaterniond(Eigen::AngleAxisd(quat_rotation_angle, quat_rotation_axis));
  }
  
  center_poses_msg.pose.orientation.w = req.target_pose.pose.orientation.w;
  center_poses_msg.pose.orientation.x = req.target_pose.pose.orientation.x;
  center_poses_msg.pose.orientation.y = req.target_pose.pose.orientation.y;
  center_poses_msg.pose.orientation.z = req.target_pose.pose.orientation.z;
  center_pub_.publish(center_poses_msg);
  ROS_INFO("[PlanToFivePoint] Published center pose to /arm_controller/center_point");
  //5.执行可达点
  //依次执行 OUT1 -> HALF1 -> MID -> HALF2 -> OUT2
  //由于已经确保 HALF1 和 OUT1 在同一侧，顺序是空间连续的
  //如果某个点为空（不可达），就跳过它继续执行下一个点
  //安全逻辑：如果 MID 消失，先去默认点再继续执行 HALF2/OUT2
  
  bool last_success = true;  // 跟踪上一步是否成功（或被跳过）
  
  // OUT1
  if (!reachable_poses_out1.empty()) {
    ROS_INFO("[PlanToFivePoint] Executing OUT1...");
    success[0] = executeMotionToTarget(reachable_poses_out1[0].pose, pitch_1, roll_1, 10.0);
    last_success = success[0];
    if (!success[0]) {
      ROS_WARN("[PlanToFivePoint] OUT1 execution failed, stopping...");
    }
  } else {
    ROS_WARN("[PlanToFivePoint] OUT1 has no reachable poses, skipping...");
    success[0] = true;  // 设为true表示跳过，以便继续执行
  }

  // 延迟 0.5 秒
  // if (last_success) {
    ros::Duration(0.5).sleep();
  // }
  
  // HALF1
  if (last_success) {
    if (!reachable_poses_half1.empty()) {
      ROS_INFO("[PlanToFivePoint] Executing HALF1...");
      success[1] = executeMotionToTarget(reachable_poses_half1[0].pose, pitch_half1, roll_half1, 10.0);
      last_success = success[1];
      if (!success[1]) {
        ROS_WARN("[PlanToFivePoint] HALF1 execution failed, stopping...");
      }
    } else {
      ROS_WARN("[PlanToFivePoint] HALF1 has no reachable poses, skipping...");
      success[1] = true;  // 跳过
    }
  }
  
  // 延迟 0.5 秒
  // if (last_success) {
    ros::Duration(0.5).sleep();
  // }
  
  // MID
  if (last_success) {
    if (!reachable_poses_mid.empty()) {
      ROS_INFO("[PlanToFivePoint] Executing MID...");
      success[2] = executeMotionToTarget(reachable_poses_mid[0].pose, pitch_0, roll_0, 10.0);
      last_success = success[2];
      if (!success[2]) {
        ROS_WARN("[PlanToFivePoint] MID execution failed, stopping...");
      }
    } else {
      // MID 消失，先去默认点
      ROS_WARN("[PlanToFivePoint] MID has no reachable poses, going to default point for safety...");
      if (!goToDefaultPoint()) {
        ROS_ERROR("[PlanToFivePoint] Failed to go to default point, stopping...");
        last_success = false;
        success[2] = false;
      } else {
        ROS_INFO("[PlanToFivePoint] Successfully moved to default point");
        success[2] = true;  // 标记为成功，继续执行
      }
    }
  }
  
  // 延迟 0.5 秒
  // if (last_success) {
    ros::Duration(0.5).sleep();
  // }
  
  // HALF2
  if (last_success) {
    if (!reachable_poses_half2.empty()) {
      ROS_INFO("[PlanToFivePoint] Executing HALF2...");
      success[3] = executeMotionToTarget(reachable_poses_half2[0].pose, pitch_half2, roll_half2, 10.0);
      last_success = success[3];
      if (!success[3]) {
        ROS_WARN("[PlanToFivePoint] HALF2 execution failed, stopping...");
      }
    } else {
      ROS_WARN("[PlanToFivePoint] HALF2 has no reachable poses, skipping...");
      success[3] = true;  // 跳过
    }
  }
  
  // 延迟 0.5 秒
  // if (last_success) {
    ros::Duration(0.5).sleep();
  // }
  
  // OUT2
  if (last_success) {
    if (!reachable_poses_out2.empty()) {
      ROS_INFO("[PlanToFivePoint] Executing OUT2...");
      success[4] = executeMotionToTarget(reachable_poses_out2[0].pose, pitch_2, roll_2, 10.0);
      if (!success[4]) {
        ROS_WARN("[PlanToFivePoint] OUT2 execution failed");
      }
    } else {
      ROS_WARN("[PlanToFivePoint] OUT2 has no reachable poses, skipping...");
      success[4] = true;  // 跳过
    }
  }
  
  // 统计执行结果
  int executed_count = 0;
  for (int i = 0; i < 5; i++) {
    if (success[i]) executed_count++;
  }
  ROS_INFO("[PlanToFivePoint] Execution completed: %d/5 points executed successfully", executed_count); 
  return true;
}

bool ArmController::planToTargetPose(const geometry_msgs::Pose& target_pose, const double& joint6_pos, const bool& use_manual_joint6) {
  ROS_INFO("[PlanToTargetPose] Planning to target position: (%.3f, %.3f, %.3f)",
           target_pose.position.x,
           target_pose.position.y,
           target_pose.position.z);
  
  // 检查机械臂状态
  if (arm_control_fsm_ != ArmControlFsm::Home &&
      arm_control_fsm_ != ArmControlFsm::Arrived) {
    ROS_WARN("[PlanToTargetPose] Arm is not in Home or Arrived state. Current state: %d", 
             static_cast<int>(arm_control_fsm_));
    return false;
  }
  
  // 获取当前状态
  Eigen::Matrix4d start_ee_pose = arm_model_->forwardKinematics(low_state_.getQ());
  Eigen::Matrix<double, 6, 1> start_joint_pos = low_state_.getQ();
  Eigen::Matrix4d camera_target_pose, target_pose_eigen;
  Eigen::Matrix<double, 6, 1> target_joint_pos;
  bool find_ik{false};
  
  // 转换目标位姿（从 geometry_msgs 到 Eigen）
  arm_controller::geometryMsgsPose2Pose(target_pose, camera_target_pose);
  target_pose_eigen = camera_target_pose;
  
  // 补偿相机偏移（从相机位置计算末端执行器位置）
  target_pose_eigen.block<3, 1>(0, 3) =
      camera_target_pose.block<3, 1>(0, 3) -
      camera_target_pose.block<3, 3>(0, 0) * kCameraPosBias_E_;
  
  // 计算逆运动学
  find_ik = arm_model_->inverseKinematics(target_pose_eigen, start_joint_pos,
                                         target_joint_pos, true);
  
  ROS_INFO("[PlanToTargetPose] IK solution found: %s", find_ik ? "YES" : "NO");
  if (find_ik) {
    ROS_INFO("[PlanToTargetPose] Joint[2] value: %.3f rad (%.1f deg)", 
             target_joint_pos[2], target_joint_pos[2] * 180.0 / M_PI);
  }
  
  if (!arm_motor_safe_) {
    ROS_ERROR("[PlanToTargetPose] Arm motor is not safe!");
    return false;
  }
  
  if (!find_ik) {
    ROS_ERROR("[PlanToTargetPose] No IK solution found for target pose");
    return false;
  }
  
  // 检查运动是否太小（已经在目标位置附近）
  if ((target_joint_pos - start_joint_pos).norm() <= 0.042) {
    ROS_INFO("[PlanToTargetPose] Already at target position");
    return true;
  }
  
  // 设置目标位姿和关节角
  ee_pose_goal_ = target_pose_eigen;
  arm_joint_goal_ = target_joint_pos;
  if (use_manual_joint6){
    arm_joint_goal_[5] = joint6_pos;
  }
  
  // 计算轨迹时间（基于距离和速度）
  plan_max_tick_ = static_cast<long unsigned int>(
      (ee_pose_goal_ - start_ee_pose).block<3, 1>(0, 3).norm() /
      average_move_speed_ / control_period_);
  plan_max_tick_ = std::max(100uL, plan_max_tick_);
  
  // 生成轨迹
  lazyPlan(start_joint_pos, arm_joint_goal_, plan_max_tick_);
  
  // 切换到运动状态
  setArmControlFsm(ArmControlFsm::PlanMove);
  
  ROS_INFO("[PlanToTargetPose] Motion plan generated. Duration: %lu ticks (%.2f seconds)",
           plan_max_tick_, plan_max_tick_ * control_period_);
  
  return true;
}
void ArmController::imuCallback(const sensor_msgs::Imu::ConstPtr& imu) {
  // tf2::Vector3 gravity_world(0, 0, -9.81);
  // tf2::Quaternion orientation;
  // tf2::fromMsg(imu->orientation, orientation);
  // tf2::Vector3 gravity_imu = tf2::quatRotate(orientation.inverse(), gravity_world);
  // arm_model_->_gravity[0] = gravity_imu.x();
  // arm_model_->_gravity[1] = gravity_imu.y();
  // arm_model_->_gravity[2] = gravity_imu.z();
}

void ArmController::executeProcessCallback(const std_msgs::Float64::ConstPtr& msg) {
  execute_process_ = msg->data;
  // ROS_INFO("[ExecuteProcess] Control signal updated: %.1f (%s)", 
  //          execute_process_, execute_process_ >= 1.0 ? "ENABLED" : "DISABLED");
}

bool ArmController::controlJoint6AndGripper(double gripper_pos, double joint6_pos) {
  // ROS_INFO("[ControlJoint6AndGripper] Setting Joint6 to: %.3f rad (%.1f deg), Gripper to: %.3f",
  //          joint6_pos, joint6_pos * 180.0 / M_PI, gripper_pos);
  
  // // 检查 Joint6 角度范围
  // if (joint6_pos < -3.14 || joint6_pos > 3.14) {
  //   ROS_ERROR("[ControlJoint6AndGripper] Invalid Joint6 position: %.3f rad (valid range: -3.14 to 3.14)",
  //             joint6_pos);
  //   return false;
  // }
  
  // // 检查夹爪位置范围
  // if (gripper_pos < -0.85 || gripper_pos > 0.0) {
  //   ROS_WARN("[ControlJoint6AndGripper] Gripper position %.3f is out of typical range (0.0 to -0.85)",
  //            gripper_pos);
  // }
  
  // // 如果不在 Arrived 或 PlanMove 状态，需要初始化机械臂位置
  // if (arm_control_fsm_ != ArmControlFsm::Arrived && 
  //     arm_control_fsm_ != ArmControlFsm::PlanMove) {
  //   arm_control_joint_pos_ = low_state_.getQ();  // 保持当前位置
  //   arm_control_joint_vel_.setZero();
  //   ROS_INFO("[ControlJoint6AndGripper] Initialized arm position from current state");
  // }
  
  // // 设置夹爪目标
  // gripper_goal_ = gripper_pos;
  
  // // 设置 Joint6 目标
  // arm_control_joint_pos_[5] = joint6_pos;  // Joint6 是索引 5
  
  // // 切换到 Arrived 状态
  // setArmControlFsm(ArmControlFsm::Arrived);
  
  // ROS_INFO("[ControlJoint6AndGripper] Control command sent successfully");
  return false;
}

std::vector<geometry_msgs::PoseStamped> ArmController::sortPosesByDistanceToPoint(
    const std::vector<geometry_msgs::PoseStamped>& poses,
    const Eigen::Vector3d& reference_point,
    const Eigen::Vector3d& line_direction,
    double target_distance) {
  
  // 复制输入列表
  std::vector<geometry_msgs::PoseStamped> sorted_poses = poses;
  
  // 按照到参考点的距离与 target_distance 的差值排序
  // 差值越小，说明距离越接近 target_distance
  std::sort(sorted_poses.begin(), sorted_poses.end(),
    [&reference_point, target_distance](const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b) {
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
  
  ROS_INFO("[SortPoses] Sorted %zu poses by distance to reference point (%.3f, %.3f, %.3f)",
           sorted_poses.size(), reference_point.x(), reference_point.y(), reference_point.z());
  ROS_INFO("[SortPoses] Target distance from reference: %.3f m",
           target_distance);
  
  // 输出前几个点的距离信息
  for (size_t i = 0; i < std::min(size_t(5), sorted_poses.size()); ++i) {
    Eigen::Vector3d point(sorted_poses[i].pose.position.x,
                         sorted_poses[i].pose.position.y,
                         sorted_poses[i].pose.position.z);
    double dist_to_ref = (point - reference_point).norm();
    double diff = std::abs(dist_to_ref - target_distance);
    
    ROS_INFO("  [%zu] Pose at (%.3f, %.3f, %.3f), distance to ref: %.3f m, diff from target: %.3f m",
             i, point.x(), point.y(), point.z(), dist_to_ref, diff);
  }
  
  return sorted_poses;
}

// ============================================================================
// 辅助函数：移动到默认点（用于安全过渡）
// ============================================================================

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

bool ArmController::executeMotionToTarget(const geometry_msgs::Pose& target_pose,
                                          double pitch,
                                          double roll,
                                          double timeout_seconds) {
  // ROS_INFO("[ExecuteMotionToTarget] Starting motion to target position: (%.3f, %.3f, %.3f)",
  //          target_pose.position.x, target_pose.position.y, target_pose.position.z);
  
  // 1. 规划到目标位姿
  // 移动加爪
  ros::Rate rate(50);  // 10Hz
  double gripper_pos = pitch;

  if (gripper_pos < -0.85 || gripper_pos > 0.0) {
    ROS_WARN("[ControlJoint6AndGripper] Gripper position %.3f is out of typical range (0.0 to -0.85)",
             gripper_pos);
  }
  double curr_gripper_pos = low_state_. getGripperQ();
  for(int i{0};i < 50; ++i){
    gripper_goal_=curr_gripper_pos + static_cast<double>(i)/50*(pitch - curr_gripper_pos);
    rate.sleep();
  }
  if (roll < -3.14 || roll > 3.14) {
    ROS_ERROR("[ControlJoint6AndGripper] Invalid Joint6 position: %.3f rad (valid range: -3.14 to 3.14)",
    roll);
    return false;
  }

  bool success_plan = planToTargetPose(target_pose, roll, true);
  
  if (!success_plan) {
    // ROS_ERROR("[ExecuteMotionToTarget] Failed to plan motion to target pose");
    return false;
  }

  // // 2. 等待机械臂到达目标位置
  // // ROS_INFO("[ExecuteMotionToTarget] Waiting for arm to reach target position...");
  // int timeout_count = 0;
  // int max_timeout = static_cast<int>(timeout_seconds * 10);  // 转换为循环次数
  
  // while (ros::ok() && arm_control_fsm_ != ArmControlFsm::Arrived && timeout_count < max_timeout) {
  //   rate.sleep();
  //   timeout_count++;
  // }
  
  // if (timeout_count >= max_timeout) {
  //   // ROS_WARN("[ExecuteMotionToTarget] Timeout waiting for arm to arrive (%.1f seconds)", timeout_seconds);
  //   return false;
  // }
  
  // // ROS_INFO("[ExecuteMotionToTarget] Arm arrived at target position");
  
  // // 3. 根据 execute_process_ 信号决定是否执行夹爪控制
  // // ROS_INFO("[ExecuteMotionToTarget] execute_process_ status: %s", 
  // //          execute_process_ ? "ENABLED" : "DISABLED");
  
  // if (execute_process_ >= 1.0) {
  //   // ROS_INFO("[ExecuteMotionToTarget] Executing gripper control (pitch: %.3f rad, roll: %.3f rad)...",
  //   //          pitch, roll);
  //   bool success_gripper = controlJoint6AndGripper(pitch, roll);
    
  //   if (!success_gripper) {
  //     // ROS_ERROR("[ExecuteMotionToTarget] Gripper control failed");
  //     return false;
  //   }
    
  //   // ROS_INFO("[ExecuteMotionToTarget] Gripper control command sent. Waiting 1 second...");
    
  //   // 等待夹爪动作执行 1 秒
  //   ros::Duration(1.0).sleep();
    
  //   // ROS_INFO("[ExecuteMotionToTarget] Gripper action completed");
  // } else {
  //   // ROS_INFO("[ExecuteMotionToTarget] Skipping gripper control (execute_process is disabled)");
  // }
  
  // ROS_INFO("[ExecuteMotionToTarget] Motion execution completed successfully");
  return true;
}

// ============================================================================
// Line3D 结构体成员函数实现
// ============================================================================

std::vector<Eigen::Vector3d> Line3D::samplePoints(double t_start, double t_end,
                                                    int num_samples) const {
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

// ============================================================================
// ArmController 直线旋转相关函数实现
// ============================================================================

Line3D ArmController::rotateLine(const Line3D& line,
                                  const Eigen::Vector3d& rotation_center,
                                  const Eigen::Vector3d& rotation_axis,
                                  double angle_rad) const {
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

// ============================================================================
// ArmController 纯几何变换函数实现
// ============================================================================

Line3D ArmController::translateLineGeometry(const Line3D& line,
                                            const Eigen::Vector3d& direction,
                                            double distance) const {
  Line3D translated_line;
  
  // 计算平移向量：方向单位化 * 距离
  Eigen::Vector3d translation = direction.normalized() * distance;
  
  // 平移直线上的点
  translated_line.point = line.point + translation;
  
  // 保持方向向量不变
  translated_line.direction = line.direction;
  
  return translated_line;
}

std::vector<geometry_msgs::PoseStamped> ArmController::sampleAndCheckReachability(
    const Line3D& line,
    double t_start,
    double t_end,
    int num_samples,
    double& pitch,
    double& roll) {
  
  std::vector<geometry_msgs::PoseStamped> reachable_poses;
  
  // 初始化输出参数
  pitch = 0.0;
  roll = 0.0;
  
  // 在直线上采样点
  std::vector<Eigen::Vector3d> sampled_points = 
      line.samplePoints(t_start, t_end, num_samples);
  
  std::cout << "[Sample & Check] Sampling " << num_samples 
            << " points on line from t=" << t_start << " to t=" << t_end << std::endl;
  
  // 创建 Pose 并检查逆运动学解
  for (size_t i = 0; i < sampled_points.size(); ++i) {
    geometry_msgs::Pose pose;
    pose.position.x = sampled_points[i].x();
    pose.position.y = sampled_points[i].y();
    pose.position.z = sampled_points[i].z();
    pose.orientation.w = 1.0;  // 默认姿态
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;

    // 检查是否有逆运动学解
    arm_controller_srvs::Plan::Request req;
    arm_controller_srvs::Plan::Response res;
    req.target_pose = pose;
    if((IsPlanServer(req, res)) && res.call_success) {
      std::cout << "  Point[" << i << "]: (" << pose.position.x << ", " 
                << pose.position.y << ", " << pose.position.z << ") is OK" << std::endl;
      
      // 转换为 PoseStamped 并添加到输出列表
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = ros::Time::now();
      pose_stamped.header.frame_id = "link00";
      pose_stamped.pose = pose;
      reachable_poses.push_back(pose_stamped);
    } else {
      std::cout << "  Point[" << i << "]: (" << pose.position.x << ", " 
                << pose.position.y << ", " << pose.position.z << ") is NOT OK" << std::endl;
    }
  }
  
  std::cout << "  Total " << reachable_poses.size() << " reachable poses found." << std::endl;
  
  // 计算相机姿态
  double test_yaw;
  if(calculateCameraOrientation(line.direction, pitch, roll, test_yaw, 0.055)) {
    // Pitch 和 Roll 已经在 calculateCameraOrientation 函数内部输出了
  }
  
  return reachable_poses;
}

// ============================================================================
// ArmController 直线平移相关函数实现（保留旧接口以兼容现有代码）
// ============================================================================

Line3D ArmController::translateLine(const Line3D& line,
                                  const Eigen::Vector3d& direction,
                                  double distance,
                                  double t_start,
                                  double t_end,
                                  int num_samples,
                                  std::vector<geometry_msgs::PoseStamped>& reachable_poses,
                                  double &pitch,
                                  double &roll) {
  
  std::cout << "[Line Translation] Generating " << num_samples 
            << " translated lines in direction: " << direction.transpose()
            << " with distance: " << distance << " m"
            << std::endl;
  
  // 1. 纯几何变换：平移直线
  Line3D translated_line = translateLineGeometry(line, direction, distance);
  
  // 2. 采样并检测可达性
  reachable_poses = sampleAndCheckReachability(translated_line, t_start, t_end, num_samples, pitch, roll);
  
  // 输出成功信息
  std::cout << "translateline,distance:" << distance << ": SUCCESS\n" << std::endl;
  
  return translated_line;
}

// ============================================================================
// ArmController 相机姿态计算相关函数实现
// ============================================================================

bool ArmController::calculateCameraOrientation(const Eigen::Vector3d& camera_direction,
                                               double& pitch,
                                               double& roll,
                                               double& yaw,
                                               double radius) const {
  // 归一化输入方向
  Eigen::Vector3d d = camera_direction.normalized();
  
  // =============================================================================
  // 暴力搜索方法：遍历所有 pitch 和 roll 组合
  // 
  // 优点：
  // 1. 简单直观，不需要复杂的数学推导
  // 2. 一定能找到最佳解（如果存在）
  // 3. 可以直接验证结果
  // =============================================================================
  
  // 相机相对末端的固定位置偏移（从 URDF）
  const Eigen::Vector3d CAM_POS_OFFSET(0.0389, 0, -0.0389);
  const double CAM_PITCH_OFFSET = 0.7854;  // 45° 相机固定姿态偏移
  
  // 步骤 1: 计算目标坐标（单位向量 × 半径）
  Eigen::Vector3d target_pos = d * radius;
  
  // 步骤 2: 暴力搜索最佳 pitch 和 roll
  double best_pitch = 0.0, best_roll = 0.0;
  double min_error = 1e10;
  
  // 搜索精度：1度
  const double step = 1.0 * M_PI / 180.0;
  const double pitch_min = -M_PI / 2.0;
  const double pitch_max = 0.0;
  const double roll_min = -M_PI / 2.0;
  const double roll_max = M_PI / 2.0;
  
  // 遍历 pitch
  for (double p = pitch_min; p <= pitch_max; p += step) {
    double cp = std::cos(p), sp = std::sin(p);
    
    // 遍历 roll
    for (double r = roll_min; r <= roll_max; r += step) {
      double cr = std::cos(r), sr = std::sin(r);
      
      // 计算相机位置：末端旋转后，相机的位置
      // R_end = Ry(pitch) * Rx(roll)
      // cam_pos = R_end * CAM_POS_OFFSET
      
      // Ry(p) * Rx(r) = [cp    sp*sr   sp*cr ]
      //                 [0     cr     -sr   ]
      //                 [-sp   cp*sr   cp*cr]
      
      Eigen::Vector3d cam_pos(
        cp * CAM_POS_OFFSET.x() + sp*sr * CAM_POS_OFFSET.y() + sp*cr * CAM_POS_OFFSET.z(),
        0  * CAM_POS_OFFSET.x() + cr    * CAM_POS_OFFSET.y() - sr    * CAM_POS_OFFSET.z(),
        -sp * CAM_POS_OFFSET.x() + cp*sr * CAM_POS_OFFSET.y() + cp*cr * CAM_POS_OFFSET.z()
      );
      
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
  yaw = 0.0;  // yaw 不可控，设为 0
  
  // 验证：计算实际相机位置
  double cp = std::cos(pitch), sp = std::sin(pitch);
  double cr = std::cos(roll), sr = std::sin(roll);
  
  Eigen::Vector3d actual_cam_pos(
    cp * CAM_POS_OFFSET.x() + sp*sr * CAM_POS_OFFSET.y() + sp*cr * CAM_POS_OFFSET.z(),
    0  * CAM_POS_OFFSET.x() + cr    * CAM_POS_OFFSET.y() - sr    * CAM_POS_OFFSET.z(),
    -sp * CAM_POS_OFFSET.x() + cp*sr * CAM_POS_OFFSET.y() + cp*cr * CAM_POS_OFFSET.z()
  );
  
  double final_error = (actual_cam_pos - target_pos).norm();
  
  // 输出结果
  // std::cout << "[Camera Position - Brute Force Search]" << std::endl;
  // std::cout << "  Target position:     " << target_pos.transpose() << std::endl;
  // std::cout << "  Calculated position: " << actual_cam_pos.transpose() << std::endl;
  // std::cout << "  Error: " << final_error << " m" << std::endl;
  std::cout << "  => End-effector Pitch: " << (pitch * 180.0 / M_PI) << " deg" <<"=="<< pitch<< std::endl;
  std::cout << "  => End-effector Roll:  " << (roll * 180.0 / M_PI) << " deg" <<"=="<< roll<< std::endl;
  
  if (final_error > 0.01) {  // 1cm 误差
    std::cout << "\n  [Warning] Large error! Target may not be achievable." << std::endl;
    return false;
  }
  
  return true;
}


std::vector<std::pair<Line3D, std::vector<Eigen::Vector3d>>>
ArmController::generateRotatedLinesWithSamples(
    const Line3D& line,
    const Eigen::Vector3d& rotation_center,
    const Eigen::Vector3d& rotation_axis,
    int num_rotations,
    double angle_step,
    double t_start,
    double t_end,
    int num_samples,
    std::vector<geometry_msgs::PoseStamped>& reachable_poses,
    double& pitch,
    double& roll,
    ros::Publisher* line_publisher,
    int viz_points) {
  
  std::vector<std::pair<Line3D, std::vector<Eigen::Vector3d>>> results;
  
  // 清空输出参数
  reachable_poses.clear();
  pitch = 0.0;
  roll = 0.0;
  
  std::cout << "[Line Rotation] Generating " << num_rotations 
            << " rotated lines around axis: " << rotation_axis.transpose()
            << " with angle step: " << (angle_step * 180.0 / M_PI) << " deg"
            << std::endl;
  
  for (int i = 1; i <= num_rotations; i++) {
    double angle = i * angle_step;
    
    // 1. 纯几何变换：旋转直线
    Line3D rotated_line = rotateLine(line, rotation_center, rotation_axis, angle);
    
    // 2. 采样并检测可达性
    std::vector<geometry_msgs::PoseStamped> current_reachable_poses = 
        sampleAndCheckReachability(rotated_line, t_start, t_end, num_samples, pitch, roll);
    
    // 将当前旋转角度的可达点添加到总列表
    reachable_poses.insert(reachable_poses.end(), 
                          current_reachable_poses.begin(), 
                          current_reachable_poses.end());
    
    // 获取采样点用于返回结果
    std::vector<Eigen::Vector3d> sampled_points = 
        rotated_line.samplePoints(t_start, t_end, num_samples);
    
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

// ============================================================================
// ZED Link to Link00 TF 广播控制
// ============================================================================

bool ArmController::zedLinkToLink00Server(
    arm_controller_srvs::zedlinktolink00::Request& req,
    arm_controller_srvs::zedlinktolink00::Response& res) {
  
  res.call_success = false;
  
  if (req.enable) {
    ROS_INFO("[ZedLinkToLink00] Querying TF transform: link00 -> estimated_object");
    
    try {
      // 查询 TF 变换：从 link00 到 estimated_object
      geometry_msgs::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_.lookupTransform(
          "link00",              // 目标坐标系（相对于这个坐标系表示）
          "estimated_object",    // 源坐标系（要查询的坐标系）
          ros::Time(0),          // 获取最新的变换
          ros::Duration(1.0)     // 超时时间：1秒
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
      ROS_INFO("  Orientation (quaternion): x=%.4f, y=%.4f, z=%.4f, w=%.4f", 
               ori_x, ori_y, ori_z, ori_w);
      
      // 转换为 RPY 角度（可选）
      tf::Quaternion quat(ori_x, ori_y, ori_z, ori_w);
      tf::Matrix3x3 mat(quat);
      double roll, pitch, yaw;
      mat.getRPY(roll, pitch, yaw);
      
      ROS_INFO("  Orientation (RPY): roll=%.4f (%.2f°), pitch=%.4f (%.2f°), yaw=%.4f (%.2f°)",
               roll, roll * 180.0 / M_PI,
               pitch, pitch * 180.0 / M_PI,
               yaw, yaw * 180.0 / M_PI);
      
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
      ROS_ERROR("  You can check available frames with: rosrun tf tf_echo link00 estimated_object");
      res.call_success = false;
    }
    
  } else {
    ROS_INFO("[ZedLinkToLink00] Service disabled (enable=false)");
    res.call_success = true;
  }
  
  return true;
}

}  // namespace arm_controller
