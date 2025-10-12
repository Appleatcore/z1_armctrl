#include "arm_controller/arm_controller.h"

#include "arm_controller/geometry_utils.h"

namespace arm_controller {

ArmController::ArmController(const ros::NodeHandle& nh) : nh_(nh) {
  arm_api_ = std::make_unique<ArmApi>();
  arm_model_ = std::make_unique<Z1ArmModel>();
  // arm_model_->_jointQMin[3] = -1.74;
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
  default_kp_ = {15, 7.5, 7.5, 5, 10, 2.5};
  default_kd_ = {1500, 500, 500, 500, 1200, 500};
  // Waiting for establishing connection with the manipulator
  while (!low_state_.arm_connected) {
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
  // 设置夹爪增益
  low_cmd_.setGripperGain();  // 使用默认增益
  // Planning
  KJointHome_ << 0, 0, -0.005, -0.074, 0, 0;
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
    double rotation_angle_deg, sample_start, sample_end;
    double origin_direction_x, origin_direction_y, origin_direction_z;
    double rotation_axis_x, rotation_axis_y, rotation_axis_z;
    int num_samples, num_rotations;
    double angle_step_deg;
    
    nh_.param("test/line_point_x", line_x, 1.0);
    nh_.param("test/line_point_y", line_y, 0.0);
    nh_.param("test/line_point_z", line_z, 0.15);
    nh_.param("test/origin_direction_x", origin_direction_x, 1.0);
    nh_.param("test/origin_direction_y", origin_direction_y, 0.0);
    nh_.param("test/origin_direction_z", origin_direction_z, 0.0);
    nh_.param("test/rotation_center_x", center_x, 1.0);
    nh_.param("test/rotation_center_y", center_y, 0.0);
    nh_.param("test/rotation_center_z", center_z, 0.15);
    nh_.param("test/rotation_axis_x", rotation_axis_x, 0.0);
    nh_.param("test/rotation_axis_y", rotation_axis_y, 0.0);
    nh_.param("test/rotation_axis_z", rotation_axis_z, 1.0);
    nh_.param("test/rotation_angle_deg", rotation_angle_deg, 45.0);
    nh_.param("test/sample_start", sample_start, -1.0);
    nh_.param("test/sample_end", sample_end, 0.0);
    nh_.param("test/num_samples", num_samples, 10);
    nh_.param("test/num_rotations", num_rotations, 1);
    nh_.param("test/angle_step_deg", angle_step_deg, 45.0);
    
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
    std::cout<<"------------MID LINE------------"<<std::endl;
    double angle_step_rad = 0.0;
    auto results = generateRotatedLinesWithSamples(
        original_line,
        rotation_center,
        rotation_axis,
        num_rotations,
        angle_step_rad,
        sample_start, sample_end,
        num_samples
    );

    //正转angle_step_deg度
    std::cout<<"------------HALF LINE------------"<<std::endl;
    double angle_step_rad_1 = angle_step_deg * M_PI / 180.0;
    auto results_1 = generateRotatedLinesWithSamples(
        original_line,
        rotation_center,
        rotation_axis,
        num_rotations,
        angle_step_rad_1,
        sample_start, sample_end,
        num_samples
    );


    //反转angle_step_deg度
    std::cout<<"------------HALF2 LINE------------"<<std::endl;
    double angle_step_rad_2 = (360.0-angle_step_deg) * M_PI / 180.0;
    auto results_2 = generateRotatedLinesWithSamples(
        original_line,
        rotation_center,
        rotation_axis,
        num_rotations,
        angle_step_rad_2,
        sample_start, sample_end,
        num_samples
    );

    //平移distance_1
    std::cout<<"------------OUT LINE------------"<<std::endl;
    double distance_1 = 0.3;
    auto results_3 = translateLine(
        original_line, Eigen::Vector3d(0.0, 1.0, 0.0), 
        distance_1, 
        sample_start, sample_end, 
        num_samples);

    //平移distance_2
    std::cout<<"------------OUT2 LINE------------"<<std::endl;
    double distance_2 = -0.3;
    auto results_4 = translateLine(
        original_line, Eigen::Vector3d(0.0, 1.0, 0.0), 
        distance_2, 
        sample_start, sample_end, 
        num_samples);
    
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
  imu_sub_ =
      nh_.subscribe("/aliengo/imu", 1, &ArmController::imuCallback, this);
}

void ArmController::initServers() {
  plan_server_ = nh_.advertiseService("plan", &ArmController::planServer, this);
  back2home_server_ = nh_.advertiseService(
      "back_to_home", &ArmController::back2HomeServer, this);
  check_pose_in_workspace_server_ = nh_.advertiseService(
      "check_pose_in_workspace", &ArmController::isInWorkspaceServer, this);
  search_plan_server_ = nh_.advertiseService(
      "search_plan", &ArmController::searchPlanServer, this);
  js_control_server_ = nh_.advertiseService(
      "joy_stick_control", &ArmController::jsControlServer, this);
  gripper_control_server_ = nh_.advertiseService(
      "gripper_control", &ArmController::gripperControlServer, this);
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
    target_pose.block<3, 1>(0, 3) =
        camera_target_pose.block<3, 1>(0, 3) -
        camera_target_pose.block<3, 3>(0, 0) * kCameraPosBias_E_;
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

bool ArmController::gripperControlServer(
    arm_controller_srvs::GripperControl::Request& req,
    arm_controller_srvs::GripperControl::Response& res) {
  res.call_success = false;
  
  // 检查 Joint6 角度范围
  if (req.joint6_pos < -3.14 || req.joint6_pos > 3.14) {
    std::cout << "[Gripper Control] Invalid Joint6 position: " << req.joint6_pos
              << " rad (valid range: -3.14 to 3.14)" << std::endl;
    return true;
  }
  
  // 如果不在 Arrived 或 PlanMove 状态，需要初始化机械臂位置
  if (arm_control_fsm_ != ArmControlFsm::Arrived && 
      arm_control_fsm_ != ArmControlFsm::PlanMove) {
    arm_control_joint_pos_ = low_state_.getQ();  // 保持当前位置
    arm_control_joint_vel_.setZero();
  }
  
  // 设置夹爪目标
  gripper_goal_ = req.gripper_pos;
  
  // 设置 Joint6 目标
  arm_control_joint_pos_[5] = req.joint6_pos;  // Joint6 是索引 5
  
  std::cout << "[Gripper Control] Setting Joint6 to: " << req.joint6_pos 
            << " rad (" << (req.joint6_pos * 180.0 / 3.14159) << " deg), "
            << "Gripper to: " << gripper_goal_ << std::endl;
  
  res.call_success = true;
  setArmControlFsm(ArmControlFsm::Arrived);
  return true;
}

void ArmController::imuCallback(const sensor_msgs::Imu::ConstPtr& imu) {
  // Note: _gravity is protected member, cannot access directly
  // If needed, you can modify the Z1 SDK to add a public setter
  // arm_model_->_gravity[0] = -imu->linear_acceleration.x;
  // arm_model_->_gravity[1] = -imu->linear_acceleration.y;
  // arm_model_->_gravity[2] = -imu->linear_acceleration.z;
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
// ArmController 直线平移相关函数实现
// ============================================================================

Line3D ArmController::translateLine(const Line3D& line,
                                  const Eigen::Vector3d& direction,
                                  double distance,
                                  double t_start,
                                  double t_end,
                                  int num_samples) {
  Line3D translated_line;
  
  std::cout << "[Line Translation] Generating " << num_samples 
  << " translated lines in direction: " << direction.transpose()
  << " with distance: " << distance << " m"
  << std::endl;
  // 计算平移向量：方向单位化 * 距离
  Eigen::Vector3d translation = direction.normalized() * distance;
  
  // 平移直线上的点
  translated_line.point = line.point + translation;
  
  // 保持方向向量不变
  translated_line.direction = line.direction;

  // 在旋转后的直线上采样点
  std::vector<Eigen::Vector3d> sampled_points = 
  translated_line.samplePoints(t_start, t_end, num_samples);

  // 创建 Pose 并赋值采样点
  std::vector<geometry_msgs::Pose> sample_poses;
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
  if((IsPlanServer(req, res)) && res.call_success){
    std::cout << "  Point[" << i << "]: (" << pose.position.x << ", " 
              << pose.position.y << ", " << pose.position.z << ") is OK" << std::endl;
    sample_poses.push_back(pose);
  }
  }
  // std::cout << "Total " << sample_poses.size() << " poses created." << std::endl;

  // 保存结果
  // results.push_back(std::make_pair(translated_line, sampled_points));

  // 测试相机姿态计算
  // std::cout << "[Camera Orientation Test]" << std::endl;
  double test_pitch, test_roll, test_yaw;
  if(calculateCameraOrientation(translated_line.direction, test_pitch, test_roll, test_yaw, 0.055)) {
    std::cout << "translateline,distance:"<<distance<<": SUCCESS\n" << std::endl;
  }
  
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
  const double pitch_max = M_PI / 2.0;
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
    int num_samples) {
  
  std::vector<std::pair<Line3D, std::vector<Eigen::Vector3d>>> results;
  
  std::cout << "[Line Rotation] Generating " << num_rotations 
            << " rotated lines around axis: " << rotation_axis.transpose()
            << " with angle step: " << (angle_step * 180.0 / M_PI) << " deg"
            << std::endl;
  
  for (int i = 1; i <= num_rotations; i++) {
    double angle = i * angle_step;
    
    // 旋转直线
    Line3D rotated_line = rotateLine(line, rotation_center, rotation_axis, angle);
    
    // 在旋转后的直线上采样点
    std::vector<Eigen::Vector3d> sampled_points = 
        rotated_line.samplePoints(t_start, t_end, num_samples);
    
    // 创建 Pose 并赋值采样点
    std::vector<geometry_msgs::Pose> sample_poses;
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
      if((IsPlanServer(req, res)) && res.call_success){
        std::cout << "  Point[" << i << "]: (" << pose.position.x << ", " 
                  << pose.position.y << ", " << pose.position.z << ") is OK" << std::endl;
        sample_poses.push_back(pose);
      }
    }
    // std::cout << "Total " << sample_poses.size() << " poses created." << std::endl;

    // 保存结果
    results.push_back(std::make_pair(rotated_line, sampled_points));
    
    // 输出调试信息
    std::cout << "  [" << i << "] Angle: " << (angle * 180.0 / M_PI) << " deg, "
              << "Line point: " << rotated_line.point.transpose() << ", "
              << "\nDirection: " << rotated_line.direction.transpose() << ", "
              << "\nSamples: " << sampled_points.size() << std::endl;
    std::cout << std::endl;

    // 测试相机姿态计算
    // std::cout << "\n[Camera Orientation Test]" << std::endl ;
    double test_pitch, test_roll, test_yaw;
    if(calculateCameraOrientation(rotated_line.direction, test_pitch, test_roll, test_yaw, 0.055)) {
      std::cout << "rotated_line,angle_step:"<<angle_step<<": SUCCESS\n" << std::endl;
    }
  }
  
  return results;
}

}  // namespace arm_controller
