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
      break;
    }
    case ArmControlFsm::PlanMove: {
      if (arm_control_tick_ == joint_pos_trajectory_.size()) {
        setArmControlFsm(ArmControlFsm::Arrived);
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
      setArmControlFsm(ArmControlFsm::PlanMove);
      res.call_success = true;
    }
  }
  return true;
}

bool ArmController::searchPlanServer(arm_controller_srvs::Plan::Request& req,
                                     arm_controller_srvs::Plan::Response& res) {
  res.call_success = false;
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

void ArmController::imuCallback(const sensor_msgs::Imu::ConstPtr& imu) {
  arm_model_->_gravity[0] = -imu->linear_acceleration.x;
  arm_model_->_gravity[1] = -imu->linear_acceleration.y;
  arm_model_->_gravity[2] = -imu->linear_acceleration.z;
}
}  // namespace arm_controller
