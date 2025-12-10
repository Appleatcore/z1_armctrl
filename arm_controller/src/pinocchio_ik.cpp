#include "arm_controller/pinocchio_ik.h"

#include <ros/console.h>
#include <ros/package.h>

#include <random>

namespace arm_controller {

PinocchioIK::PinocchioIK(const std::string& urdf_path, const std::string& end_frame_name) : end_frame_name_(end_frame_name), is_initialized_(false) {
  try {
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);

    bool frame_found = false;
    if (model_.existFrame(end_frame_name_)) {
      for (pinocchio::FrameIndex i = 0; i < model_.nframes; ++i) {
        const auto& frame = model_.frames[i];
        if (frame.name == end_frame_name_) {
          if (frame.type == pinocchio::BODY) {
            end_frame_id_ = i;
            frame_found = true;
            ROS_INFO("[PinocchioIK] Locked on Link Frame '%s' (ID: %ld)", frame.name.c_str(), i);
            break;
          }
        }
      }
      if (!frame_found) {
        ROS_WARN("[PinocchioIK] No BODY frame found for '%s', using fallback.", end_frame_name_.c_str());
        end_frame_id_ = model_.getFrameId(end_frame_name_);
        frame_found = true;
      }
    }

    if (!frame_found) throw std::runtime_error("Frame [" + end_frame_name_ + "] not found!");

    is_initialized_ = true;
    // 这里的 njoints 通常包含宇宙关节(Universe)，nq 才是配置变量数 (例如 6 或 7)
    ROS_INFO("[PinocchioIK] Init Success. DOF: %d", model_.nq);

  } catch (const std::exception& e) {
    ROS_ERROR("[PinocchioIK] Init Failed: %s", e.what());
    is_initialized_ = false;
  }
}

bool PinocchioIK::forwardKinematics(const Eigen::VectorXd& q, Eigen::Matrix4d& end_pose) {
  if (!is_initialized_) return false;

  // 维度安全检查
  if (q.size() != model_.nq) {
    ROS_ERROR_THROTTLE(1.0, "[PinocchioIK] FK dimension mismatch! Expected %d, got %ld", model_.nq, q.size());
    return false;
  }

  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  pinocchio::SE3 end_SE3 = data_.oMf[end_frame_id_];
  end_pose.block<3, 3>(0, 0) = end_SE3.rotation();
  end_pose.block<3, 1>(0, 3) = end_SE3.translation();
  end_pose.block<1, 4>(3, 0) << 0, 0, 0, 1;

  return true;
}

bool PinocchioIK::inverseKinematics(const Eigen::Matrix4d& target_pose, const Eigen::VectorXd& q_init, Eigen::VectorXd& q_result, const Eigen::Matrix<double, 6, 1>& weights, int max_iter, double eps) {
  if (!is_initialized_) return false;

  if (q_init.size() != model_.nq) {
    ROS_ERROR_THROTTLE(1.0, "[PinocchioIK] IK input dimension mismatch! Expected %d, got %ld", model_.nq, q_init.size());
    return false;
  }

  // 1. Warm Start
  if (solveCLIK(target_pose, q_init, q_result, weights, max_iter, eps)) {
    return true;
  }

  // 2. Random Retry
  ROS_WARN_THROTTLE(1.0, "[PinocchioIK] First attempt failed. Starting retries...");

  const int max_retries = 5;
  Eigen::VectorXd q_seed = q_init;  // 保持维度一致

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.5, 0.5);

  for (int retry = 1; retry <= max_retries; ++retry) {
    q_seed = q_init;
    // 【关键修改】遍历所有自由度（包括夹爪）进行扰动
    for (int i = 0; i < model_.nq; ++i) q_seed[i] += dis(gen);

    clampToJointLimits(q_seed);

    if (solveCLIK(target_pose, q_seed, q_result, weights, max_iter, eps)) {
      // double dist = (q_result - q_init).norm();
      // if (dist < 3.0) {
      //   ROS_INFO("[PinocchioIK] Retry %d success!", retry);
      return true;
      // } else {
      //   ROS_WARN("[PinocchioIK] Retry %d converged but unsafe dist: %.2f", retry, dist);
      // }
    }
  }

  return false;
}

bool PinocchioIK::solveCLIK(const Eigen::Matrix4d& target_pose, const Eigen::VectorXd& q_guess, Eigen::VectorXd& q_out, const Eigen::Matrix<double, 6, 1>& weights, int max_iter, double eps) {
  Eigen::VectorXd q = q_guess;  // 直接复制，维度自动匹配
  pinocchio::SE3 target_SE3(target_pose.block<3, 3>(0, 0), target_pose.block<3, 1>(0, 3));

  const double dt = 0.2;
  const double damping = 1e-3;

  for (int i = 0; i < max_iter; ++i) {
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);

    pinocchio::SE3 current_SE3 = data_.oMf[end_frame_id_];

    // 计算误差 (6D)
    pinocchio::SE3 error_SE3 = current_SE3.actInv(target_SE3);
    Eigen::Matrix<double, 6, 1> error_vec = pinocchio::log6(error_SE3).toVector();

    // 应用权重 (Masking)
    error_vec = error_vec.cwiseProduct(weights);

    if (error_vec.norm() < eps) {
      q_out = q;
      return true;
    }

    // 计算雅可比 (6 x nv)
    // 如果模型是7轴，J 就是 6x7 矩阵
    pinocchio::Data::Matrix6x J(6, model_.nv);
    J.setZero();
    pinocchio::computeFrameJacobian(model_, data_, q, end_frame_id_, pinocchio::LOCAL, J);

    // 应用权重到雅可比
    for (int k = 0; k < 6; ++k) {
      if (weights[k] < 1e-6) J.row(k).setZero();
    }

    // DLS 求解: dq = J^T * (J*J^T + lambda^2*I)^-1 * e
    // JJt 仍然是 6x6 矩阵，不受关节数影响
    Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
    JJt.diagonal().array() += damping * damping;

    Eigen::VectorXd v = JJt.ldlt().solve(error_vec);

    // dq 的维度将是 model_.nv (即 7)
    Eigen::VectorXd dq = J.transpose() * v;

    q = pinocchio::integrate(model_, q, dq * dt);
    q = q.cwiseMin(model_.upperPositionLimit).cwiseMax(model_.lowerPositionLimit);
    clampToJointLimits(q);
  }

  return false;
}

bool PinocchioIK::checkJointLimits(const Eigen::VectorXd& q) const {
  if (!is_initialized_) return false;
  if (q.size() != model_.nq) return false;

  // 【关键修改】遍历所有关节
  for (int i = 0; i < model_.nq; ++i) {
    if (q[i] < model_.lowerPositionLimit[i] - 0.01 || q[i] > model_.upperPositionLimit[i] + 0.01) {
      return false;
    }
  }
  return true;
}

void PinocchioIK::clampToJointLimits(Eigen::VectorXd& q) const {
  // 【关键修改】遍历所有关节
  for (int i = 0; i < model_.nq; ++i) {
    q[i] = std::max(model_.lowerPositionLimit[i], std::min(q[i], model_.upperPositionLimit[i]));
  }
}

}  // namespace arm_controller