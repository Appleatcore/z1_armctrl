#include "arm_controller/pinocchio_ik.h"

#include <ros/package.h>
#include <ros/console.h>
#include <random> // 必须包含用于随机重试

namespace arm_controller {

PinocchioIK::PinocchioIK(const std::string& urdf_path, const std::string& end_frame_name) 
    : end_frame_name_(end_frame_name), is_initialized_(false) {
  try {
    // 1. 加载模型
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);

    // 2. 智能查找 Frame ID
    bool frame_found = false;

    if (model_.existFrame(end_frame_name_)) {
      for (pinocchio::FrameIndex i = 0; i < model_.nframes; ++i) {
        const auto& frame = model_.frames[i];
        if (frame.name == end_frame_name_) {
          // 这里的关键是：我们只想要 BODY (Link)，不要 JOINT
          if (frame.type == pinocchio::BODY) {
            end_frame_id_ = i;
            frame_found = true;
            ROS_INFO("[PinocchioIK] Locked on Link Frame '%s' (ID: %ld)", frame.name.c_str(), i);
            break;
          }
        }
      }

      // 没找到 BODY，尝试退路
      if (!frame_found) {
        ROS_WARN("[PinocchioIK] No BODY frame found for '%s', falling back to first match.", end_frame_name_.c_str());
        end_frame_id_ = model_.getFrameId(end_frame_name_);
        frame_found = true;
      }
    }

    if (!frame_found) {
      throw std::runtime_error("Frame [" + end_frame_name_ + "] not found in URDF!");
    }

    is_initialized_ = true;
    ROS_INFO("[PinocchioIK] Init Success. Model has %d joints, %d frames.", model_.njoints, model_.nframes);

  } catch (const std::exception& e) {
    ROS_ERROR("[PinocchioIK] Init Failed: %s", e.what());
    is_initialized_ = false;
  }
}

bool PinocchioIK::forwardKinematics(const Eigen::Matrix<double, 6, 1>& q, Eigen::Matrix4d& end_pose) {
  if (!is_initialized_) {
    ROS_ERROR("[PinocchioIK] Not initialized!");
    return false;
  }

  // 扩展到完整的关节空间
  Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model_.nq);
  q_full.head<6>() = q;

  // 计算正向运动学
  pinocchio::forwardKinematics(model_, data_, q_full);
  pinocchio::updateFramePlacements(model_, data_);

  // 提取末端位姿
  pinocchio::SE3 end_SE3 = data_.oMf[end_frame_id_];
  end_pose.block<3, 3>(0, 0) = end_SE3.rotation();
  end_pose.block<3, 1>(0, 3) = end_SE3.translation();
  end_pose.block<1, 4>(3, 0) << 0, 0, 0, 1;

  return true;
}

// ==================================================================================
// 外部接口：带随机重试逻辑
// ==================================================================================
bool PinocchioIK::inverseKinematics(const Eigen::Matrix4d& target_pose, const Eigen::Matrix<double, 6, 1>& q_init, Eigen::Matrix<double, 6, 1>& q_result, int max_iter, double eps) {
  if (!is_initialized_) {
    ROS_ERROR("[PinocchioIK] Not initialized!");
    return false;
  }

  // 1. 第一次尝试：使用传入的初始猜想 (Warm Start)
  // 如果当前位置离目标很近，这里通常会直接成功
  if (solveCLIK(target_pose, q_init, q_result, max_iter, eps)) {
    return true; 
  }

  // 2. 如果失败，进入“安全随机重试”模式
  // 仅在当前位置附近微扰，避免算出大幅度翻转的解
  ROS_WARN_THROTTLE(1.0, "[PinocchioIK] First attempt failed. Starting perturbation retries...");
  
  const int max_retries = 5;
  Eigen::Matrix<double, 6, 1> q_seed;
  
  // 随机数生成器
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.5, 0.5); // 扰动范围 +/- 0.5 rad (约 28度)

  for (int retry = 1; retry <= max_retries; ++retry) {
    // 生成扰动种子：在 q_init 基础上增加随机值
    q_seed = q_init;
    for(int i=0; i<6; ++i) q_seed[i] += dis(gen);
    
    // 限制种子在关节限位内
    Eigen::VectorXd q_seed_vec = Eigen::VectorXd::Zero(model_.nq);
    q_seed_vec.head<6>() = q_seed;
    clampToJointLimits(q_seed_vec);
    q_seed = q_seed_vec.head<6>();

    // 再次尝试求解
    if (solveCLIK(target_pose, q_seed, q_result, max_iter, eps)) {
      // 3. 安全性二次检查：防止解虽然收敛，但距离初始位置太远（例如翻转）
      double dist = (q_result - q_init).norm();
      if (dist < 3.0) { // 阈值可调，3.0 rad 是个经验值，防止机械臂"大翻身"
        ROS_INFO("[PinocchioIK] Retry %d success! (Perturbation solved local minima)", retry);
        return true;
      } else {
        ROS_WARN("[PinocchioIK] Retry %d converged but solution is too far (Unsafe dist: %.2f). Discarding.", retry, dist);
      }
    }
  }

  ROS_ERROR("[PinocchioIK] All %d retries failed to converge.", max_retries);
  return false;
}

// ==================================================================================
// 内部核心求解器：Damped Least Squares (DLS)
// ==================================================================================
bool PinocchioIK::solveCLIK(const Eigen::Matrix4d& target_pose, 
                            const Eigen::Matrix<double, 6, 1>& q_guess, 
                            Eigen::Matrix<double, 6, 1>& q_out, 
                            int max_iter, 
                            double eps) {
  
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
  q.head<6>() = q_guess;

  pinocchio::SE3 target_SE3(target_pose.block<3, 3>(0, 0), target_pose.block<3, 1>(0, 3));
  
  const double dt = 0.2;        // 步长，适当减小以保证稳定
  const double damping = 1e-3;  // 关键：阻尼系数 (lambda)，防止奇异点飞车

  for (int i = 0; i < max_iter; ++i) {
    // 1. 正向运动学
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);

    pinocchio::SE3 current_SE3 = data_.oMf[end_frame_id_];
    
    // 2. 计算误差 (使用 log6 在 Body Frame 计算流形误差)
    // actInv 计算的是 current^{-1} * target
    pinocchio::SE3 error_SE3 = current_SE3.actInv(target_SE3);
    Eigen::Matrix<double, 6, 1> error_vec = pinocchio::log6(error_SE3).toVector(); 

    // 3. 检查收敛
    if (error_vec.norm() < eps) {
      q_out = q.head<6>();
      return true;
    }

    // 4. 计算雅可比 (使用 LOCAL 坐标系，与上面的 error_SE3 对应)
    pinocchio::Data::Matrix6x J(6, model_.nv);
    J.setZero();
    pinocchio::computeFrameJacobian(model_, data_, q, end_frame_id_, pinocchio::LOCAL, J); 

    // 5. Damped Least Squares (DLS) 求解
    // 公式: dq = J^T * (J * J^T + lambda^2 * I)^-1 * error
    // 这比直接求 J 的伪逆 (J_pinv) 要稳定得多
    
    Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
    JJt.diagonal().array() += damping * damping; // 添加阻尼项
    
    // 求解线性方程 (比求逆更快更准)
    Eigen::VectorXd v = JJt.ldlt().solve(error_vec);
    Eigen::VectorXd dq = J.transpose() * v;

    // 6. 更新关节角
    q = pinocchio::integrate(model_, q, dq * dt);
    clampToJointLimits(q);
  }

  return false;
}

bool PinocchioIK::checkJointLimits(const Eigen::Matrix<double, 6, 1>& q) const {
  if (!is_initialized_) return false;

  for (int i = 0; i < 6; ++i) {
    // 增加一点点容差 (0.01)，防止浮点数临界值误报
    if (q[i] < model_.lowerPositionLimit[i] - 0.01 || q[i] > model_.upperPositionLimit[i] + 0.01) {
      ROS_WARN("[PinocchioIK] Joint %d out of limits: %.3f (limits: [%.3f, %.3f])", i, q[i], model_.lowerPositionLimit[i], model_.upperPositionLimit[i]);
      return false;
    }
  }
  return true;
}

void PinocchioIK::clampToJointLimits(Eigen::VectorXd& q) const {
  for (int i = 0; i < model_.nq; ++i) {
    q[i] = std::max(model_.lowerPositionLimit[i], std::min(q[i], model_.upperPositionLimit[i]));
  }
}

}  // namespace arm_controller