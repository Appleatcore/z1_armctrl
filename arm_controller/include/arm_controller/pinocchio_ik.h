#ifndef PINOCCHIO_IK_H_
#define PINOCCHIO_IK_H_

#include <Eigen/Dense>
#include <pinocchio/fwd.hpp>
#include <string>
#include <vector>

// 修复冲突宏
#ifdef I3
#undef I3
#endif
#ifdef I6
#undef I6
#endif

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace arm_controller {

class PinocchioIK {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit PinocchioIK(const std::string& urdf_path, const std::string& end_frame_name = "camera_optical_frame");
  ~PinocchioIK() = default;

  /**
   * @brief 正向运动学
   * @param q 关节角度 (VectorXd, 维度需匹配模型，如 6 或 7)
   */
  bool forwardKinematics(const Eigen::VectorXd& q, Eigen::Matrix4d& end_pose);

  /**
   * @brief 逆向运动学 (通用自由度版)
   * @param q_init 初始关节角 (VectorXd)
   * @param q_result 输出关节角 (VectorXd)
   * @param weights 任务空间权重 (固定为 6x1: xyz + rpy)
   */
  bool inverseKinematics(const Eigen::Matrix4d& target_pose, const Eigen::VectorXd& q_init, Eigen::VectorXd& q_result, const Eigen::Matrix<double, 6, 1>& weights = Eigen::Matrix<double, 6, 1>::Ones(), int max_iter = 1000,
                         double eps = 1e-4);

  bool checkJointLimits(const Eigen::VectorXd& q) const;

  int getNumJoints() const { return model_.nq; }
  std::string getEndFrameName() const { return end_frame_name_; }
  /**
   * @brief 设置特定关节的限位
   * @param joint_index 关节在 q 向量中的索引 (通常从 0 开始，0 代表关节1)
   * @param max_val 角度 (弧度)
   */
  void setJointLimitMin(int joint_index, double min_val);

  void setJointLimitMax(int joint_index, double max_val);

 private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex end_frame_id_;
  std::string end_frame_name_;
  bool is_initialized_;

  void clampToJointLimits(Eigen::VectorXd& q) const;

  // 内部求解器
  bool solveCLIK(const Eigen::Matrix4d& target_pose, const Eigen::VectorXd& q_guess, Eigen::VectorXd& q_out, const Eigen::Matrix<double, 6, 1>& weights, int max_iter, double eps);
};

}  // namespace arm_controller

#endif  // PINOCCHIO_IK_H_