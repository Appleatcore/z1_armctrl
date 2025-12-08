#ifndef PINOCCHIO_IK_H_
#define PINOCCHIO_IK_H_

// // 关键：在包含任何头文件之前定义 Boost 限制宏
// #ifndef BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
// #define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
// #endif

// #ifndef BOOST_MPL_LIMIT_LIST_SIZE
// #define BOOST_MPL_LIMIT_LIST_SIZE 30
// #endif

// // Pinocchio 相关宏
// #ifndef PINOCCHIO_WITH_URDFDOM
// #define PINOCCHIO_WITH_URDFDOM
// #endif

// #include <ros/ros.h>
#include <Eigen/Dense>
#include <pinocchio/fwd.hpp>
#include <random>
#include <string>
#include <vector>

// 修复冲突：Unitree SDK 定义了 I3 和 I6 宏
#ifdef I3
#undef I3
#endif

#ifdef I6
#undef I6
#endif

// Pinocchio 头文件（仅在此处包含，不在其他任何地方）
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace arm_controller {

/**
 * @brief Pinocchio 运动学求解器
 *
 * 封装了基于 Pinocchio 库的正向/逆向运动学计算
 * 支持直接求解末端摄像头的目标位姿
 */
class PinocchioIK {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief 构造函数
   * @param urdf_path URDF 文件路径
   * @param end_frame_name 末端执行器坐标系名称（如 "camera_optical_frame"）
   */
  explicit PinocchioIK(const std::string& urdf_path, const std::string& end_frame_name = "camera_optical_frame");

  ~PinocchioIK() = default;

  /**
   * @brief 正向运动学：计算给定关节角下的末端位姿
   * @param q 关节角度 (6x1 向量)
   * @param end_pose 输出：末端位姿 (4x4 齐次变换矩阵)
   * @return true 如果计算成功
   */
  bool forwardKinematics(const Eigen::Matrix<double, 6, 1>& q, Eigen::Matrix4d& end_pose);

  /**
   * @brief 逆向运动学：求解达到目标位姿的关节角
   * @param target_pose 目标位姿 (4x4 齐次变换矩阵)
   * @param q_init 初始关节角 (6x1 向量)
   * @param q_result 输出：求解的关节角 (6x1 向量)
   * @param max_iter 最大迭代次数（默认 1000）
   * @param eps 收敛误差阈值（默认 1e-4）
   * @return true 如果求解成功
   */
  bool inverseKinematics(const Eigen::Matrix4d& target_pose, const Eigen::Matrix<double, 6, 1>& q_init, Eigen::Matrix<double, 6, 1>& q_result, int max_iter = 1000, double eps = 1e-4);

  /**
   * @brief 检查给定关节角是否在安全范围内
   * @param q 关节角度 (6x1 向量)
   * @return true 如果所有关节在限位内
   */
  bool checkJointLimits(const Eigen::Matrix<double, 6, 1>& q) const;

  /**
   * @brief 获取模型的关节数量
   */
  int getNumJoints() const { return model_.njoints; }

  /**
   * @brief 获取末端坐标系名称
   */
  std::string getEndFrameName() const { return end_frame_name_; }

 private:
  pinocchio::Model model_;              // Pinocchio 机器人模型
  pinocchio::Data data_;                // Pinocchio 计算数据缓存
  pinocchio::FrameIndex end_frame_id_;  // 末端坐标系 ID
  std::string end_frame_name_;          // 末端坐标系名称

  bool is_initialized_;  // 是否成功初始化

  /**
   * @brief 内部函数：限制关节角在安全范围内
   */
  void clampToJointLimits(Eigen::VectorXd& q) const;
  // 内部单次求解函数
  bool solveCLIK(const Eigen::Matrix4d& target_pose, const Eigen::Matrix<double, 6, 1>& q_guess, Eigen::Matrix<double, 6, 1>& q_out, int max_iter, double eps);
};

}  // namespace arm_controller

#endif  // PINOCCHIO_IK_H_