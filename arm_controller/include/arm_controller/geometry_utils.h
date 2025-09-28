#ifndef GEOMETRY_UTILS_H_
#define GEOMETRY_UTILS_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>

namespace arm_controller {

/**
 * @brief Quaternion to rpy
 * @details
 * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * @param quat
 * @param rpy
 */
void quatToRpy(const Eigen::Vector4d &quat, Eigen::Ref<Eigen::Vector3d> rpy);
/**
 * @brief Quaternion to rpy
 * @details
 * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * @param quat
 * @param rpy
 * @return The roll pitch and yaw representation of the quaternion
 */
[[nodiscard]] inline Eigen::Vector3d quatToRpy(const Eigen::Vector4d &quat) {
  Eigen::Vector3d rpy;
  quatToRpy(quat, rpy);
  return rpy;
};

/**
 * @brief Quaternion to rotation matrix
 * @param quat
 * @param rot
 */
void quatToRot(const Eigen::Vector4d &quat, Eigen::Ref<Eigen::Matrix3d> rot);
/**
 * @brief Quaternion to rotation matrix
 * @param quat
 * @param rot
 * @return The rotation matrix representation of the quaternion
 */
[[nodiscard]] inline Eigen::Matrix3d quatToRot(const Eigen::Vector4d &quat) {
  Eigen::Matrix3d rot;
  quatToRot(quat, rot);
  return rot;
};

/**
 * @brief Rpy to quaternion
 * @details
 * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * @param rpy
 * @param quat
 */
void rpyToQuat(const Eigen::Ref<const Eigen::Vector3d> &rpy,
               Eigen::Ref<Eigen::Vector4d> quat);
/**
 * @brief Rpy to quaternion
 * @details
 * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * @param rpy
 * @param quat
 * @return The quaternion representation of the roll pitch and yaw
 */
[[nodiscard]] inline Eigen::Vector4d rpyToQuat(
    const Eigen::Ref<const Eigen::Vector3d> &rpy) {
  Eigen::Vector4d quat;
  rpyToQuat(rpy, quat);
  return quat;
};

/**
 * @brief Rpy to rotation matrix
 * @details
 * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * @param rpy
 * @param rot
 */
void rpyToRot(const Eigen::Ref<const Eigen::Vector3d> &rpy,
              Eigen::Ref<Eigen::Matrix3d> rot);
/**
 * @brief Rpy to rotation matrix
 * @details
 * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * @param rpy
 * @param rot
 * @return The rotation matrix representation of the roll pitch and yaw
 */
[[nodiscard]] inline Eigen::Matrix3d rpyToRot(
    const Eigen::Ref<const Eigen::Vector3d> &rpy) {
  Eigen::Matrix3d rot=Eigen::Matrix3d::Identity();
  rpyToRot(rpy, rot);
  return rot;
};

/**
 * @brief Rotation matrix to roll, pitch and yaw
 * @param rot
 * @param rpy
 */
void rotToRpy(const Eigen::Ref<const Eigen::Matrix3d> &rot,
              Eigen::Ref<Eigen::Vector3d> rpy);
/**
 * @brief Rotation matrix to roll, pitch and yaw
 * @param rot
 * @param rpy
 * @return The roll pitch and yaw representation of the rotation matrix
 */
[[nodiscard]] inline Eigen::Vector3d rotToRpy(
    const Eigen::Ref<const Eigen::Matrix3d> &rot) {
  Eigen::Vector3d rpy;
  rotToRpy(rot, rpy);
  return rpy;
};

/**
 * @brief Rotation matrix to quaternion
 * @param rot rotation matrix
 * @param quat
 */
void rotToQuat(const Eigen::Ref<const Eigen::Matrix3d> &rot,
               Eigen::Ref<Eigen::Vector4d> quat);
/**
 * @brief Rotation matrix to quaternion
 * @param rot rotation matrix
 * @param quat
 * @return The quaternion representation of the rotation matrix
 */
[[nodiscard]] inline Eigen::Vector4d rotToQuat(
    const Eigen::Ref<const Eigen::Matrix3d> &rot) {
  Eigen::Vector4d quat;
  rotToQuat(rot, quat);
  return quat;
};

/**
 * @brief Calculate the product of two quaternions
 * @param p
 * @param q
 * @param res The result of the product of p and q
 */
void quatProduct(const Eigen::Ref<const Eigen::Vector4d> &p,
                 const Eigen::Ref<const Eigen::Vector4d> &q,
                 Eigen::Ref<Eigen::Vector4d> res);
/**
 * @brief Calculate the product of two quaternions
 * @param p
 * @param q
 * @return The result of the product of p and q
 */
[[nodiscard]] inline Eigen::Vector4d quatProduct(
    const Eigen::Ref<const Eigen::Vector4d> &p,
    const Eigen::Ref<const Eigen::Vector4d> &q) {
  Eigen::Vector4d res;
  quatProduct(p, q, res);
  return res;
};

/**
 * @brief Calculate the inverse of a quaternion
 * @param q
 * @return The inverse of the quaternion
 */
[[nodiscard]] inline Eigen::Vector4d quatInv(
    const Eigen::Ref<const Eigen::Vector4d> &q) {
  Eigen::Vector4d q_inv;
  q_inv(0) = q(0);
  q_inv.tail(3) = -q.tail(3);
  return q_inv;
};

/**
 * @brief Calculate the angle velocity under body frame by using rpy rate
 * @param rpy
 * @param rpy_rate
 * @param ang_vel_B
 */
void rpyRateToAngVelBody(const Eigen::Ref<const Eigen::Vector3d> &rpy,
                         const Eigen::Ref<const Eigen::Vector3d> &rpy_rate,
                         Eigen::Ref<Eigen::Vector3d> ang_vel_B);
/**
 * @brief Calculate the angle velocity under body frame by using rpy rate
 * @param rpy
 * @param rpy_rate
 * @return Eigen::Vector3d The angle velocity under body frame
 */
[[nodiscard]] inline Eigen::Vector3d rpyRateToAngVelBody(
    const Eigen::Ref<const Eigen::Vector3d> &rpy,
    const Eigen::Ref<const Eigen::Vector3d> &rpy_rate) {
  Eigen::Vector3d ang_vel_B;
  rpyRateToAngVelBody(rpy, rpy_rate, ang_vel_B);
  return ang_vel_B;
};

/**
 * @brief Calculate the angle velocity under world frame by using rpy rate
 * @param rpy
 * @param rpy_rate
 * @param ang_vel_W
 */
void rpyRateToAngVelWorld(const Eigen::Ref<const Eigen::Vector3d> &rpy,
                          const Eigen::Ref<const Eigen::Vector3d> &rpy_rate,
                          Eigen::Ref<Eigen::Vector3d> ang_vel_W);
/**
 * @brief Calculate the angle velocity under world frame by using rpy rate
 * @param rpy
 * @param rpy_rate
 * @return Eigen::Vector3d The angle velocity under world frame
 */
[[nodiscard]] inline Eigen::Vector3d rpyRateToAngVelWorld(
    const Eigen::Ref<const Eigen::Vector3d> &rpy,
    const Eigen::Ref<const Eigen::Vector3d> &rpy_rate) {
  Eigen::Vector3d ang_vel_W;
  rpyRateToAngVelWorld(rpy, rpy_rate, ang_vel_W);
  return ang_vel_W;
};

/**
 * @brief Calculate the rpy rate by using angle velocity under world frame
 * @param rpy
 * @param ang_vel_W
 * @param rpy_rate
 */
void angVelWorldToRpyRate(const Eigen::Ref<const Eigen::Vector3d> &rpy,
                          const Eigen::Ref<const Eigen::Vector3d> &ang_vel_W,
                          Eigen::Ref<Eigen::Vector3d> rpy_rate);
/**
 * @brief Calculate the rpy rate by using angle velocity under world frame
 * @param rpy
 * @param ang_vel_W
 * @return Eigen::Vector3d The rpy rate
 */
[[nodiscard]] inline Eigen::Vector3d angVelWorldToRpyRate(
    const Eigen::Ref<const Eigen::Vector3d> &rpy,
    const Eigen::Ref<const Eigen::Vector3d> &ang_vel_W) {
  Eigen::Vector3d rpy_rate;
  angVelWorldToRpyRate(rpy, ang_vel_W, rpy_rate);
  return rpy_rate;
};

/**
 * @brief Calculate the rpy rate by using angle velocity under body frame
 * @param rpy
 * @param ang_vel_B
 * @param rpy_rate
 */
void angVelBodyToRpyRate(const Eigen::Ref<const Eigen::Vector3d> &rpy,
                         const Eigen::Ref<const Eigen::Vector3d> &ang_vel_B,
                         Eigen::Ref<Eigen::Vector3d> rpy_rate);
/**
 * @brief Calculate the rpy rate by using angle velocity under body frame
 * @param rpy
 * @param ang_vel_B
 * @return Eigen::Vector3d The rpy rate
 */
[[nodiscard]] inline Eigen::Vector3d angVelBodyToRpyRate(
    const Eigen::Ref<const Eigen::Vector3d> &rpy,
    const Eigen::Ref<const Eigen::Vector3d> &ang_vel_B) {
  Eigen::Vector3d rpy_rate;
  angVelBodyToRpyRate(rpy, ang_vel_B, rpy_rate);
  return rpy_rate;
};

/**
 * @brief Use the given quaternion to rotate the vector
 * @param v
 * @param q
 * @param res
 */
void rotateVecByQuat(const Eigen::Ref<const Eigen::Vector3d> &v,
                     const Eigen::Ref<const Eigen::Vector4d> &q,
                     Eigen::Ref<Eigen::Vector3d> res);
/**
 * @brief Use the given quaternion to rotate the vector
 * @param v
 * @param q
 * @param res
 * @return The rotated vector
 */
[[nodiscard]] inline Eigen::Vector3d rotateVecByQuat(
    const Eigen::Ref<const Eigen::Vector3d> &v,
    const Eigen::Ref<const Eigen::Vector4d> &q) {
  Eigen::Vector3d res;
  rotateVecByQuat(v, q, res);
  return res;
};

/**
 * @brief Use the inverse of the given quaternion to rotate the vector
 * @param v
 * @param q
 * @param res
 */
void rotateVecByInvQuat(const Eigen::Ref<const Eigen::Vector3d> &v,
                        const Eigen::Ref<const Eigen::Vector4d> &q,
                        Eigen::Ref<Eigen::Vector3d> res);
/**
 * @brief Use the inverse of the given quaternion to rotate the vector
 * @param v
 * @param q
 * @return Eigen::Vector3d The rotated vector
 */
[[nodiscard]] inline Eigen::Vector3d rotateVecByInvQuat(
    const Eigen::Ref<const Eigen::Vector3d> &v,
    const Eigen::Ref<const Eigen::Vector4d> &q) {
  Eigen::Vector3d res;
  rotateVecByInvQuat(v, q, res);
  return res;
};

/**
 * @brief Transform the vector to its skew symmetric matrix
 * @param vec
 * @param skew_mat
 */
inline void vecToSkewMat(const Eigen::Ref<const Eigen::Vector3d> &vec,
                         Eigen::Ref<Eigen::Matrix3d> skew_mat) {
  skew_mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
};
/**
 * @brief Transform the vector to its skew symmetric matrix
 * @param vec
 * @param skew_mat
 */
[[nodiscard]] inline Eigen::Matrix3d vecToSkewMat(
    const Eigen::Ref<const Eigen::Vector3d> &vec) {
  Eigen::Matrix3d skew_mat;
  skew_mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return skew_mat;
};

/**
 * @brief
 *
 * @param skew_mat
 * @param vec
 */
inline void skewMatToVec(const Eigen::Ref<const Eigen::Matrix3d> skew_mat,
                         Eigen::Ref<Eigen::Vector3d> vec) {
  vec << skew_mat(2, 1), skew_mat(0, 2), skew_mat(1, 0);
};
/**
 * @brief
 *
 * @param skew_mat
 * @return Eigen::Vector3d
 */
[[nodiscard]] inline Eigen::Vector3d skewMatToVec(
    const Eigen::Ref<const Eigen::Matrix3d> skew_mat) {
  Eigen::Vector3d vec;
  vec << skew_mat(2, 1), skew_mat(0, 2), skew_mat(1, 0);
  return vec;
};

/**
 * @brief Mapping of so(3) space to rotation matrix SO(3)
 * @param skew_mat the skew symmetric matrix which is converted from
              the unit vector
 * @param theta the rotation angle which belongs to (-pi, pi]
 * @param rot rotation matrix
 */
void so3ToRot(const Eigen::Ref<const Eigen::Matrix3d> &skew_mat,
              const double &theta, Eigen::Ref<Eigen::Matrix3d> rot);
/**
 * @brief
 *
 * @param skew_mat
 * @param theta
 * @return Eigen::Matrix3d
 */
[[nodiscard]] inline Eigen::Matrix3d so3ToRot(
    const Eigen::Ref<const Eigen::Matrix3d> &skew_mat, const double &theta) {
  Eigen::Matrix3d rot;
  so3ToRot(skew_mat, theta, rot);
  return rot;
};

/**
 * @brief Mapping of r(3) space to rotation matrix SO(3)
 *
 * @param axis_vec the rotation axis, is presented as a unit vector
 * @param theta the rotation angle which belongs to (-pi, pi]
 * @param rot rotation matrix
 */
void angleAxisToRot(const Eigen::Ref<const Eigen::Vector3d> &axis_vec,
                    const double &theta, Eigen::Ref<Eigen::Matrix3d> rot);
/**
 * @brief
 *
 * @param axis_vec
 * @param theta
 * @return Eigen::Matrix3d
 */
[[nodiscard]] inline Eigen::Matrix3d angleAxisToRot(
    const Eigen::Ref<const Eigen::Vector3d> &axis_vec, const double &theta) {
  Eigen::Matrix3d rot;
  angleAxisToRot(axis_vec, theta, rot);
  return rot;
};

/**
 * @brief Mapping of rotation matrix SO(3) to r(3) space
 *
 * @param rot rotation matrix
 * @param axis_vec the rotation axis, is presented as a unit vector
 * @param theta the rotation angle which belongs to [0, pi]
 */
void rotToAngleAxis(const Eigen::Ref<const Eigen::Matrix3d> &rot,
                    Eigen::Ref<Eigen::Vector3d> axis_vec, double &theta);

/**
 * @brief
 *
 * @param rot
 * @param sigma
 * @param axis_vec
 * @param theta
 */
// void rotToExtendAngleAxis(const Eigen::Ref<const Eigen::Matrix3d> &rot,
//                           const Eigen::Ref<const Eigen::Vector3d> sigma,
//                           Eigen::Ref<Eigen::Vector3d> axis_vec, double
//                           &theta);

/**
 * @brief Mapping of R(3) space to rotation matrix SO(3)
 *
 * @param vec the L2 norm is the rotation angle and
              the direction is the rotation axis
 * @param rot rotation matrix
 */
inline void vecToRot(const Eigen::Ref<const Eigen::Vector3d> &vec,
                     Eigen::Ref<Eigen::Matrix3d> rot) {
  double theta(vec.norm());
  if (std::abs(theta) < 1e-4) {
    rot = Eigen::Matrix3d::Identity();
  } else {
    Eigen::Vector3d axis_vec = vec / vec.norm();
    angleAxisToRot(axis_vec, theta, rot);
  }
};
/**
 * @brief
 *
 * @param vec
 * @return Eigen::Matrix3d
 */
[[nodiscard]] inline Eigen::Matrix3d vecToRot(
    const Eigen::Ref<const Eigen::Vector3d> &vec) {
  Eigen::Matrix3d rot;
  double theta(vec.norm());
  if (std::abs(theta) < 1e-4) {
    rot = Eigen::Matrix3d::Identity();
  } else {
    Eigen::Vector3d axis_vec = vec / vec.norm();
    angleAxisToRot(axis_vec, theta, rot);
  }
  return rot;
};

/**
 * @brief Mapping of SO(3) space to R(3)
 *
 * @param rot rotation matrix
 * @param vec the L2 norm is the rotation angle and
              the direction is the rotation axis
 */
inline void rotToVec(const Eigen::Ref<const Eigen::Matrix3d> &rot,
                     Eigen::Ref<Eigen::Vector3d> vec) {
  double theta;
  Eigen::Vector3d axis_vec;
  rotToAngleAxis(rot, axis_vec, theta);
  vec = theta * axis_vec;
};
/**
 * @brief Mapping of SO(3) space to R(3)
 *
 * @param rot
 * @return Eigen::Vector3d
 */
[[nodiscard]] inline Eigen::Vector3d rotToVec(
    const Eigen::Ref<const Eigen::Matrix3d> &rot) {
  double theta;
  Eigen::Vector3d axis_vec;
  rotToAngleAxis(rot, axis_vec, theta);
  return theta * axis_vec;
};

/**
 * @brief Geodesic distance between two rotation matrices
 *
 * @param rot1
 * @param rot2
 */
[[nodiscard]] inline double rotGeodesicDistance(
    const Eigen::Ref<const Eigen::Matrix3d> &rot1,
    const Eigen::Ref<const Eigen::Matrix3d> &rot2) {
  double theta;
  Eigen::Vector3d axis_vec;
  rotToAngleAxis(rot1.transpose() * rot2, axis_vec, theta);
  return theta;
};

/**
 * @brief Chordal distance between two rotation matrices
 *
 * @param rot1
 * @param rot2
 */
[[nodiscard]] inline double rotChordalDistance(
    const Eigen::Ref<const Eigen::Matrix3d> &rot1,
    const Eigen::Ref<const Eigen::Matrix3d> &rot2) {
  return (rot1.transpose() * rot2 - Eigen::Matrix3d::Identity()).norm();
};

/**
 * @brief Transform rotation matrix and position vector
          to homogeneous transformation matrix.
 * @param rot rotation matrix
 * @param pos position vector
 * @param trans_mat homogeneous transformation matrix
 */
inline void rotAndPosToTransMat(const Eigen::Ref<const Eigen::Matrix3d> &rot,
                                const Eigen::Ref<const Eigen::Vector3d> &pos,
                                Eigen::Ref<Eigen::Matrix4d> trans_mat) {
  trans_mat.setIdentity();
  trans_mat.block<3, 3>(0, 0) = rot;
  trans_mat.block<3, 1>(0, 3) = pos;
};
/**
 * @brief
 *
 * @param rot
 * @param pos
 * @return Eigen::Matrix4d
 */
[[nodiscard]] inline Eigen::Matrix4d rotAndPosToTransMat(
    const Eigen::Ref<const Eigen::Matrix3d> &rot,
    const Eigen::Ref<const Eigen::Vector3d> &pos) {
  Eigen::Matrix4d trans_mat = Eigen::Matrix4d::Identity();
  trans_mat.block<3, 3>(0, 0) = rot;
  trans_mat.block<3, 1>(0, 3) = pos;
  return trans_mat;
};
/**
 * @brief Transform homogeneous transformation matrix to
          rotation matrix and position vector.
 * @param trans_mat
 * @param rot
 * @param pos
 */
inline void transMatToRotAndPos(
    const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
    Eigen::Ref<Eigen::Matrix3d> rot, Eigen::Ref<Eigen::Vector3d> pos) {
  rot = trans_mat.block<3, 3>(0, 0);
  pos = trans_mat.block<3, 1>(0, 3);
};

/**
 * @brief Get 4x4 matrix representation of twist
 * @param twist 6x1 vector
 * @param twist_homo_mat 4x4 matrix
 */
void twistToTwistHomo(const Eigen::Ref<const Eigen::VectorXd> &twist,
                      Eigen::Ref<Eigen::Matrix4d> twist_homo);
/**
 * @brief
 *
 * @param twist
 * @return Eigen::Matrix4d
 */
[[nodiscard]] Eigen::Matrix4d twistToTwistHomo(
    const Eigen::Ref<const Eigen::VectorXd> &twist);

/**
 * @brief Get twist from its homogeneous matrix representation
 * @param twist_homo 4x4 matrix
 * @param twist 6x1 vector
 */
void twistHomoToTwist(const Eigen::Ref<const Eigen::Matrix4d> &twist_homo,
                      Eigen::Ref<Eigen::VectorXd> twist);
/**
 * @brief
 *
 * @param twist_homo
 * @return Eigen::VectorXd
 */
[[nodiscard]] Eigen::VectorXd twistHomoToTwist(
    const Eigen::Ref<const Eigen::Matrix4d> &twist_homo);

/**
 * @brief Get homogeneous transformation's adjoint presentation
 *
 * @param trans_mat
 * @param adjoint_mat
 */
void adjointMat(const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
                Eigen::Ref<Eigen::MatrixXd> adjoint_mat);
/**
 * @brief
 *
 * @param trans_mat
 * @return Eigen::MatrixXd
 */
[[nodiscard]] Eigen::MatrixXd adjointMat(
    const Eigen::Ref<const Eigen::Matrix4d> &trans_mat);

/**
 * @brief Change representation coordinates of one twist
 * @param trans_mat homogeneous transformation matrix
 * @param twist
 * @param result_twist_homo
 */
void adjointMapToTwistHomo(const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
                           const Eigen::Ref<const Eigen::VectorXd> &twist,
                           Eigen::Ref<Eigen::Matrix4d> result_twist_homo);
/**
 * @brief Change representation coordinates of one twist
 * @param trans_mat
 * @param twist
 * @return Eigen::Matrix4d The Homogeneous matrix of the result twist
 */
[[nodiscard]] Eigen::Matrix4d adjointMapToTwistHomo(
    const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
    const Eigen::Ref<const Eigen::VectorXd> &twist);

/**
 * @brief Change representation coordinates of one twist
 * @param trans_mat homogeneous transformation matrix
 * @param twist_homo
 * @param result_twist_homo
 */
void adjointMapTwistHomoToTwistHomo(
    const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
    const Eigen::Ref<const Eigen::Matrix4d> &twist_homo,
    Eigen::Ref<Eigen::Matrix4d> result_twist_homo);
/**
 * @brief Change representation coordinates of one twist
 * @param trans_mat
 * @param twist
 * @return Eigen::Matrix4d The Homogeneous matrix of the result twist
 */
[[nodiscard]] Eigen::Matrix4d adjointMapTwistHomoToTwistHomo(
    const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
    const Eigen::Ref<const Eigen::Matrix4d> &twist);

/**
 * @brief Change representation coordinates of one twist
 *
 * @param trans_mat homogeneous transformation matrix
 * @param twist
 * @param result
 */
void adjointMap(const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
                const Eigen::Ref<const Eigen::VectorXd> &twist,
                Eigen::Ref<Eigen::VectorXd> result);
/**
 * @brief Change representation coordinates of one twist
 * @param trans_mat homogeneous transformation matrix
 * @param twist
 * @return Eigen::VectorXd The result twist
 */
[[nodiscard]] Eigen::VectorXd adjointMap(
    const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
    const Eigen::Ref<const Eigen::VectorXd> &twist);

/**
 * @brief Use screw to represent twist
 * @details This also can be used to convert one screw vector to its axis and
 * norm
 * @param twist
 * @param screw_axis 6x1 vector
 * @param d_theta derivation of theta
 */
void twistToScrew(const Eigen::Ref<const Eigen::VectorXd> &twist,
                  Eigen::Ref<Eigen::VectorXd> screw_axis, double &d_theta);

/**
 * @brief Convert se3 to SE3
 * @param screw_skew_mat screw axis (6x1 vector)
 * @param theta
 * @param trans_mat homogeneous transformation
 */
void se3ToTransMat(const Eigen::Ref<const Eigen::Matrix4d> &screw_skew_mat,
                   const double &theta, Eigen::Ref<Eigen::Matrix4d> trans_mat);
/**
 * @brief Convert screw to homogeneous transformation(R6 -> se3 -> SE3)
 * @param screw_skew_mat screw axis (6x1 vector)
 * @param theta
 * @param trans_mat homogeneous transformation
 * @return Eigen::Matrix4d The homogeneous transformation
 */
[[nodiscard]] Eigen::Matrix4d se3ToTransMat(
    const Eigen::Ref<const Eigen::Matrix4d> &screw_skew_mat,
    const double &theta);

/**
 * @brief Convert screw to homogeneous transformation(R6 -> se3 -> SE3)
 * @param screw_axis screw axis (6x1 vector)
 * @param theta
 * @param trans_mat homogeneous transformation
 */
void screwToTransMat(const Eigen::Ref<const Eigen::VectorXd> &screw_axis,
                     const double &theta,
                     Eigen::Ref<Eigen::Matrix4d> trans_mat);
/**
 * @brief
 *
 * @param screw_axis
 * @param theta
 * @return Eigen::Matrix4d
 */
[[nodiscard]] Eigen::Matrix4d screwToTransMat(
    const Eigen::Ref<const Eigen::VectorXd> &screw_axis, const double &theta);

/**
 * @brief  Convert screw vector to homogeneous transformation(R6 -> se3 -> SE3)
 *
 * @param screw_vec screw axis * theta
 * @param trans_mat homogeneous transformation
 */
void twistToTransMat(const Eigen::Ref<const Eigen::VectorXd> &twist,
                     Eigen::Ref<Eigen::Matrix4d> trans_mat);
/**
 * @brief
 *
 * @param twist
 * @return Eigen::Matrix4d
 */
[[nodiscard]] inline Eigen::Matrix4d twistToTransMat(
    const Eigen::Ref<const Eigen::VectorXd> &twist) {
  Eigen::Matrix4d trans_mat;
  twistToTransMat(twist, trans_mat);
  return trans_mat;
};

/**
 * @brief Convert homogeneous transformation to screw (SE3 -> se3 -> R6)
 *
 * @param trans_mat
 * @param screw_axis
 * @param theta
 */
void transMatToScrew(const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
                     Eigen::Ref<Eigen::VectorXd> screw_axis, double &theta);

/**
 * @brief Convert homogeneous transformation to screw vector (SE3 -> se3 -> R6)
 *
 * @param trans_mat
 * @param screw_vec
 */
inline void transMatToTwist(const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
                            Eigen::Ref<Eigen::VectorXd> twist) {
  Eigen::VectorXd screw_axis(6);
  double theta;
  transMatToScrew(trans_mat, screw_axis, theta);
  twist = screw_axis * theta;
};
/**
 * @brief
 *
 * @param trans_mat
 * @return Eigen::VectorXd
 */
[[nodiscard]] inline Eigen::VectorXd transMatToTwist(
    const Eigen::Ref<const Eigen::Matrix4d> &trans_mat) {
  Eigen::VectorXd screw_axis(6);
  double theta;
  transMatToScrew(trans_mat, screw_axis, theta);
  return screw_axis * theta;
};

/**
 * @brief
 *
 * @param rot_W_A
 * @param pos_W_A
 * @param pos_W_B
 * @param vel_W_A
 * @param vel_W_B
 * @param ang_vel_W_A
 * @param ang_vel_W_B
 * @param vel_A_B
 * @param ang_vel_A_B
 */
inline void calcRelativeVel(
    const Eigen::Ref<const Eigen::Matrix3d> &rot_W_A,
    const Eigen::Ref<const Eigen::Vector3d> &pos_W_A,
    const Eigen::Ref<const Eigen::Vector3d> &pos_W_B,
    const Eigen::Ref<const Eigen::Vector3d> &vel_W_A,
    const Eigen::Ref<const Eigen::Vector3d> &vel_W_B,
    const Eigen::Ref<const Eigen::Vector3d> &ang_vel_W_A,
    const Eigen::Ref<const Eigen::Vector3d> &ang_vel_W_B,
    Eigen::Ref<Eigen::Vector3d> vel_A_B,
    Eigen::Ref<Eigen::Vector3d> ang_vel_A_B) {
  Eigen::Vector3d pos_A_B = rot_W_A.transpose() * (pos_W_B - pos_W_A);
  vel_A_B = rot_W_A.transpose() *
            (vel_W_B - vel_W_A - vecToSkewMat(ang_vel_W_A) * rot_W_A * pos_A_B);
  ang_vel_A_B = rot_W_A.transpose() * (ang_vel_W_B - ang_vel_W_A);
};

/**
 * @brief Convert cartesian coordinates to spherical coordinates
 *
 * @param pos the cartesian coordinates
 * @param lpy spherical coordinates, l: length, p: pitch, y: yaw
 */
void posToLpy(const Eigen::Ref<const Eigen::Vector3d> &pos,
              Eigen::Ref<Eigen::Vector3d> lpy);
/**
 * @brief
 *
 * @param pos
 * @return Eigen::Vector3d
 */
[[nodiscard]] inline Eigen::Vector3d posToLpy(
    const Eigen::Ref<const Eigen::Vector3d> &pos) {
  Eigen::Vector3d lpy;
  posToLpy(pos, lpy);
  return lpy;
};

/**
 * @brief Convert spherical coordinates to cartesian coordinates
 *
 * @param lpy spherical coordinates, l: length, p: pitch(satisfy right hand
 *            coordinate)[0, pi]->[-pi/2, pi / 2], y: yaw[-pi, pi]
 * @param pos the cartesian coordinates
 */
void lpyToPos(const Eigen::Ref<const Eigen::Vector3d> &lpy,
              Eigen::Ref<Eigen::Vector3d> pos);
/**
 * @brief
 *
 * @param lpy
 * @return Eigen::Vector3d
 */
[[nodiscard]] inline Eigen::Vector3d lpyToPos(
    const Eigen::Ref<const Eigen::Vector3d> &lpy) {
  Eigen::Vector3d pos;
  lpyToPos(lpy, pos);
  return pos;
};

/**
 * @brief
 *
 * @param lpy
 * @param lpy_rate
 * @param lin_vel
 */
void lpyRateToLinVel(const Eigen::Ref<const Eigen::Vector3d> &lpy,
                     const Eigen::Ref<const Eigen::Vector3d> &lpy_rate,
                     Eigen::Ref<Eigen::Vector3d> lin_vel);
/**
 * @brief
 *
 * @param lpy
 * @param lpy_rate
 * @return Eigen::Vector3d
 */
[[nodiscard]] inline Eigen::Vector3d lpyRateToLinVel(
    const Eigen::Ref<const Eigen::Vector3d> &lpy,
    const Eigen::Ref<const Eigen::Vector3d> &lpy_rate) {
  Eigen::Vector3d lin_vel_W;
  lpyRateToLinVel(lpy, lpy_rate, lin_vel_W);
  return lin_vel_W;
};

/**
 * @brief
 *
 * @param lpy
 * @param lin_vel
 * @param lpy_rate
 */
void linVelToLpyRate(const Eigen::Ref<const Eigen::Vector3d> &lpy,
                     const Eigen::Ref<const Eigen::Vector3d> &lin_vel,
                     Eigen::Ref<Eigen::Vector3d> lpy_rate);
/**
 * @brief
 *
 * @param lpy
 * @param lin_vel
 * @return Eigen::Vector3d
 */
[[nodiscard]] inline Eigen::Vector3d linVelToLpyRate(
    const Eigen::Ref<const Eigen::Vector3d> &lpy,
    const Eigen::Ref<const Eigen::Vector3d> &lin_vel) {
  Eigen::Vector3d lpy_rate;
  linVelToLpyRate(lpy, lin_vel, lpy_rate);
  return lpy_rate;
};

inline void geometryMsgsPose2Pose(const geometry_msgs::Pose& pose_msg,
                                  Eigen::Ref<Eigen::Matrix4d> pose) {
  pose.setIdentity();
  pose(0, 3) = pose_msg.position.x;
  pose(1, 3) = pose_msg.position.y;
  pose(2, 3) = pose_msg.position.z;
  Eigen::Vector4d quat;
  quat << pose_msg.orientation.w, pose_msg.orientation.x,
      pose_msg.orientation.y, pose_msg.orientation.z;
  quatToRot(quat, pose.block<3, 3>(0, 0));
}

inline void geometryMsgsPoseStamped2Pose(
    const geometry_msgs::PoseStamped& pose_msg,
    Eigen::Ref<Eigen::Matrix4d> pose) {
  pose.setIdentity();
  pose(0, 3) = pose_msg.pose.position.x;
  pose(1, 3) = pose_msg.pose.position.y;
  pose(2, 3) = pose_msg.pose.position.z;
  Eigen::Vector4d quat;
  quat << pose_msg.pose.orientation.w, pose_msg.pose.orientation.x,
      pose_msg.pose.orientation.y, pose_msg.pose.orientation.z;
  quatToRot(quat, pose.block<3, 3>(0, 0));
}


}  // namespace arm_controller
#endif  // GEOMETRY_GEOMETRY_UTILS_H_
