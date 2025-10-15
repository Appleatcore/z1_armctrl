#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <arm_controller_srvs/CheckPoseInWorkspace.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

// 直线结构体
struct Line3D {
  Eigen::Vector3d point;      // 直线上的一点
  Eigen::Vector3d direction;  // 方向向量
  
  // 在直线上采样点
  std::vector<Eigen::Vector3d> samplePoints(double t_start, double t_end, int num_samples) const {
    std::vector<Eigen::Vector3d> samples;
    if (num_samples <= 0) return samples;
    
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
};

class LineCalculator {
public:
  LineCalculator(ros::NodeHandle& nh) : nh_(nh), enable_ik_check_(false) {
    // 发布采样点
    pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/calculated_poses", 10);
    
    // 创建服务客户端(用于IK检测)
    check_pose_client_ = nh_.serviceClient<arm_controller_srvs::CheckPoseInWorkspace>(
        "/arm_controller_node/check_pose_in_workspace");
    
    // 等待服务可用(非阻塞)
    if (check_pose_client_.waitForExistence(ros::Duration(2.0))) {
      enable_ik_check_ = true;
      ROS_INFO("IK service can be used");
    } else {
      ROS_WARN("IK service can not be used,need to roslaunch arm_controller_node first");
    }
  }
  
  // 旋转直线
  Line3D rotateLine(const Line3D& line,
                    const Eigen::Vector3d& rotation_center,
                    const Eigen::Vector3d& rotation_axis,
                    double angle_rad) const {
    Line3D rotated_line;
    
    Eigen::AngleAxisd rotation(angle_rad, rotation_axis.normalized());
    Eigen::Matrix3d R = rotation.toRotationMatrix();
    
    Eigen::Vector3d relative_point = line.point - rotation_center;
    rotated_line.point = R * relative_point + rotation_center;
    rotated_line.direction = (R * line.direction).normalized();
    
    return rotated_line;
  }
  
  // 检测位姿是否可达(通过IK)
  bool checkPoseReachable(const Eigen::Vector3d& position) {
    if (!enable_ik_check_) {
      return true;  // 如果服务不可用,默认返回true
    }
    
    arm_controller_srvs::CheckPoseInWorkspace srv;
    srv.request.target_pose.position.x = position.x();
    srv.request.target_pose.position.y = position.y();
    srv.request.target_pose.position.z = position.z();
    srv.request.target_pose.orientation.w = 1.0;
    srv.request.target_pose.orientation.x = 0.0;
    srv.request.target_pose.orientation.y = 0.0;
    srv.request.target_pose.orientation.z = 0.0;
    
    if (check_pose_client_.call(srv)) {
      return srv.response.is_in_workspace;
    }
    return false;
  }
  
  // 平移直线
  Line3D translateLine(const Line3D& line,
                       const Eigen::Vector3d& direction,
                       double distance) const {
    Line3D translated_line;
    Eigen::Vector3d translation = direction.normalized() * distance;
    translated_line.point = line.point + translation;
    translated_line.direction = line.direction;
    return translated_line;
  }
  
  // 生成旋转直线并采样
  std::vector<std::pair<Line3D, std::vector<Eigen::Vector3d>>>
  generateRotatedLinesWithSamples(const Line3D& line,
                                   const Eigen::Vector3d& rotation_center,
                                   const Eigen::Vector3d& rotation_axis,
                                   int num_rotations,
                                   double angle_step,
                                   double t_start,
                                   double t_end,
                                   int num_samples) {
    std::vector<std::pair<Line3D, std::vector<Eigen::Vector3d>>> results;
    
    ROS_INFO("[Line Rotation] Generating %d rotated lines around axis: (%.3f, %.3f, %.3f) with angle step: %.1f deg",
             num_rotations, rotation_axis.x(), rotation_axis.y(), rotation_axis.z(),
             angle_step * 180.0 / M_PI);
    
    for (int i = 1; i <= num_rotations; i++) {
      double angle = i * angle_step;
      Line3D rotated_line = rotateLine(line, rotation_center, rotation_axis, angle);
      std::vector<Eigen::Vector3d> sampled_points = rotated_line.samplePoints(t_start, t_end, num_samples);
      
      results.push_back(std::make_pair(rotated_line, sampled_points));
      
      ROS_INFO("  [%d] Angle: %.1f deg, Line point: (%.3f, %.3f, %.3f), Direction: (%.3f, %.3f, %.3f), Samples: %d",
               i, angle * 180.0 / M_PI,
               rotated_line.point.x(), rotated_line.point.y(), rotated_line.point.z(),
               rotated_line.direction.x(), rotated_line.direction.y(), rotated_line.direction.z(),
               (int)sampled_points.size());
      
      // 打印采样点并检测IK
      int reachable_count = 0;
      for (size_t j = 0; j < sampled_points.size(); ++j) {
        bool reachable = checkPoseReachable(sampled_points[j]);
        if (reachable) reachable_count++;
        
        if (enable_ik_check_) {
          ROS_INFO("    Point[%zu]: (%.3f, %.3f, %.3f) %s",
                   j, sampled_points[j].x(), sampled_points[j].y(), sampled_points[j].z(),
                   reachable ? "[OK]" : "[UNREACHABLE]");
        } else {
          ROS_INFO("    Point[%zu]: (%.3f, %.3f, %.3f)",
                   j, sampled_points[j].x(), sampled_points[j].y(), sampled_points[j].z());
        }
      }
      
      if (enable_ik_check_) {
        ROS_INFO("    => Reachable: %d/%d points", reachable_count, (int)sampled_points.size());
      }
      
      // 测试相机姿态计算
      double test_pitch, test_roll, test_yaw;
      if(calculateCameraOrientation(rotated_line.direction, test_pitch, test_roll, test_yaw, 0.055)) {
        std::cout << "rotated_line,angle_step:" << (angle_step * 180.0 / M_PI) << ": SUCCESS\n" << std::endl;
      }
    }
    
    return results;
  }
  
  // 发布采样点为PoseArray
  void publishPoses(const std::vector<Eigen::Vector3d>& points, const std::string& frame_id = "link00") {
    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = frame_id;
    
    for (const auto& point : points) {
      geometry_msgs::Pose pose;
      pose.position.x = point.x();
      pose.position.y = point.y();
      pose.position.z = point.z();
      pose.orientation.w = 1.0;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose_array.poses.push_back(pose);
    }
    
    pose_array_pub_.publish(pose_array);
    ROS_INFO("Published %zu poses to /calculated_poses", points.size());
  }
  
  bool calculateCameraOrientation(const Eigen::Vector3d& camera_direction,
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
  // 参数监听器回调
  void paramCheckCallback(const ros::TimerEvent&) {
    // 检查参数是否变化
    double new_x, new_y, new_z;
    nh_.param("test/line_point_x", new_x, last_line_x_);
    nh_.param("test/line_point_y", new_y, last_line_y_);
    nh_.param("test/line_point_z", new_z, last_line_z_);
    
    if (new_x != last_line_x_ || new_y != last_line_y_ || new_z != last_line_z_) {
      ROS_INFO("\n========================================");
      ROS_INFO("Parameters changed! Recalculating...");
      ROS_INFO("New position: (%.3f, %.3f, %.3f)", new_x, new_y, new_z);
      ROS_INFO("========================================\n");
      
      last_line_x_ = new_x;
      last_line_y_ = new_y;
      last_line_z_ = new_z;
      
      // 重新计算
      run();
    }
  }
  
  void run() {
    // 从参数服务器读取配置
    double line_x, line_y, line_z;
    double line_pitch, line_yaw, line_roll;
    double sample_start, sample_end;
    int num_samples, num_rotations;
    double angle_step_deg;
    
    nh_.param("test/line_point_x", line_x, 0.8);
    nh_.param("test/line_point_y", line_y, 0.0);
    nh_.param("test/line_point_z", line_z, 0.15);
    nh_.param("test/line_point_pitch", line_pitch, -0.7854);
    nh_.param("test/line_point_yaw", line_yaw, 0.0);
    nh_.param("test/line_point_roll", line_roll, 0.0);
    nh_.param("test/sample_start", sample_start, -1.0);
    nh_.param("test/sample_end", sample_end, 0.0);
    nh_.param("test/num_samples", num_samples, 10);
    nh_.param("test/num_rotations", num_rotations, 1);
    nh_.param("test/angle_step_deg", angle_step_deg, 45.0);
    
    // 计算方向向量
    double origin_direction_x = cos(line_yaw) * cos(line_pitch);
    double origin_direction_y = sin(line_yaw) * cos(line_pitch);
    double origin_direction_z = sin(line_pitch);
    
    ROS_INFO("=== Line Calculator Started ===");
    ROS_INFO("Origin direction: (%.3f, %.3f, %.3f)", origin_direction_x, origin_direction_y, origin_direction_z);
    
    // 计算旋转轴
    double rotation_axis_x = -origin_direction_z;
    double rotation_axis_y = origin_direction_y;
    double rotation_axis_z = origin_direction_x;
    ROS_INFO("Rotation axis: (%.3f, %.3f, %.3f)", rotation_axis_x, rotation_axis_y, rotation_axis_z);
    
    // 创建原始直线
    Line3D original_line;
    original_line.point = Eigen::Vector3d(line_x, line_y, line_z);
    original_line.direction = Eigen::Vector3d(origin_direction_x, origin_direction_y, origin_direction_z).normalized();
    
    Eigen::Vector3d rotation_center(line_x, line_y, line_z);
    Eigen::Vector3d rotation_axis(rotation_axis_x, rotation_axis_y, rotation_axis_z);
    
    // 原直线
    ROS_INFO("\n============ MID LINE ============");
    auto results = generateRotatedLinesWithSamples(
        original_line, rotation_center, rotation_axis,
        num_rotations, 0.0, sample_start, sample_end, num_samples);
    
    // 正转
    ROS_INFO("\n============ HALF LINE ============");
    double angle_step_rad = angle_step_deg * M_PI / 180.0;
    auto results_1 = generateRotatedLinesWithSamples(
        original_line, rotation_center, rotation_axis,
        num_rotations, angle_step_rad, sample_start, sample_end, num_samples);
    
    // 反转
    ROS_INFO("\n============ HALF2 LINE ============");
    double angle_step_rad_2 = (360.0 - angle_step_deg) * M_PI / 180.0;
    auto results_2 = generateRotatedLinesWithSamples(
        original_line, rotation_center, rotation_axis,
        num_rotations, angle_step_rad_2, sample_start, sample_end, num_samples);
    
    // 平移
    ROS_INFO("\n============ OUT LINE ============");
    Line3D translated_line_1 = translateLine(original_line, Eigen::Vector3d(0.0, 1.0, 0.0), 0.3);
    auto samples_1 = translated_line_1.samplePoints(sample_start, sample_end, num_samples);
    ROS_INFO("Translated line (+0.3m in Y): Point (%.3f, %.3f, %.3f), Samples: %d",
             translated_line_1.point.x(), translated_line_1.point.y(), translated_line_1.point.z(),
             (int)samples_1.size());
    for (size_t j = 0; j < samples_1.size(); ++j) {
      ROS_INFO("  Point[%zu]: (%.3f, %.3f, %.3f)", j, samples_1[j].x(), samples_1[j].y(), samples_1[j].z());
    }
    // 测试相机姿态计算
    double test_pitch_1, test_roll_1, test_yaw_1;
    if(calculateCameraOrientation(translated_line_1.direction, test_pitch_1, test_roll_1, test_yaw_1, 0.055)) {
      std::cout << "translateline,distance:0.3: SUCCESS\n" << std::endl;
    }
    
    ROS_INFO("\n============ OUT2 LINE ============");
    Line3D translated_line_2 = translateLine(original_line, Eigen::Vector3d(0.0, 1.0, 0.0), -0.3);
    auto samples_2 = translated_line_2.samplePoints(sample_start, sample_end, num_samples);
    ROS_INFO("Translated line (-0.3m in Y): Point (%.3f, %.3f, %.3f), Samples: %d",
             translated_line_2.point.x(), translated_line_2.point.y(), translated_line_2.point.z(),
             (int)samples_2.size());
    for (size_t j = 0; j < samples_2.size(); ++j) {
      ROS_INFO("  Point[%zu]: (%.3f, %.3f, %.3f)", j, samples_2[j].x(), samples_2[j].y(), samples_2[j].z());
    }
    // 测试相机姿态计算
    double test_pitch_2, test_roll_2, test_yaw_2;
    if(calculateCameraOrientation(translated_line_2.direction, test_pitch_2, test_roll_2, test_yaw_2, 0.055)) {
      std::cout << "translateline,distance:-0.3: SUCCESS\n" << std::endl;
    }
    
    ROS_INFO("\n=== Calculation Complete ===");
    
    // 发布第一组采样点
    if (!results.empty()) {
      publishPoses(results[0].second);
    }
  }

  // 启动参数监听
  void startParamMonitoring(double check_rate = 1.0) {
    // 初始化参数缓存
    nh_.param("test/line_point_x", last_line_x_, 0.8);
    nh_.param("test/line_point_y", last_line_y_, 0.0);
    nh_.param("test/line_point_z", last_line_z_, 0.15);
    
    // 创建定时器(每秒检查一次)
    param_check_timer_ = nh_.createTimer(
        ros::Duration(1.0 / check_rate),
        &LineCalculator::paramCheckCallback,
        this);
    
    ROS_INFO("Parameter monitoring started (check rate: %.1f Hz)", check_rate);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pose_array_pub_;
  ros::ServiceClient check_pose_client_;
  ros::Timer param_check_timer_;
  bool enable_ik_check_;
  
  // 参数缓存
  double last_line_x_;
  double last_line_y_;
  double last_line_z_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "calculate_node");
  ros::NodeHandle nh("~");
  
  LineCalculator calculator(nh);
  
  // 首次计算
  calculator.run();
  
  // 启动参数监听(1Hz)
  calculator.startParamMonitoring(1.0);
  
  ROS_INFO("calculate_node ready. Listening for parameter changes...");
  
  ros::spin();
  return 0;
}
