# Z1 Arm (unitree_ros): 项目 README

## 1. 项目里程碑 (Milestones)

* **[10.26]** 完善 `cross_task` 逻辑并增加 `horizon` (水平线) 辅助功能。(6965d18)
* **[10.22]** 修复 `arm_sdk` 依赖问题。 (e2ac125)
* **[10.21]** "race" (竞赛) 任务开发。 (889e784)
* **[10.17]** **实机测试 (On-robot test) 成功**。 (849baaa)
* **[10.15]** 增加检测 (Detection) 功能。 (87211d3)

## 2. 主要功能 (Features & Modifications)

### 2.1. 仿真与模型 (Simulation & URDF)

* **模块化 (Modular):** `camera.xacro` (in `z1_description/xacro/`).
* **集成 (Integration):** 相机 fixed to `gripperMover` link.
* **姿态修正 (Pose):** Corrected default vertical pose (RPY offset).
* **Gazebo 插件:** `libgazebo_ros_camera.so` (800x800, 30Hz, color image).
* **仿真环境 (World):** Added "pipe" model for grasping.

### 2.2. 核心逻辑 (Core Logic)

* **目标检测 (Detection):** Implemented detection function. (87211d3)
* **坐标转化 (TF):** Logic for coordinate transformation and sending goals. (1240efb)
* **重力补偿 (Gravity):** Fixed gravity compensation. (e96e813)
* **夹爪控制 (Gripper):** Added gripper control interface. (2afaaa8)

### 2.3. ROS API (服务与接口)

* **`getgoalandangle.srv`:** "5个点" 的服务流程，用于获取目标和角度。(0a14608)
* **`cameratolink00` srv:** 获取相机到 `link00` 的转换。 (9f826ec)
* **姿态接口 (Pose API):** 增加了模型 Pitch/Roll/Yaw 的接口。 (a9226a5)

## 3. 关键配置 (Key Config)

* **机械臂基座高度 (Base Height):** **0.025m**.
* **抓取参考点 (Grasping Reference):** Arm @ **0.1m** aligns with Pipe @ **0.025m**.

## 4. 后续步骤 (Next Steps)

* **IK (Inverse Kinematics):** Need to configure solver.
* **任务优化 (Task):** 持续优化 `cross_task` 逻辑。
* **可视化 (Debug):** 完善 `debug_line` 和 `horizon` 可视化调试功能。

## 5. 开发者提示 (Developer Notes)

* **如需深度 (Need Depth):** Use `libgazebo_ros_openni_kinect.so` plugin.
    * (Provides: color, depth, point cloud).