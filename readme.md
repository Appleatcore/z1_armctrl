# Unitree Z1 Arm Control System (ROS)

![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue)
![Platform](https://img.shields.io/badge/Platform-Ubuntu%2020.04-orange)

基于 ROS 的 Unitree Z1 机械臂控制系统。使用本项目前请先阅读宇树的z1的sdk开发文档：https://support.unitree.com/home/zh/Z1_developer/z1

本项目集成了 Gazebo 仿真、Pinocchio 运动学解算、视觉目标检测（Visual Detection）以及自动抓取任务规划。

将本项目放置在宇树的unitree_legged_msgs和unitree_ros-master  (https://github.com/Applepie0323/z1_env)  统一目录下编译

---

## 📋 目录（Table of Contents）

- [项目简介](#简介)  
- [系统要求与环境依赖](#环境依赖)  
- [安装与编译指南](#安装与编译)  
- [快速开始：运行仿真与任务](#快速开始)  
- [核心功能介绍](#核心功能)  
- [ROS 接口说明](#ros-接口说明)  
- [开发日志与更新记录](#开发日志)  
- [后续计划与待办事项](#todo)

## 📂 项目结构与工作区设置 (Project Structure & Workspace Setup)

为了确保项目能够顺利编译和运行，请参照以下目录结构组织你的 ROS 工作空间。本项目核心代码位于 `z1_ctrl_sys` 包中。

```
你的工作空间（例如 `unitree_ws_demo`）的目录结构应如下所示：
unitree_ws_demo/              # 工作空间根目录 (Workspace Root)
├── build/
├── devel/
├── src/                      # 源码目录
│   ├── z1_ctrl_sys/          # [核心项目] 本仓库代码
│   │   ├── arm_controller/
│   │   ├── arm_controller_srvs/
│   │   ├── scripts/
│   │   └── readme.md
│   │
│   ├── unitree_ros-master/   # [依赖] 仿真环境：https://github.com/Applepie0323/z1_env
│   ├── unitree_legged_msgs/  # [依赖] 通讯消息定义
│   └── realsense-ros/        # [可选] RealSense 相机驱动 (如不使用实机相机可忽略)
│
├── z1_controller/            # Unitree SDK 也就是底层的控制器
└── z1_sdk/                   # Unitree Z1 SDK
```


## 📦 环境依赖 (Prerequisites)

* **OS:** Ubuntu 20.04 LTS
* **ROS:** Noetic Ninjemys
* **Hardware:** Unitree Z1 Arm (Optional for simulation)
* **Dependencies:**
    * `unitree_legged_msgs`
    * `pinocchio` (运动学库)
    * `realsense2_camera` (深度相机，该项目演示demo暂时不需要)
    * `gazebo_ros_pkgs`

## 🛠️ 安装与编译 (Installation)

1.  **克隆工作空间**
    
    ```bash
    cd ~/unitree_ws_demo/src
    git clone https://github.com/Applepie0323/z1_armctrl.git
    ```
    
2.  **安装依赖**
    
    * 见宇树文档https://support.unitree.com/home/zh/Z1_developer/z1
    
3.  **编译**
    ```bash
    cd ~/unitree_ws_demo
    catkin_make
    source devel/setup.bash
    ```

## 🚀 快速开始 (Quick Start)

### 1. 启动仿真环境
进入脚本目录

```
cd z1_ctrl_sys/scripts
```

使用一键启动脚本加载 Gazebo 环境、控制器和 RViz：

```bash
# 场景一：梯形管道任务
./start_sim_combine.sh

# 场景二：花形管道任务
./start_sim_flower.sh
```

如果脚本无法运行，你可以手动打开 **4个终端** 依次执行以下命令：

- **终端 1 (环境):**

  Bash

  ```
  cd ~/unitree_ws_demo
  source devel/setup.bash
  roslaunch unitree_gazebo z1_combine.launch
  ```

- **终端 2 (控制器):**

  Bash

  ```
  cd ~/unitree_ws_demo/z1_controller/build
  ./sim_ctrl
  ```

- **终端 3 (ROS节点):**

  Bash

  ```
  cd ~/unitree_ws_demo
  source devel/setup.bash
  roslaunch arm_controller arm_controller_node.launch
  ```

- **终端 4 (RViz可视化):**

  Bash

  ```
  cd ~/unitree_ws_demo
  source devel/setup.bash
  roslaunch z1_description z1_combine_rviz.launch
  ```

### 2. 启动任务控制

另启一个终端，进入脚本目录并赋予执行权限：

```
cd src/z1_ctrl_sys/scripts
chmod +x get_goal_control_combine.py
chmod +x get_goal_control_flower.py
```

启动执行任务脚本，请根据终端提示进行交互：

```
# 执行梯形管道抓取任务
python3 get_goal_control_combine.py
# 执行花形管道抓取任务
python3 get_goal_control_flower.py
```

## 📝 TODO List

* 将核心控制逻辑从线性流程重构为 FSM (有限状态机) 形式

* 优化 Pinocchio IK 解算的收敛速度