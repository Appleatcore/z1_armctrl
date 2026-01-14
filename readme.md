# Unitree Z1 Arm Control System (ROS)

![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue)
![Platform](https://img.shields.io/badge/Platform-Ubuntu%2020.04-orange)

基于 ROS 的 Unitree Z1 机械臂控制系统。使用本项目前请先阅读宇树的z1的sdk开发文档：https://support.unitree.com/home/zh/Z1_developer/z1

本项目集成了 Gazebo 仿真、Pinocchio 运动学解算、视觉图像回调以及自动抓取任务规划。

将本项目放置在宇树的unitree_legged_msgs和unitree_ros-master  (https://github.com/Applepie0323/z1_env)  统一目录下编译

---

## 📋 目录（Table of Contents）

1.  [📖 项目简介 (Introduction)](#简介)
2.  [📂 项目结构与工作区 (Project Structure)](#项目结构与工作区设置-project-structure--workspace-setup)
3.  [📦 环境与依赖 (Prerequisites)](#环境依赖-prerequisites)
4.  [🛠️ 安装与编译 (Installation)](#安装与编译-installation)
5.  [🚀 快速开始 (Quick Start)](#快速开始-quick-start)
    * [启动仿真](#1-启动仿真环境)
    * [启动任务](#2-启动任务控制)
9.  [📝 待办事项 (TODO)](#todo-list)

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

### 1. 克隆工作空间

```
cd ~/unitree_ws_demo/src
git clone https://github.com/Applepie0323/z1_armctrl.git
```

### 2. 安装核心依赖 (关键步骤)

由于 ROS Noetic 官方源缺失部分库或版本不兼容，**请严格按照以下步骤安装，否则会导致编译失败**：

#### A. 安装 Pinocchio (运动学库)

**注意**：官方 apt 源中没有 `ros-noetic-pinocchio`。必须使用 Robotpkg 源安装：

```
# 1. 添加 Robotpkg 源和密钥
sudo apt update && sudo apt install -y curl lsb-release gnupg2
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
sudo apt update

# 2. 安装 Pinocchio (适配 Python 3.8)
sudo apt install -y robotpkg-py38-pinocchio

# 3. 配置环境变量 (建议写入 ~/.zshrc 或 ~/.bashrc)
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

#### B. 安装其他系统依赖

解决 `moveit_visual_tools` 缺失和 `pybind11.h` 找不到的问题：

```
# 安装 MoveIt 可视化工具 (注意不是 rviz_visual_tools)
sudo apt install ros-noetic-moveit-visual-tools

# 安装 Pybind11 开发库
sudo apt install ros-noetic-pybind11-catkin pybind11-dev
```

### 3. 编译工作空间

```
cd ~/unitree_ws_demo
catkin_make
source devel/setup.bash
```

## ❓ 常见编译问题 (Troubleshooting)

**Q1: 编译时报错 `Could not find a package configuration file provided by "realsense2_camera"`**

- **原因**: 缺少 RealSense 驱动依赖，但如果不使用实体相机，可以跳过编译。

- **解决**: 在相关包目录下创建 `CATKIN_IGNORE` 文件以忽略编译。

  ```
  # 进入报错的包目录 (例如 realsense-ros)
  cd src/realsense-ros
  touch CATKIN_IGNORE
  ```

**Q2: 报错 `fatal error: pybind11/pybind11.h: No such file`**

- **解决**: 请确保执行了上述安装步骤中的 `sudo apt install pybind11-dev`，并且建议删除 `build/` 和 `devel/` 文件夹后重新编译。

**Q3: 报错 `Make Error at ... find_package(moveit_visual_tools)`**

- **解决**: 这是一个很容易混淆的包。请确认你安装的是 **`ros-noetic-moveit-visual-tools`**，而不是 `ros-noetic-rviz-visual-tools`。

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