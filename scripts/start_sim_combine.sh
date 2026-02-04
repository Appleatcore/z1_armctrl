#!/bin/bash

# 启动脚本 - 用于启动机械臂控制系统的所有必要程序 (Combine 版本)
# 使用方法: ./start_sim.sh

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 工作空间路径
WORKSPACE_DIR="/home/applepie/unitree_ws"

# 存储所有启动的进程PID
PIDS=()

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}启动机械臂控制系统 (xterm 版)${NC}"
echo -e "${GREEN}========================================${NC}"

# 检查工作空间是否存在
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${YELLOW}错误: 工作空间目录不存在: $WORKSPACE_DIR${NC}"
    exit 1
fi

# 检查 xterm 是否安装
if ! command -v xterm &> /dev/null; then
    echo -e "${RED}错误: 未检测到 xterm。请执行: sudo apt install xterm${NC}"
    exit 1
fi

# 进入工作空间
cd "$WORKSPACE_DIR"

# 1. 启动Gazebo仿真环境 (左上角)
echo -e "${YELLOW}[1/4] 启动Gazebo仿真环境...${NC}"
xterm -T "Gazebo Simulation" -geometry 80x24+0+0 -e "bash -c \"
    cd $WORKSPACE_DIR;
    if [ -f devel/setup.bash ]; then source devel/setup.bash; fi
    echo '启动Gazebo仿真环境...';
    roslaunch unitree_gazebo z1_combine.launch;
    exec bash
\"" &
PIDS+=($!)

# 等待Gazebo启动
echo -e "${YELLOW}等待Gazebo启动 (10秒)...${NC}"
sleep 10

# 2. 启动控制器 (右上角)
echo -e "${YELLOW}[2/4] 启动机械臂控制器...${NC}"
xterm -T "Arm Controller" -geometry 80x24+500+0 -e "bash -c \"
    cd $WORKSPACE_DIR/z1_controller/build;
    if [ -f $WORKSPACE_DIR/devel/setup.bash ]; then source $WORKSPACE_DIR/devel/setup.bash; fi
    echo '启动机械臂控制器...';
    ./sim_ctrl;
    exec bash
\"" &
PIDS+=($!)

# 等待控制器启动
echo -e "${YELLOW}等待控制器启动 (3秒)...${NC}"
sleep 3s

# 3. 启动 arm_controller_node (左下角)
echo -e "${YELLOW}[3/4] 启动 arm_controller_node...${NC}"
xterm -T "Arm Controller Node" -geometry 80x24+0+450 -e "bash -c \"
    cd $WORKSPACE_DIR;
    if [ -f devel/setup.bash ]; then source devel/setup.bash; fi
    echo '启动 arm_controller_node...';
    roslaunch arm_controller arm_controller_node.launch;
    exec bash
\"" &
PIDS+=($!)

# 等待准备启动RViz
echo -e "${YELLOW}[4/4] 准备启动RViz...${NC}"
sleep 3s

# 4. 启动RViz (右下角)
echo -e "${YELLOW}启动RViz可视化...${NC}"
# 注意：这里统一使用 bash 替代 zsh 以保证 xterm 调用的兼容性
xterm -T "RViz" -geometry 80x24+500+450 -e "bash -c \"
    cd $WORKSPACE_DIR;
    if [ -f devel/setup.bash ]; then source devel/setup.bash; fi
    echo '启动RViz...';
    roslaunch z1_description z1_combine_rviz.launch;
    exec bash
\"" &
PIDS+=($!)

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}所有程序已通过 xterm 启动并平铺布局！${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${RED}按 Ctrl+C 一键退出所有程序${NC}"
echo ""

# 捕获Ctrl+C信号，退出所有程序
cleanup() {
    echo -e "\n${YELLOW}正在关闭所有程序...${NC}"
    
    # 关闭所有相关进程
    killall -9 gzserver gzclient 2>/dev/null
    killall -9 sim_ctrl 2>/dev/null
    killall -9 rviz 2>/dev/null
    
    # 关闭ROS节点和相关launch
    pkill -9 -f "arm_controller_node" 2>/dev/null
    pkill -9 -f "z1_combine.launch" 2>/dev/null
    pkill -9 -f "z1_combine_rviz.launch" 2>/dev/null
    
    # 关闭 xterm 终端窗口
    for pid in "${PIDS[@]}"; do
        kill -15 "$pid" 2>/dev/null
    done
    
    echo -e "${GREEN}所有程序已关闭 (保留roscore)${NC}"
    exit 0
}

trap cleanup INT

# 保持脚本运行，等待Ctrl+C
while true; do
    sleep 1
done