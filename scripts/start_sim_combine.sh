#!/bin/bash

# 启动脚本 - 用于启动机械臂控制系统的所有必要程序
# 使用方法: ./start_sim.sh

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 工作空间路径
WORKSPACE_DIR="/home/applepie/unitree_ws_demo"

# 存储所有启动的进程PID
PIDS=()

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}启动机械臂控制系统${NC}"
echo -e "${GREEN}========================================${NC}"

# 检查工作空间是否存在
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${YELLOW}错误: 工作空间目录不存在: $WORKSPACE_DIR${NC}"
    exit 1
fi

# 进入工作空间
cd "$WORKSPACE_DIR"

# Source ROS环境
# echo -e "${YELLOW}[1/5] 加载ROS环境...${NC}"
# if [ -f devel/setup.bash ]; then
#     source devel/setup.bash
#     echo -e "${GREEN}已加载 devel/setup.bash${NC}"
# elif [ -f devel/setup.zsh ]; then
#     source devel/setup.zsh
#     echo -e "${GREEN}已加载 devel/setup.zsh${NC}"
# else
#     echo -e "${RED}错误: 找不到 setup 文件${NC}"
#     exit 1
# fi

# 启动Gazebo仿真环境
echo -e "${YELLOW}[2/5] 启动Gazebo仿真环境...${NC}"
gnome-terminal --tab --title="Gazebo Simulation" -- bash -c "
    cd $WORKSPACE_DIR;
    if [ -f devel/setup.bash ]; then
        source devel/setup.bash;
    elif [ -f devel/setup.zsh ]; then
        source devel/setup.zsh;
    fi
    echo '启动Gazebo仿真环境...';
    roslaunch unitree_gazebo z1_combine.launch;
    # roslaunch unitree_gazebo z1_flower.launch;
    exec bash
" &
PIDS+=($!)

# 等待Gazebo启动
echo -e "${YELLOW}等待Gazebo启动 (10秒)...${NC}"
sleep 10

# 启动控制器
echo -e "${YELLOW}[3/5] 启动机械臂控制器...${NC}"
gnome-terminal --tab --title="Arm Controller" -- bash -c "
    cd $WORKSPACE_DIR/z1_controller/build;
    if [ -f $WORKSPACE_DIR/devel/setup.bash ]; then
        source $WORKSPACE_DIR/devel/setup.bash;
    fi
    echo '启动机械臂控制器...';
    ./sim_ctrl;
    exec bash
" &
PIDS+=($!)

# 等待控制器启动
echo -e "${YELLOW}等待控制器启动 (5秒)...${NC}"
sleep 3s

# 启动 arm_controller_node
echo -e "${YELLOW}[4/5] 启动 arm_controller_node...${NC}"
gnome-terminal --tab --title="Arm Controller Node" -- bash -c "
    cd $WORKSPACE_DIR;
    if [ -f devel/setup.bash ]; then
        source devel/setup.bash;
    elif [ -f devel/setup.zsh ]; then
        source devel/setup.zsh;
    fi
    echo '启动 arm_controller_node...';
    roslaunch arm_controller arm_controller_node.launch;
    exec bash
" &
PIDS+=($!)

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}前3个程序已启动！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "已启动的程序:"
echo "  - Gazebo仿真环境"
echo "  - 机械臂控制器 (仿真)"
echo "  - arm_controller_node"
echo ""

# 等待用户输入启动RViz
echo -e "${YELLOW}[5/5] 准备启动RViz...${NC}"
# echo -e "${YELLOW}按 Enter 键启动RViz，或按 Ctrl+C 跳过...${NC}"
# read -r
sleep 3s


# 启动RViz
echo -e "${YELLOW}启动RViz可视化...${NC}"
gnome-terminal --tab --title="RViz" -- zsh -c "
    cd $WORKSPACE_DIR;
    if [ -f devel/setup.zsh ]; then
        source devel/setup.zsh;
    fi
    echo '启动RViz...';
    roslaunch z1_description z1_combine_rviz.launch;
    # roslaunch z1_description z1_flower_rviz.launch;
    exec zsh
" &
PIDS+=($!)

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}所有程序已启动！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "已启动的程序:"
echo "  - Gazebo仿真环境"
echo "  - 机械臂控制器 (仿真)"
echo "  - arm_controller_node"
echo "  - RViz可视化"
echo ""
echo -e "${RED}按 Ctrl+C 一键退出所有程序${NC}"
echo ""

# 捕获Ctrl+C信号，退出所有程序
cleanup() {
    echo -e "\n${YELLOW}正在关闭所有程序...${NC}"
    
    # 关闭所有相关进程
    killall -9 gzserver gzclient 2>/dev/null
    killall -9 sim_ctrl 2>/dev/null
    killall -9 rviz 2>/dev/null
    
    # 关闭ROS节点
    pkill -9 -f "arm_controller_node" 2>/dev/null
    pkill -9 -f "z1_flower.launch" 2>/dev/null
    pkill -9 -f "z1_flower_rviz.launch" 2>/dev/null
    
    # 关闭终端窗口
    kill ${PIDS[@]} 2>/dev/null
    
    # 注意: 不关闭 roscore/rosmaster，因为用户可能在独立终端中运行
    
    echo -e "${GREEN}所有程序已关闭 (保留roscore)${NC}"
    exit 0
}

trap cleanup INT

# 保持脚本运行，等待Ctrl+C
while true; do
    sleep 1
done
