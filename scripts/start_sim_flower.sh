#!/bin/bash

# 启动脚本 - 用于启动机械臂控制系统的所有必要程序
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
echo -e "${GREEN}启动机械臂控制系统 (使用 xterm)${NC}"
echo -e "${GREEN}========================================${NC}"

# 检查工作空间是否存在
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${YELLOW}错误: 工作空间目录不存在: $WORKSPACE_DIR${NC}"
    exit 1
fi

# 检查是否安装了 xterm
if ! command -v xterm &> /dev/null; then
    echo -e "${RED}错误: 未检测到 xterm。请先安装: sudo apt install xterm${NC}"
    exit 1
fi

# 进入工作空间
cd "$WORKSPACE_DIR"

# 1. 启动Gazebo仿真环境 (左上角)
echo -e "${YELLOW}[1/4] 启动Gazebo仿真环境...${NC}"
xterm -T "Gazebo Simulation" -geometry 80x24+0+0 -e "bash -c \"
    cd $WORKSPACE_DIR;
    [ -f devel/setup.bash ] && source devel/setup.bash;
    echo '启动Gazebo仿真环境...';
    roslaunch unitree_gazebo z1_flower.launch;
    exec bash
\"" &
PIDS+=($!)

echo -e "${YELLOW}等待Gazebo启动 (10秒)...${NC}"
sleep 10

# 2. 启动控制器 (右上角)
echo -e "${YELLOW}[2/4] 启动机械臂控制器...${NC}"
xterm -T "Arm Controller" -geometry 80x24+500+0 -e "bash -c \"
    cd $WORKSPACE_DIR/z1_controller/build;
    [ -f $WORKSPACE_DIR/devel/setup.bash ] && source $WORKSPACE_DIR/devel/setup.bash;
    echo '启动机械臂控制器...';
    ./sim_ctrl;
    exec bash
\"" &
PIDS+=($!)

echo -e "${YELLOW}等待控制器启动 (3秒)...${NC}"
sleep 3s

# 3. 启动 arm_controller_node (左下角)
echo -e "${YELLOW}[3/4] 启动 arm_controller_node...${NC}"
xterm -T "Arm Controller Node" -geometry 80x24+0+450 -e "bash -c \"
    cd $WORKSPACE_DIR;
    [ -f devel/setup.bash ] && source devel/setup.bash;
    echo '启动 arm_controller_node...';
    roslaunch arm_controller arm_controller_node.launch;
    exec bash
\"" &
PIDS+=($!)

echo -e "${YELLOW}[4/4] 准备启动RViz...${NC}"
sleep 3s

# 4. 启动RViz (右下角)
echo -e "${YELLOW}启动RViz可视化...${NC}"
xterm -T "RViz" -geometry 80x24+500+450 -e "bash -c \"
    cd $WORKSPACE_DIR;
    [ -f devel/setup.bash ] && source devel/setup.bash;
    echo '启动RViz...';
    roslaunch z1_description z1_flower_rviz.launch;
    exec bash
\"" &
PIDS+=($!)

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}所有程序已通过 xterm 启动！${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${RED}按 Ctrl+C 一键退出所有程序${NC}"
echo ""

# 捕获Ctrl+C信号，退出所有程序
# 捕获Ctrl+C信号，退出所有程序
cleanup() {
    echo -e "\n${YELLOW}正在清理 ROS 进程环境...${NC}"
    
    # 1. 尝试使用 ROS 自带工具优雅关闭所有节点
    # 这会通知所有节点进行正常的析构流程
    rosnode kill -a 2>/dev/null
    sleep 1

    # 2. 关闭仿真器核心
    echo -e "${YELLOW}清理仿真组件...${NC}"
    killall -9 gzserver gzclient 2>/dev/null
    killall -9 rviz 2>/dev/null
    killall -9 sim_ctrl 2>/dev/null

    # 3. 强力清理所有包含特定关键字的 python/cpp 进程
    # 这样可以抓取那些由 roslaunch 启动但名称不直接包含 .launch 的隐藏进程
    echo -e "${YELLOW}深度清理节点进程...${NC}"
    pkill -9 -f "unitree" 2>/dev/null
    pkill -9 -f "arm_controller" 2>/dev/null
    pkill -9 -f "z1_" 2>/dev/null
    
    # 4. 关闭所有启动的终端 (xterm)
    # 使用进程组 ID (PGID) 杀掉整个树比单个 PID 更有效
    for pid in "${PIDS[@]}"; do
        # 杀掉该进程及其所有子进程
        pkill -TERM -P "$pid" 2>/dev/null
        kill -9 "$pid" 2>/dev/null
    done

    # 5. 【可选】彻底重置 ROS 核心 (根据需要决定是否启用)
    # 如果你发现下次启动提示 "master already running"，请取消下面两行的注释
    # echo -e "${YELLOW}重置 rosmaster...${NC}"
    # pkill -9 -f rosmaster 2>/dev/null
    # pkill -9 -f roscore 2>/dev/null

    echo -e "${GREEN}所有程序已彻底清理${NC}"
    exit 0
}

trap cleanup INT

# 保持脚本运行
while true; do
    sleep 1
done