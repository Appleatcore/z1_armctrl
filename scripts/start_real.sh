#!/bin/zsh

# 启动脚本 - 用于启动机械臂控制系统的所有必要程序
# 使用方法: ./start_sim.sh

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 工作空间路径
WORKSPACE_DIR="/home/jetson/workspace/arm_ctrl"

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
# echo -e "${YELLOW}[1/3] 加载ROS环境...${NC}"
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

# 启动控制器
echo -e "${YELLOW}[2/3] 启动机械臂控制器...${NC}"
gnome-terminal --tab --title="Arm Controller" -- zsh -c "
    cd $WORKSPACE_DIR/z1_controller/build;
    if [ -f $WORKSPACE_DIR/devel/setup.zsh ]; then
        source $WORKSPACE_DIR/devel/setup.zsh;
    fi
    echo '启动机械臂控制器...';
    ./z1_ctrl;
    exec zsh
" &
PIDS+=($!)

# 等待控制器启动
echo -e "${YELLOW}等待控制器启动 (3秒)...${NC}"
sleep 3

# 启动 arm_controller_node
echo -e "${YELLOW}[3/3] 启动 arm_controller_node...${NC}"
gnome-terminal --tab --title="Arm Controller Node" -- zsh -c "
    cd $WORKSPACE_DIR;
    if [ -f devel/setup.zsh ]; then
        source devel/setup.zsh;
    elif [ -f devel/setup.bash ]; then
        source devel/setup.bash;
    fi
    echo '启动 arm_controller_node...';
    roslaunch arm_controller arm_controller_node.launch;
    exec zsh
" &
PIDS+=($!)

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}前3个程序已启动！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "已启动的程序:"
echo "  - 机械臂控制器 (真实机器人)"
echo "  - arm_controller_node"
echo ""

# 定义清理函数
cleanup() {
    echo ""
    echo -e "${YELLOW}正在关闭所有程序...${NC}"
    
    # 关闭所有相关进程
    killall -9 z1_ctrl 2>/dev/null
    killall -9 rviz 2>/dev/null
    
    # 关闭ROS节点
    pkill -9 -f "arm_controller_node" 2>/dev/null
    pkill -9 -f "z1_rviz.launch" 2>/dev/null
    
    # 关闭所有gnome-terminal窗口(通过PID)
    for pid in "${PIDS[@]}"; do
        kill -9 $pid 2>/dev/null
    done
    
    # 关闭所有相关的gnome-terminal进程
    pkill -9 -f "gnome-terminal.*Arm Controller" 2>/dev/null
    pkill -9 -f "gnome-terminal.*Arm Controller Node" 2>/dev/null
    pkill -9 -f "gnome-terminal.*RViz" 2>/dev/null
    
    echo -e "${GREEN}所有程序和终端已关闭${NC}"
    exit 0
}

# 捕获Ctrl+C信号
trap cleanup INT

# 等待用户输入
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}前3个程序运行中...${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}请选择操作:${NC}"
echo -e "  ${GREEN}[Enter]${NC} - 启动RViz可视化"
echo -e "  ${RED}[1]${NC}     - 跳过RViz,保持当前程序运行"
echo -e "  ${RED}[Ctrl+C]${NC} - 终止所有程序并退出"
echo ""

# 读取用户输入
read -k 1 user_input
echo ""

if [[ "$user_input" == "1" ]]; then
    # 用户按了1,跳过RViz
    echo -e "${YELLOW}跳过RViz启动${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}当前运行的程序:${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo "已启动的程序:"
    echo "  - 机械臂控制器 (真实机器人)"
    echo "  - arm_controller_node"
    echo ""
    echo -e "${RED}按 Ctrl+C 终止所有程序并退出${NC}"
    echo ""
else
    # 用户按了Enter,启动RViz
    echo -e "${YELLOW}启动RViz可视化...${NC}"
    gnome-terminal --tab --title="RViz" -- zsh -c "
        cd $WORKSPACE_DIR;
        if [ -f devel/setup.zsh ]; then
            source devel/setup.zsh;
        elif [ -f devel/setup.bash ]; then
            source devel/setup.bash;
        fi
        echo '启动RViz...';
        roslaunch z1_description z1_rviz.launch;
        exec zsh
    " &
    PIDS+=($!)
    
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}所有程序已启动！${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo "已启动的程序:"
    echo "  - 机械臂控制器 (真实机器人)"
    echo "  - arm_controller_node"
    echo "  - RViz可视化"
    echo ""
    echo -e "${RED}按 Ctrl+C 终止所有程序并退出${NC}"
    echo ""
fi

# 保持脚本运行,等待Ctrl+C
while true; do
    sleep 1
done
