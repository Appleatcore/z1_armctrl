#!/bin/bash

# 清理所有ROS相关进程的脚本
# 使用方法: ./kill_all_ros.sh

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${RED}========================================${NC}"
echo -e "${RED}清理所有ROS相关进程${NC}"
echo -e "${RED}========================================${NC}"
echo ""

# 清理Gazebo相关进程
echo -e "${YELLOW}[1/6] 清理Gazebo进程...${NC}"
killall -9 gzserver gzclient 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}  ✓ Gazebo进程已清理${NC}"
else
    echo -e "  - 没有Gazebo进程运行"
fi

# 清理机械臂控制器
echo -e "${YELLOW}[2/6] 清理机械臂控制器...${NC}"
killall -9 sim_ctrl z1_ctrl real_ctrl 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}  ✓ 控制器进程已清理${NC}"
else
    echo -e "  - 没有控制器进程运行"
fi

# 清理RViz
echo -e "${YELLOW}[3/6] 清理RViz...${NC}"
killall -9 rviz 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}  ✓ RViz进程已清理${NC}"
else
    echo -e "  - 没有RViz进程运行"
fi

# 清理roslaunch启动的节点
echo -e "${YELLOW}[4/6] 清理roslaunch节点...${NC}"
pkill -9 -f "roslaunch" 2>/dev/null
pkill -9 -f "arm_controller_node" 2>/dev/null
pkill -9 -f "z1_camera.launch" 2>/dev/null
pkill -9 -f "z1_rviz.launch" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}  ✓ roslaunch节点已清理${NC}"
else
    echo -e "  - 没有roslaunch节点运行"
fi

# 清理robot_state_publisher等
echo -e "${YELLOW}[5/6] 清理其他ROS节点...${NC}"
pkill -9 -f "robot_state_publisher" 2>/dev/null
pkill -9 -f "controller_spawner" 2>/dev/null
pkill -9 -f "spawn_model" 2>/dev/null
echo -e "${GREEN}  ✓ 其他节点已清理${NC}"

# 清理ROS核心（roscore/rosmaster）
echo -e "${YELLOW}[6/6] 清理ROS核心...${NC}"
killall -9 roscore rosmaster rosout 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}  ✓ ROS核心已清理${NC}"
else
    echo -e "  - 没有ROS核心运行"
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}所有ROS进程清理完成！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 显示剩余的ROS相关进程（如果有）
REMAINING=$(ps aux | grep -E "ros|gazebo|rviz" | grep -v grep | grep -v "kill_all_ros.sh")
if [ -n "$REMAINING" ]; then
    echo -e "${YELLOW}警告: 仍有以下进程在运行:${NC}"
    echo "$REMAINING"
else
    echo -e "${GREEN}✓ 确认: 没有ROS相关进程在运行${NC}"
fi
