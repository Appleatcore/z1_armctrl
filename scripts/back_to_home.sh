#!/bin/bash

# 机械臂回到home位置脚本

echo "=========================================="
echo "机械臂回到Home位置"
echo "=========================================="

echo ""
echo "正在执行回到home位置..."
rosservice call /arm_controller_node/back_to_home "back_to_home: false"

if [ $? -eq 0 ]; then
    echo "✓ 回到home位置成功"
    exit 0
else
    echo "✗ 回到home位置失败"
    exit 1
fi
