#!/bin/bash

# 测试 plan_to_horizon_height 服务
# 用法: ./plan_to_horizon_height.sh [height_value]
# 默认 height 值为 0.0

HEIGHT_VALUE=${1:-0.0}

echo "Calling /arm_controller_node/plan_to_horizon_height service with height: $HEIGHT_VALUE"
rosservice call /arm_controller_node/plan_to_horizon_height "height: $HEIGHT_VALUE"
