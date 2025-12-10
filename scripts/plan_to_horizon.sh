#!/bin/bash

# Script to call the plan_to_horizon service
# Usage: ./plan_to_horizon.sh [true|false]
# Default: false

PLAN_VALUE=${1:-false}

echo "Calling /arm_controller_node/plan_to_horizon service with plan_to_horizon: $PLAN_VALUE"
rosservice call /arm_controller_node/plan_to_horizon "plan_to_horizon: $PLAN_VALUE"
