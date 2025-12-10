#!/bin/bash

# Script to call the plan_to_default service
# Usage: ./plan_to_default.sh [true|false]
# Default: false

PLAN_VALUE=${1:-false}

echo "Calling /arm_controller_node/plan_to_default service with plan_to_default: $PLAN_VALUE"
rosservice call /arm_controller_node/plan_to_default "plan_to_default: $PLAN_VALUE"
