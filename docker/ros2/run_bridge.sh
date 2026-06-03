#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /workspaces/balance_car/ws/install/setup.bash
set -u

ros2 run balance_car_bridge bridge_node
