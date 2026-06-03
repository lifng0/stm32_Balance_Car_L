#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /workspaces/balance_car/ws/install/setup.bash
set -u

ros2 launch balance_car_bringup bridge_only.launch.py
