#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

cd /workspaces/balance_car/ws

colcon build --symlink-install \
  --packages-select \
  balance_car_interfaces \
  balance_car_bridge \
  balance_car_lidar \
  balance_car_behavior \
  balance_car_perception \
  balance_car_navigation \
  balance_car_bringup
