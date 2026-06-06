#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-/home/lifng0/workspace/balance_car}"
DOCKER_DIR="${WORKSPACE_ROOT}/docker/ros2"
BACKEND_HOST="${BACKEND_HOST:-127.0.0.1}"
BACKEND_PORT="${BACKEND_PORT:-8765}"
WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-30}"
ENABLE_NAVIGATION="${ENABLE_NAVIGATION:-false}"
NAVIGATION_TASK="${NAVIGATION_TASK:-avoid}"
ROS2_CONTAINER_NAME="${ROS2_CONTAINER_NAME:-balance-car-ros2-dev}"
ROS2_PACKAGES="${ROS2_PACKAGES:-balance_car_interfaces balance_car_bridge balance_car_lidar balance_car_behavior balance_car_perception balance_car_navigation balance_car_bringup}"

log() {
  printf '[ros2-autostart] %s\n' "$*" >&2
}

ros_setup_prefix='set +u; source /opt/ros/humble/setup.bash; set -u'

wait_for_backend() {
  local deadline
  deadline=$((SECONDS + WAIT_TIMEOUT_SEC))

  while (( SECONDS < deadline )); do
    if python3 "${WORKSPACE_ROOT}/scripts/pi_backend_client.py" \
      --host "${BACKEND_HOST}" \
      --port "${BACKEND_PORT}" \
      --timeout 1.0 \
      ping >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done

  return 1
}

cd "${DOCKER_DIR}"

log "waiting for coordinator backend ${BACKEND_HOST}:${BACKEND_PORT}"
if ! wait_for_backend; then
  log "coordinator backend not ready within ${WAIT_TIMEOUT_SEC}s"
  exit 1
fi

log "starting ros2 container ${ROS2_CONTAINER_NAME}"
docker compose -f compose.yaml up -d ros2-dev

log "verifying ros2 workspace packages inside ${ROS2_CONTAINER_NAME}"
docker exec "${ROS2_CONTAINER_NAME}" bash -lc "\
  set -euo pipefail; \
  cd /workspaces/balance_car/ws; \
  ${ros_setup_prefix}; \
  if [ ! -f install/setup.bash ] || ! (set +u; source install/setup.bash; set -u; ros2 pkg prefix balance_car_navigation >/dev/null 2>&1); then \
    colcon build --symlink-install --packages-select ${ROS2_PACKAGES}; \
  fi"

log "starting ros2 bringup inside ${ROS2_CONTAINER_NAME}"
exec docker exec "${ROS2_CONTAINER_NAME}" bash -lc "\
  set -euo pipefail; \
  cd /workspaces/balance_car/ws; \
  ${ros_setup_prefix}; \
  set +u; \
  source /workspaces/balance_car/ws/install/setup.bash; \
  set -u; \
  exec ros2 launch balance_car_bringup system.launch.py \
    enable_navigation:=${ENABLE_NAVIGATION} \
    navigation_task:=${NAVIGATION_TASK}"
