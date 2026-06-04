#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-/home/lifng0/workspace/balance_car}"
DOCKER_DIR="${WORKSPACE_ROOT}/docker/ros2"
BACKEND_HOST="${BACKEND_HOST:-127.0.0.1}"
BACKEND_PORT="${BACKEND_PORT:-8765}"
WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-30}"
ENABLE_NAVIGATION="${ENABLE_NAVIGATION:-false}"
NAVIGATION_TASK="${NAVIGATION_TASK:-avoid}"

log() {
  printf '[ros2-autostart] %s\n' "$*" >&2
}

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

log "starting docker ros2 bringup"
exec docker compose -f compose.yaml run --rm --no-deps ros2-dev bash -lc \
  "source /opt/ros/humble/setup.bash && \
   source /workspaces/balance_car/ws/install/setup.bash && \
   ros2 launch balance_car_bringup system.launch.py \
     enable_navigation:=${ENABLE_NAVIGATION} \
     navigation_task:=${NAVIGATION_TASK}"
