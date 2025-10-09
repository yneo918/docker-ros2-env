#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
ENV_FILE="${REPO_ROOT}/.env"

if [ -f "${ENV_FILE}" ]; then
  set -a
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
  set +a
fi

CONTAINER_NAME=${ROS2_CONTAINER_NAME:-ros2_dev}
ROS_DISTRO_NAME=${ROS_DISTRO:-jazzy}

if [ $# -lt 1 ]; then
  echo "Usage: $(basename "$0") <ros2 arguments...>" >&2
  exit 1
fi

if ! status=$(docker container inspect -f '{{.State.Status}}' "${CONTAINER_NAME}" 2>/dev/null); then
  echo "Container '${CONTAINER_NAME}' not found. Start it with 'docker compose up -d ros2' first." >&2
  exit 1
fi

if [ "${status}" != "running" ]; then
  echo "Container '${CONTAINER_NAME}' is not running (status: ${status}). Start it with 'docker compose up -d ros2' first." >&2
  exit 1
fi

if [[ -t 1 ]]; then
  DOCKER_TTY_ARGS=(-it)
else
  DOCKER_TTY_ARGS=(-i)
fi

ros_cmd=""
for arg in "$@"; do
  ros_cmd+=" $(printf '%q' "${arg}")"
done

exec docker exec "${DOCKER_TTY_ARGS[@]}" "${CONTAINER_NAME}" bash -lc "source /opt/ros/${ROS_DISTRO_NAME}/setup.bash; if [ -f /workspaces/host_ws/install/setup.bash ]; then source /workspaces/host_ws/install/setup.bash; fi; ros2${ros_cmd}"
