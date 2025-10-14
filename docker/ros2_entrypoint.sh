#!/usr/bin/env bash
set -euo pipefail

ROS_UID=${ROS2_UID:-1000}
ROS_GID=${ROS2_GID:-1000}
ROS_USER_DEFAULT=${ROS2_USERNAME:-ros2}
ROS_GROUP_DEFAULT=${ROS2_GROUPNAME:-ros2}
STATE_DIR=/run/ros2_env

create_user_and_group() {
  local target_uid=$1
  local target_gid=$2
  local default_user=$3
  local default_group=$4

  local group_name
  if group_name=$(getent group "${target_gid}" | cut -d: -f1); then
    :
  else
    group_name=${default_group}
    if ! getent group "${group_name}" >/dev/null 2>&1; then
      groupadd -g "${target_gid}" "${group_name}"
    else
      groupmod -g "${target_gid}" "${group_name}"
    fi
  fi

  local user_name
  if user_name=$(getent passwd "${target_uid}" | cut -d: -f1); then
    :
  else
    user_name=${default_user}
    if ! id -u "${user_name}" >/dev/null 2>&1; then
      useradd -m -u "${target_uid}" -g "${target_gid}" -s /bin/bash "${user_name}"
    else
      usermod -u "${target_uid}" -g "${target_gid}" "${user_name}"
    fi
  fi

  local user_home
  user_home=$(getent passwd "${target_uid}" | cut -d: -f6)
  mkdir -p "${user_home}"
  chown "${target_uid}:${target_gid}" "${user_home}"

  mkdir -p "${STATE_DIR}"
  printf '%s\n' "${target_uid}" > "${STATE_DIR}/active_uid"
  printf '%s\n' "${target_gid}" > "${STATE_DIR}/active_gid"
  printf '%s\n' "${user_name}" > "${STATE_DIR}/active_user"
  printf '%s\n' "${group_name}" > "${STATE_DIR}/active_group"

  echo "${user_name}"
}

source_ros_environment() {
  local had_u=0
  case $- in
    *u*)
      had_u=1
      set +u
      ;;
  esac

  # Source the base ROS 2 environment provided by the image
  # shellcheck source=/dev/null
  source /ros_entrypoint.sh

  if [ -f /workspaces/host_ws/install/setup.bash ]; then
    # shellcheck source=/dev/null
    source /workspaces/host_ws/install/setup.bash
  fi

  if [ "${had_u}" -eq 1 ]; then
    set -u
  fi
}

if [ "$(id -u)" -eq 0 ]; then
  ACTIVE_USER=$(create_user_and_group "${ROS_UID}" "${ROS_GID}" "${ROS_USER_DEFAULT}" "${ROS_GROUP_DEFAULT}")
  USER_HOME=$(getent passwd "${ROS_UID}" | cut -d: -f6)
  export HOME=${USER_HOME}
  export USER=${ACTIVE_USER}
  export LOGNAME=${ACTIVE_USER}

  source_ros_environment

  exec gosu "${ACTIVE_USER}" "$@"
else
  source_ros_environment
  exec "$@"
fi
