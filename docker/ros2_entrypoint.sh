#!/usr/bin/env bash
set -e

# Source the base ROS 2 environment provided by the image
source /ros_entrypoint.sh

# If the host workspace has been built, add it to the environment
if [ -f /workspaces/host_ws/install/setup.bash ]; then
  source /workspaces/host_ws/install/setup.bash
fi

exec "$@"
