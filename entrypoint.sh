#!/bin/bash
set -e

# Source ROS 2 humble
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# Source a local workspace if present
if [ -f "$HOME/workspace/install/setup.bash" ]; then
  source "$HOME/workspace/install/setup.bash"
elif [ -f "$HOME/workspace/install/local_setup.bash" ]; then
  source "$HOME/workspace/install/local_setup.bash"
fi

exec "$@"
