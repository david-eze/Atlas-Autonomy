#!/bin/bash
set -e

# Source ROS 2 and the workspace if it has been built.
source /opt/ros/jazzy/setup.bash
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

exec "$@"