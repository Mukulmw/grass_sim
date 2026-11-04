#!/usr/bin/env bash
# Start the GRASS kinematics stick model in RViz (sliders + FK/IK check).
# Usage: ./run_kinematics.sh [--build] [extra launch args, e.g. robot_config:=/path/robot.yaml]
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"

source /opt/ros/humble/setup.bash

if [ "$1" = "--build" ] || [ ! -f install/setup.bash ]; then
  [ "$1" = "--build" ] && shift
  colcon build --symlink-install
fi

source install/setup.bash
exec ros2 launch grass_sim kinematics.launch.py "$@"
