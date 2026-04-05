#!/usr/bin/env bash
# run_sim_local_nav.sh
#
# Launch a single-robot local navigation simulation in the office world.
# Uses the ROS2 launch system to bring up Gazebo, the robot description,
# all sensor nodes, the SLAM front-end, the occupancy grid mapper, and the
# local / global planners.
#
# Usage:
#   ./scripts/run_sim_local_nav.sh [--headless] [--log-level DEBUG]
#
# Requirements:
#   - ROS2 Humble (or later) sourced in the shell
#   - Gazebo Classic / Ignition installed
#   - Python 3.10+ with the packages listed in pyproject.toml

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

# Default options
HEADLESS=false
LOG_LEVEL="INFO"
WORLD="office_world"
ROBOT_PARAMS="${REPO_ROOT}/config/robot_params.yaml"
SENSOR_CONFIG="${REPO_ROOT}/config/sensors.yaml"

# Parse arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless)
      HEADLESS=true
      shift ;;
    --log-level)
      LOG_LEVEL="$2"
      shift 2 ;;
    --world)
      WORLD="$2"
      shift 2 ;;
    *)
      echo "Unknown argument: $1"
      echo "Usage: $0 [--headless] [--log-level LEVEL] [--world WORLD_NAME]"
      exit 1 ;;
  esac
done

echo "============================================================"
echo " From Signal to System — Single Robot Local Navigation"
echo " World       : ${WORLD}"
echo " Headless    : ${HEADLESS}"
echo " Log level   : ${LOG_LEVEL}"
echo " Robot params: ${ROBOT_PARAMS}"
echo "============================================================"

# Source ROS2 (adjust path for your distro if needed)
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "WARNING: ROS2 Humble setup.bash not found. Assuming ROS2 is already sourced."
fi

# Build the workspace if not already built
if [[ ! -d "${REPO_ROOT}/install" ]]; then
  echo "Building workspace with colcon..."
  (cd "${REPO_ROOT}" && colcon build --symlink-install)
  # shellcheck disable=SC1090
  source "${REPO_ROOT}/install/setup.bash"
fi

# Launch
ros2 launch "${REPO_ROOT}/launch/single_robot_nav.launch.py" \
  world:="${WORLD}" \
  headless:="${HEADLESS}" \
  log_level:="${LOG_LEVEL}" \
  robot_params:="${ROBOT_PARAMS}" \
  sensor_config:="${SENSOR_CONFIG}"
