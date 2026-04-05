#!/usr/bin/env bash
# run_multi_robot_explore.sh
#
# Launch a multi-robot exploration simulation with configurable communication
# constraints.  Brings up N robots in a shared Gazebo world, each running its
# own perception / SLAM pipeline, connected to a shared base-station node
# through the link simulator.
#
# Usage:
#   ./scripts/run_multi_robot_explore.sh [--robots N] [--bandwidth BPS] \
#       [--scenario SCENARIO_FILE] [--headless]
#
# Example:
#   ./scripts/run_multi_robot_explore.sh --robots 3 --bandwidth 250000

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

# Defaults
N_ROBOTS=2
BANDWIDTH_BPS=500000      # 500 kbps uplink budget
LATENCY_MS=30
PACKET_LOSS=0.02
HEADLESS=false
SCENARIO="${REPO_ROOT}/config/exploration_scenarios.yaml"
COMMS_QOS="${REPO_ROOT}/config/comms_qos_profiles.yaml"
WORLD="warehouse_world"

# Parse arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robots)       N_ROBOTS="$2";       shift 2 ;;
    --bandwidth)    BANDWIDTH_BPS="$2";  shift 2 ;;
    --latency)      LATENCY_MS="$2";     shift 2 ;;
    --packet-loss)  PACKET_LOSS="$2";    shift 2 ;;
    --scenario)     SCENARIO="$2";       shift 2 ;;
    --world)        WORLD="$2";          shift 2 ;;
    --headless)     HEADLESS=true;       shift ;;
    *)
      echo "Unknown argument: $1"
      exit 1 ;;
  esac
done

echo "============================================================"
echo " From Signal to System — Multi-Robot Exploration"
echo " Robots      : ${N_ROBOTS}"
echo " Bandwidth   : ${BANDWIDTH_BPS} bps"
echo " Latency     : ${LATENCY_MS} ms"
echo " Packet loss : ${PACKET_LOSS}"
echo " World       : ${WORLD}"
echo " Scenario    : ${SCENARIO}"
echo "============================================================"

# Source ROS2
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "WARNING: ROS2 Humble setup.bash not found. Assuming ROS2 is already sourced."
fi

# Optionally build
if [[ ! -d "${REPO_ROOT}/install" ]]; then
  echo "Building workspace with colcon..."
  (cd "${REPO_ROOT}" && colcon build --symlink-install)
  # shellcheck disable=SC1090
  source "${REPO_ROOT}/install/setup.bash"
fi

# Launch the multi-robot exploration
ros2 launch "${REPO_ROOT}/launch/multi_robot_explore.launch.py" \
  n_robots:="${N_ROBOTS}" \
  world:="${WORLD}" \
  headless:="${HEADLESS}" \
  bandwidth_bps:="${BANDWIDTH_BPS}" \
  base_latency_ms:="${LATENCY_MS}" \
  packet_loss_rate:="${PACKET_LOSS}" \
  scenario_file:="${SCENARIO}" \
  comms_qos_file:="${COMMS_QOS}"
