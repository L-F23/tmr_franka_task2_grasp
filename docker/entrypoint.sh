#!/usr/bin/env bash
set -eo pipefail

readonly policy_root="${POLICY_ROOT:-/opt/tmr-task2/policy}"
readonly ros_env_file="${ROS_ENV_FILE:-/home/aup/tmr_env.sh}"

usage() {
  cat <<'EOF'
Usage: docker run [docker options] IMAGE MODE [arguments]

Modes:
  preflight          Validate the mounted testbed environment; send no motion.
  check              Run the read-only ROS and camera health checks.
  prepare            Prepare healthy services; do not launch the motion policy.
  execute            Launch the complete Task 2 policy with physical motion.
  start-project      Initialize the robot and run live red-strip detection.
  offline-detect     Run detect_red_strip.py with the supplied arguments.
  shell              Open a Bash shell in the prepared policy environment.
  help               Show this message.

The default mode is "check". Physical motion requires the explicit "execute"
or "start-project" mode.
EOF
}

fail() {
  echo "container preflight failed: $*" >&2
  exit 78
}

if [[ "${1:-}" == "help" || "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

[[ -d "${policy_root}" ]] || fail "policy root is missing: ${policy_root}"
[[ -r /opt/ros/jazzy/setup.bash ]] || fail "ROS 2 Jazzy is missing from the image"
[[ -r "${ros_env_file}" ]] || fail \
  "mount the testbed /home/aup directory read-only; missing ${ros_env_file}"
[[ -d /home/aup/tmr-mobile-manipulation ]] || fail \
  "missing testbed reference project: /home/aup/tmr-mobile-manipulation"
[[ -d /home/aup/.ssh ]] || fail "missing testbed SSH configuration: /home/aup/.ssh"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# The testbed file loads the custom Franka and Spine message overlays and sets
# the deployed CycloneDDS configuration. The host directory is mounted at the
# same absolute path so every prefix recorded by the setup files remains valid.
# shellcheck disable=SC1090
source "${ros_env_file}"

export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
export ROS_HOME="${ROS_HOME:-/tmp/tmr-ros}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${ROS_HOME}/log}"
export XDG_CACHE_HOME="${XDG_CACHE_HOME:-/tmp/tmr-cache}"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}" "${XDG_CACHE_HOME}"
cd "${policy_root}"

for command in curl ros2 screen ssh; do
  command -v "${command}" >/dev/null 2>&1 || fail "required command not found: ${command}"
done

/usr/bin/python3 - <<'PY' || fail "required Python or ROS message packages are unavailable"
import cv2
import numpy
import rclpy
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import ListControllers, SwitchController
from franka_msgs.action import ErrorRecovery, PTPMotion
from franka_msgs.msg import FrankaRobotState
from franka_spine_msgs.srv import GetPosition
from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK, GetStateValidity
from realsense2_camera_msgs.msg import Extrinsics
PY

mode="${1:-check}"
if [[ $# -gt 0 ]]; then
  shift
fi

echo "TMR Task 2 policy revision: ${TMR_TASK2_POLICY_REVISION:-unknown}" >&2

case "${mode}" in
  preflight)
    echo "Container preflight passed; no motion command was sent."
    ;;
  check)
    exec /usr/bin/python3 -u quick_start.py --check-only "$@"
    ;;
  prepare)
    exec /usr/bin/python3 -u quick_start.py "$@"
    ;;
  execute)
    exec /usr/bin/python3 -u quick_start.py --execute "$@"
    ;;
  start-project)
    exec /usr/bin/python3 -u start_project.py "$@"
    ;;
  offline-detect)
    exec /usr/bin/python3 -u detect_red_strip.py "$@"
    ;;
  shell)
    exec /bin/bash "$@"
    ;;
  *)
    usage >&2
    fail "unknown mode: ${mode}"
    ;;
esac
