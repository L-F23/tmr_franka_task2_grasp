#!/usr/bin/env bash
set -eo pipefail

readonly policy_root="${POLICY_ROOT:-/opt/tmr-task2/policy}"
readonly interface_overlay="/opt/tmr-interfaces/install/setup.bash"

usage() {
  cat <<'EOF'
Usage: docker run [docker options] IMAGE MODE [arguments]

Modes:
  preflight          Validate the self-contained image; send no motion.
  zed-bridge         Bridge only the ZED topic between two DDS domains.
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
[[ -r /opt/ros/humble/setup.bash ]] || fail "ROS 2 Humble is missing from the image"
[[ -r "${interface_overlay}" ]] || fail \
  "bundled Franka interface overlay is missing: ${interface_overlay}"
[[ -r /opt/tmr-task2/docker/zed_domain_bridge.yaml ]] || fail \
  "bundled ZED domain-bridge configuration is missing"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "${interface_overlay}"

ros2 pkg prefix domain_bridge >/dev/null 2>&1 || fail \
  "bundled ROS 2 domain_bridge package is unavailable"

export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
export ROS_HOME="${ROS_HOME:-/tmp/tmr-ros}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${ROS_HOME}/log}"
export XDG_CACHE_HOME="${XDG_CACHE_HOME:-/tmp/tmr-cache}"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}" "${XDG_CACHE_HOME}"
cd "${policy_root}"

for command in curl ros2; do
  command -v "${command}" >/dev/null 2>&1 || fail "required command not found: ${command}"
done

/usr/bin/python3 - <<'PY' || fail "required Python or ROS message packages are unavailable"
import cv2
import numpy
import rclpy
from cv_bridge import CvBridge
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import ListControllers, SwitchController
from franka_msgs.action import ErrorRecovery, PTPMotion
from franka_msgs.msg import FrankaRobotState
from franka_spine_msgs.srv import GetPosition
from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK, GetStateValidity
from realsense2_camera_msgs.msg import Extrinsics
from sensor_msgs.msg import CompressedImage, Image
PY

start_camera_viewer() {
  if curl --noproxy '*' --silent --fail --max-time 1 \
      http://localhost:18081/status.json >/dev/null 2>&1; then
    return
  fi
  /usr/bin/python3 -u camera_viewer.py --port 18081 \
    >/tmp/tmr_task2_camera_viewer.log 2>&1 &
  readonly viewer_pid=$!
  for _ in $(seq 1 40); do
    if curl --noproxy '*' --silent --fail --max-time 1 \
        http://localhost:18081/status.json >/dev/null 2>&1; then
      echo "Bundled camera viewer started (pid ${viewer_pid})." >&2
      return
    fi
    if ! kill -0 "${viewer_pid}" 2>/dev/null; then
      fail "camera viewer exited; see /tmp/tmr_task2_camera_viewer.log"
    fi
    sleep 0.25
  done
  fail "camera viewer did not become ready; see /tmp/tmr_task2_camera_viewer.log"
}

mode="${1:-check}"
if [[ $# -gt 0 ]]; then
  shift
fi

echo "TMR Task 2 policy revision: ${TMR_TASK2_POLICY_REVISION:-unknown}" >&2

case "${mode}" in
  preflight)
    echo "Container preflight passed; no motion command was sent."
    ;;
  zed-bridge)
    zed_domain="${TMR_ZED_DOMAIN_ID:-1}"
    robot_domain="${TMR_ROBOT_DOMAIN_ID:-0}"
    [[ "${zed_domain}" =~ ^[0-9]+$ && "${zed_domain}" -le 232 ]] || fail \
      "TMR_ZED_DOMAIN_ID must be an integer from 0 through 232"
    [[ "${robot_domain}" =~ ^[0-9]+$ && "${robot_domain}" -le 232 ]] || fail \
      "TMR_ROBOT_DOMAIN_ID must be an integer from 0 through 232"
    [[ "${zed_domain}" != "${robot_domain}" ]] || fail \
      "ZED and robot domains are identical; a domain bridge is unnecessary"
    exec ros2 run domain_bridge domain_bridge \
      --from "${zed_domain}" \
      --to "${robot_domain}" \
      /opt/tmr-task2/docker/zed_domain_bridge.yaml "$@"
    ;;
  check)
    start_camera_viewer
    exec /usr/bin/python3 -u quick_start.py --check-only "$@"
    ;;
  prepare)
    start_camera_viewer
    exec /usr/bin/python3 -u quick_start.py "$@"
    ;;
  execute)
    start_camera_viewer
    exec /usr/bin/python3 -u quick_start.py --execute "$@"
    ;;
  start-project)
    start_camera_viewer
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
