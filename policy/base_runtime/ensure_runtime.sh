#!/usr/bin/env bash
# Check the native Humble base graph and start/reuse the bundled adapter when requested.
set -eo pipefail

check_only=false
if [[ "${1:-}" == "--check-only" ]]; then
  check_only=true
elif [[ $# -gt 0 ]]; then
  echo '{"status":"failed","error":"unknown base-runtime argument"}' >&2
  exit 64
fi

runtime_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
adapter="${runtime_dir}/cmd_vel_adapter.py"
pid_file="/tmp/tmr_task2_cmd_vel_adapter.pid"
log_file="/tmp/tmr_task2_cmd_vel_adapter.log"

topic_has_subscriber() {
  timeout 3 ros2 topic info /tmr_cycle/mission_cmd_vel --no-daemon 2>/dev/null \
    | grep -Eq '^Subscription count: [1-9][0-9]*$'
}

controller_has_subscriber() {
  timeout 3 ros2 topic info /swerve_drive_controller/cmd_vel --no-daemon 2>/dev/null \
    | grep -Eq '^Subscription count: [1-9][0-9]*$'
}

fresh_odometry() {
  timeout 4 ros2 topic echo --once --no-daemon \
    /swerve_drive_controller/odom >/dev/null 2>&1
}

if ! fresh_odometry; then
  echo '{"status":"failed","error":"fresh base odometry is unavailable"}' >&2
  exit 71
fi

if ! controller_has_subscriber; then
  echo '{"status":"failed","error":"base drive controller is not subscribed to cmd_vel"}' >&2
  exit 74
fi

if ${check_only}; then
  printf '{"status":"success","base_runtime":"read_only","domain":%s}\n' \
    "${ROS_DOMAIN_ID:-0}"
  exit 0
fi

if topic_has_subscriber; then
  printf '{"status":"success","base_runtime":"existing_adapter","domain":%s}\n' \
    "${ROS_DOMAIN_ID:-0}"
  exit 0
fi

if [[ -s "${pid_file}" ]]; then
  old_pid="$(cat "${pid_file}")"
  if kill -0 "${old_pid}" 2>/dev/null; then
    kill -TERM "${old_pid}" 2>/dev/null || true
    sleep 0.3
  fi
  rm -f "${pid_file}"
fi

nohup python3 -u "${adapter}" >"${log_file}" 2>&1 </dev/null &
adapter_pid="$!"
printf '%s\n' "${adapter_pid}" >"${pid_file}"

for _ in $(seq 1 30); do
  if topic_has_subscriber; then
    printf '{"status":"success","base_runtime":"staged_adapter","pid":%s,"domain":%s}\n' \
      "${adapter_pid}" "${ROS_DOMAIN_ID:-0}"
    exit 0
  fi
  if ! kill -0 "${adapter_pid}" 2>/dev/null; then
    echo '{"status":"failed","error":"staged velocity adapter exited"}' >&2
    tail -n 40 "${log_file}" >&2 2>/dev/null || true
    exit 72
  fi
  sleep 0.2
done

echo '{"status":"failed","error":"velocity adapter did not create its subscription"}' >&2
tail -n 40 "${log_file}" >&2 2>/dev/null || true
exit 73
