# Docker deployment

The image is self-contained at build time. It includes the Task 2 policy,
OpenCV/NumPy, ROS 2 Jazzy, CycloneDDS configuration, the camera-to-MJPEG
bridge, and the exact `franka_msgs` and `franka_spine_msgs` interfaces used by
the policy. It does **not** require `/home/aup/tmr_env.sh`,
`/home/aup/tmr-mobile-manipulation`, `/home/aup/.ssh`, or `/run/screen`.

The default command is the read-only `check` mode. Physical motion starts only
through the explicit `execute` mode.

## Operator commands

Run on the robot computer at `172.16.0.100`. Set `POLICY_COMMIT` to the same
full 40-character SHA entered in the submission form.

```bash
set -euo pipefail

export POLICY_REPOSITORY='https://github.com/L-F23/tmr_franka_task2_grasp.git'
export POLICY_COMMIT='<FULL_40_CHARACTER_PINNED_COMMIT_SHA>'
export POLICY_IMAGE="tmr-task2-policy:${POLICY_COMMIT}"
export POLICY_CHECKOUT="$PWD/tmr-task2-policy-${POLICY_COMMIT}"

test ! -e "$POLICY_CHECKOUT"
git clone "$POLICY_REPOSITORY" "$POLICY_CHECKOUT"
cd "$POLICY_CHECKOUT"
git checkout --detach "$POLICY_COMMIT"
test "$(git rev-parse HEAD)" = "$POLICY_COMMIT"
test -z "$(git status --porcelain)"

docker build --pull \
  --build-arg POLICY_UID="$(id -u)" \
  --build-arg POLICY_GID="$(id -g)" \
  --build-arg POLICY_REVISION="$POLICY_COMMIT" \
  --tag "$POLICY_IMAGE" \
  .

docker volume create "tmr-task2-config-${POLICY_COMMIT}"
docker volume create "tmr-task2-outputs-${POLICY_COMMIT}"
docker volume create "tmr-task2-runtime-${POLICY_COMMIT}"
```

Validate the image without contacting the robot graph or sending motion:

```bash
docker run --rm \
  --network host \
  --ipc host \
  "$POLICY_IMAGE" preflight
```

Run the read-only ROS graph and camera health check:

```bash
docker run --rm \
  --network host \
  --ipc host \
  --mount type=volume,src="tmr-task2-config-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/config \
  --mount type=volume,src="tmr-task2-outputs-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/outputs \
  --mount type=volume,src="tmr-task2-runtime-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/runtime \
  "$POLICY_IMAGE" check
```

After the standard testbed services are running, launch the policy directly;
no authentication file, SSH agent, or host-side project directory is needed:

```bash
docker run --rm --interactive --tty \
  --name tmr-task2-policy \
  --stop-timeout 10 \
  --network host \
  --ipc host \
  --mount type=volume,src="tmr-task2-config-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/config \
  --mount type=volume,src="tmr-task2-outputs-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/outputs \
  --mount type=volume,src="tmr-task2-runtime-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/runtime \
  "$POLICY_IMAGE" execute
```

Before `execute`, confirm the required initial pose, gripper state, clear work
area, reachable emergency stop, and that the standard arm, camera, Spine, and
base services are already running. The policy performs a fail-closed preflight
and does not start duplicate hardware drivers.

## Entrypoint

```text
/usr/bin/tini -- /opt/tmr-task2/docker/entrypoint.sh
```

For `execute`, the entrypoint sources `/opt/ros/jazzy/setup.bash` and the
built-in `/opt/tmr-interfaces/install/setup.bash`, starts the bundled camera
bridge, validates all required Python and ROS interfaces, and launches:

```text
/usr/bin/python3 -u /opt/tmr-task2/policy/quick_start.py --execute
```

The writable `/tmp/tmr_task2_motion.lock` is created inside the container; no
host lock file is required. The named volumes preserve calibration/config,
output images, and runtime records. No `--privileged`, device mount,
`/home/aup` bind mount, or `/run/screen` bind mount is used.

## Environment and dependencies

- Base image: digest-pinned `ros:jazzy-ros-base-noble` (Ubuntu 24.04,
  ROS 2 Jazzy).
- ROS middleware: `rmw_cyclonedds_cpp`, domain 0, host networking, bundled DDS
  configuration with automatic interface selection and explicit peers
  `172.16.0.50` and `172.16.0.100`.
- Franka interfaces: upstream `franka_ros2` v3.4.1 at commit
  `b4164f555500fe50c3f44f24d4cccc452ffac442`, built in the image.
- GPU/CUDA: not used; no GPU, CUDA toolkit, or NVIDIA driver is required.
- Runtime network: no Internet access is required. Layer-2/IP access to the
  testbed controllers and ROS graph is required. No runtime authentication is
  required by the policy.
- Host: Linux with Docker Engine, host networking support, and enough free
  disk space to build the ROS image.

## Hardware assumptions

- Two deployed Franka FR3 arms, Robotiq grippers, Franka Spine, ZED head
  camera, and left/right Intel RealSense D405 wrist cameras are started by the
  normal testbed procedure before policy launch.
- The policy expects the ROS service/action/topic names checked by
  `policy/quick_start.py`; camera topics can be overridden with
  `TMR_MAIN_CAMERA_TOPIC`, `TMR_LEFT_CAMERA_TOPIC`, and
  `TMR_RIGHT_CAMERA_TOPIC`.
- The mobile base controller publishes `/swerve_drive_controller/odom` and
  subscribes to `/tmr_cycle/mission_cmd_vel` on the testbed DDS graph. The
  bundled odometry-closed-loop worker runs inside the host-network container.
  If the testbed uses a nonzero DDS domain, pass it as
  `--env TMR_BASE_ROS_DOMAIN_ID=<id>`.
- No GPU inference rate applies. Camera freshness is checked approximately
  one second apart; base commands use live odometry and bounded timeouts.
- Camera placement, hand-eye transform, arm initial pose, table/object layout,
  and calibrated distances must match the Task 2 testbed setup shown in the
  demo and stored under `policy/config/`. Lighting must allow the red/gray/black
  color and structure detectors to separate their targets.
- Between rounds, return both arms, Spine, base, grippers, thermal pad, and red
  placement pad to their documented initial state, clear previous objects from
  the path, and verify the emergency stop.
