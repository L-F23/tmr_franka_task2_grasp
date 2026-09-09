# Docker deployment

The image contains the complete Task 2 policy and its ROS 2 Jazzy client
environment. Arm, Spine, IK, and wrist-camera clients run in the host-network
container on robot computer `.100`. At launch, the entrypoint stages the
bundled base worker and zero-latching velocity adapter into an isolated
`/tmp/tmr-task2-*` directory on `.50`; those processes run in the base
computer's native ROS 2 Humble domain.

This preserves the deployed Humble/Jazzy and DDS-domain boundary used by the
validated reference implementation. The image does not require
`/home/aup/tmr_env.sh`, `/home/aup/tmr-mobile-manipulation`,
`/home/aup/.ssh`, or `/run/screen`.

The default mode is the read-only `check` mode. Physical motion starts only
through the explicit `execute` mode.

## Operator commands

Run on robot computer `172.16.0.100`. Set `POLICY_COMMIT` to the full SHA
submitted for evaluation.

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

docker volume create "tmr-task2-config-${POLICY_COMMIT}" >/dev/null
docker volume create "tmr-task2-outputs-${POLICY_COMMIT}" >/dev/null
docker volume create "tmr-task2-runtime-${POLICY_COMMIT}" >/dev/null

export POLICY_SSH_AUTH_SOCK="${SSH_AUTH_SOCK:?the existing .50 login must be loaded in ssh-agent}"
test -S "$POLICY_SSH_AUTH_SOCK"
```

Validate the image without contacting the robot graph or sending motion:

```bash
docker run --rm "$POLICY_IMAGE" preflight
```

Verify the testbed's already-configured, non-interactive base login by
forwarding its SSH-agent socket. No private key, `.ssh` directory, password,
or credential file is copied or mounted into the image:

```bash
docker run --rm --network host \
  --mount type=bind,src="$POLICY_SSH_AUTH_SOCK",dst=/tmp/tmr-task2-ssh-agent.sock \
  --env SSH_AUTH_SOCK=/tmp/tmr-task2-ssh-agent.sock \
  "$POLICY_IMAGE" shell -lc \
  'ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new \
    -o UserKnownHostsFile=/tmp/tmr_task2_known_hosts \
    tmr-user@172.16.0.50 true'
```

After the normal testbed services are running, perform the read-only Jazzy ROS
graph and camera health check:

```bash
docker run --rm \
  --network host \
  --ipc host \
  --mount type=volume,src="tmr-task2-config-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/config \
  --mount type=volume,src="tmr-task2-outputs-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/outputs \
  --mount type=volume,src="tmr-task2-runtime-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/runtime \
  "$POLICY_IMAGE" check
```

After confirming the initial state, clear workspace, and reachable emergency
stop, launch the physical policy:

```bash
docker run --rm --interactive --tty \
  --name tmr-task2-policy \
  --stop-timeout 10 \
  --network host \
  --ipc host \
  --mount type=bind,src="$POLICY_SSH_AUTH_SOCK",dst=/tmp/tmr-task2-ssh-agent.sock \
  --env SSH_AUTH_SOCK=/tmp/tmr-task2-ssh-agent.sock \
  --mount type=volume,src="tmr-task2-config-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/config \
  --mount type=volume,src="tmr-task2-outputs-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/outputs \
  --mount type=volume,src="tmr-task2-runtime-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/runtime \
  "$POLICY_IMAGE" execute
```

## Entrypoint

```text
/usr/bin/tini -- /opt/tmr-task2/docker/entrypoint.sh
```

For `execute`, the entrypoint:

1. sources the image's Jazzy and Franka interface overlays;
2. transfers `guarded_lateral_step.py`, `stage0_wall_docking_base.py`, and
   `base_runtime/` from the image to `.50:/tmp/tmr-task2-policy`;
3. starts the bundled three-camera bridge;
4. verifies the deployed ROS graph and base-local runtime; and
5. launches:

```text
/usr/bin/python3 -u /opt/tmr-task2/policy/quick_start.py --execute
```

The base preflight starts or reuses the staged, zero-latching Task 2 velocity
adapter only after fresh base odometry is observed. It never starts a second
base controller. The writable `/tmp/tmr_task2_motion.lock` is created inside
the container. No `--privileged`, device mount, host project bind mount,
`tmr_env.sh`, or `/run/screen` mount is used.

## Environment and dependencies

- Image base: digest-pinned `ros:jazzy-ros-base-noble` (Ubuntu 24.04,
  ROS 2 Jazzy).
- Container DDS: `rmw_cyclonedds_cpp`, domain 0, host networking, bundled
  CycloneDDS configuration.
- Franka interfaces: `franka_ros2` v3.4.1 at upstream commit
  `b4164f555500fe50c3f44f24d4cccc452ffac442`, built in the image.
- Base host `.50`: ROS 2 Humble, `/opt/ros/humble/setup.bash`, the deployed
  base controller/workspace, domain 97, and localhost-only discovery. The
  staged base processes use that native environment.
- The existing non-interactive `tmr-user@172.16.0.50` login must be loaded in
  the operator's SSH agent. The run command forwards only that agent socket;
  the policy never requests, stores, copies, or mounts private-key files.
- GPU/CUDA: not used; no GPU, CUDA toolkit, or NVIDIA driver is required.
- Runtime Internet access: not required.
- Runtime testbed LAN access: required for Jazzy DDS on `.100`, the base
  login/control handoff to `.50`, the ZED JPEG endpoint on
  `172.16.0.50:18082`, and the deployed Spine/robot services.
- Build-time Internet access is required to clone the repository, pull the
  base image, and install image packages.

## Hardware assumptions

- Two deployed Franka FR3 arms, Robotiq grippers, Franka Spine, ZED-M head
  camera, and left/right Intel RealSense D405 wrist cameras are started by the
  normal testbed procedure.
- Main ZED frames come from
  `http://172.16.0.50:18082/tmr_zed_latest.jpg`. Wrist frames remain ROS
  topics `/wrist_camera_left/color/image_raw` and
  `/wrist_camera_right/color/image_raw` in the Jazzy graph.
- The base-local graph provides fresh `/swerve_drive_controller/odom`; the
  staged adapter owns `/tmr_cycle/mission_cmd_vel` and forwards accepted,
  fresh mission commands to `/swerve_drive_controller/cmd_vel`.
- No GPU inference rate applies. The base motion loop is approximately 40 Hz;
  the staged adapter watchdog is 20 Hz. Camera health requires advancing main
  and left-wrist frames.
- Camera placement, hand-eye transform, arm initial pose, object layout,
  lighting, and calibrated distances must match the Task 2 setup and
  `policy/config/`.
- Between rounds, restore both arms, Spine, base, grippers, thermal pad, and
  red placement pad to their documented initial state and clear all swept
  volumes.
