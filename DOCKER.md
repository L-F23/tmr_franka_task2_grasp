# Docker deployment

This submission targets the current `172.16.16.50` evaluation computer and
its native ROS 2 Humble graph. The container is also ROS 2 Humble. It does not
join the graph with a Jazzy participant, use an explicit CycloneDDS peer list,
or contact the obsolete `172.16.0.50`/`172.16.0.100` addresses.

The image contains the Task 2 policy, OpenCV/NumPy, the required Franka
interfaces, the camera bridge, and the fail-closed base velocity adapter. It
does not require SSH, `tmr_env.sh`, another repository, or a host workspace
mount. The default mode is the non-motion `check`; physical motion requires
the explicit `execute` mode.

## Operator commands

Run on the `172.16.16.50` evaluation computer. Set `POLICY_COMMIT` to the full
40-character SHA submitted for evaluation.

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
```

Validate the image without joining the robot graph or sending motion:

```bash
docker run --rm "$POLICY_IMAGE" preflight
```

After the normal Humble testbed services are running, perform the non-motion
graph, odometry, controller, and camera check:

```bash
docker run --rm \
  --network host \
  --env ROS_DOMAIN_ID=0 \
  --env ROS_LOCALHOST_ONLY=0 \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --env TMR_MAIN_CAMERA_TOPIC=/head_camera/zed/rgb/color/rect/image/compressed \
  --env TMR_LEFT_CAMERA_TOPIC=/wrist_camera_left/color/image_raw \
  --mount type=volume,src="tmr-task2-config-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/config \
  --mount type=volume,src="tmr-task2-outputs-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/outputs \
  --mount type=volume,src="tmr-task2-runtime-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/runtime \
  "$POLICY_IMAGE" check
```

Only after that command succeeds, confirm the designated initial state, clear
all swept volumes, and verify that the emergency stop is reachable. Then
launch the physical policy:

```bash
docker run --rm --interactive --tty \
  --name tmr-task2-policy \
  --stop-timeout 10 \
  --network host \
  --env ROS_DOMAIN_ID=0 \
  --env ROS_LOCALHOST_ONLY=0 \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --env TMR_MAIN_CAMERA_TOPIC=/head_camera/zed/rgb/color/rect/image/compressed \
  --env TMR_LEFT_CAMERA_TOPIC=/wrist_camera_left/color/image_raw \
  --mount type=volume,src="tmr-task2-config-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/config \
  --mount type=volume,src="tmr-task2-outputs-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/outputs \
  --mount type=volume,src="tmr-task2-runtime-${POLICY_COMMIT}",dst=/opt/tmr-task2/policy/runtime \
  "$POLICY_IMAGE" execute
```

## Entrypoint

```text
/usr/bin/tini -- /opt/tmr-task2/docker/entrypoint.sh
```

In `execute` mode it launches:

```text
/usr/bin/python3 -u /opt/tmr-task2/policy/quick_start.py --execute
```

The read-only check expects these MoveIt services:

```text
/compute_fk
/compute_ik
/check_state_validity
/plan_kinematic_path
```

It subscribes directly to the evaluator's existing ZED stream:
`/head_camera/zed/rgb/color/rect/image/compressed`. The optional
`TMR_MAIN_CAMERA_URL` JPEG input remains as a fallback and does not need to be
available when the ROS ZED stream is fresh. It also subscribes only to the left
wrist color topic needed by the policy:
`/wrist_camera_left/color/image_raw`. Avoiding the unused right
wrist stream reduces DDS and image-copy load during evaluation.

## Environment and dependencies

- Image base: digest-pinned `ros:humble-ros-base-jammy` (Ubuntu 22.04,
  ROS 2 Humble).
- DDS: `rmw_fastrtps_cpp`, domain 0, no custom peer list or DDS XML. Both the
  container and deployed graph use Humble.
- Franka interfaces: `franka_ros2` v3.4.1 at upstream commit
  `b4164f555500fe50c3f44f24d4cccc452ffac442`, built in the image.
- Host: Linux with Docker Engine and host networking. Host IPC is deliberately
  not shared; DDS communication with host processes uses the host network.
- Existing services: the deployed Humble arm, gripper, Spine, MoveIt, wrist
  camera, ZED ROS publisher, odometry, and swerve controller services must
  already be running. A separate team-specific ZED HTTP exporter is not
  required. The policy does not start duplicate hardware drivers.
- GPU/CUDA/ROCm: not used. No GPU or GPU runtime is required.
- Runtime Internet access: not required.
- Runtime testbed network access: required for the native Humble ROS graph.
  Blocking Internet access is supported; blocking the testbed network is not.
- Build-time Internet access is required to clone the repository, pull the
  base image, and install image packages.

## Hardware assumptions

- Two Franka FR3 arms, Robotiq grippers, Franka Spine, ZED-M head camera,
  left/right RealSense D405 cameras, and the swerve-drive base are started by
  the standard testbed procedure.
- The base loop runs at approximately 40 Hz, its fail-closed watchdog at
  20 Hz, and the ZED JPEG poller at approximately 10 Hz. Arm real-time control
  remains in the deployed robot drivers.
- Camera mounting, hand-eye calibration, arm poses, object layout, and
  lighting must match the demonstrated Task 2 setup and `policy/config/`.
- Between rounds, restore both arms, grippers, Spine, base pose, thermal pad,
  black grasp base, and red placement pad; remove previous objects and clear
  all swept volumes.
