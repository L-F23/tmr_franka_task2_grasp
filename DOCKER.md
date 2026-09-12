# Docker deployment

This submission targets the current two-host evaluation setup: the head ZED
service runs on `172.16.16.50`, while the wrist cameras and robot services run
on `172.16.16.100`. Both are reached through native ROS 2 Humble graphs. The
container is also ROS 2 Humble. It does not
join the graph with a Jazzy participant, use an explicit CycloneDDS peer list,
or contact the obsolete `172.16.0.50`/`172.16.0.100` addresses.

The image contains the Task 2 policy, OpenCV/NumPy, the required Franka
interfaces, an isolated ROS 2 DDS domain bridge for the ZED compressed-image
topic, the policy camera viewer, and the fail-closed base velocity adapter. It
does not require SSH, `tmr_env.sh`, another repository, or a host workspace
mount. The default mode is the non-motion `check`; physical motion requires
the explicit `execute` mode.

## Operator commands

Run on the evaluation computer that runs Docker and can reach the testbed ROS
graph. Set `POLICY_COMMIT` to the full 40-character SHA submitted for
evaluation.

```bash
set -euo pipefail

export POLICY_REPOSITORY='https://github.com/L-F23/tmr_franka_task2_grasp.git'
export POLICY_COMMIT='<FULL_40_CHARACTER_PINNED_COMMIT_SHA>'
export POLICY_IMAGE="tmr-task2-policy:${POLICY_COMMIT}"
export POLICY_CHECKOUT="$PWD/tmr-task2-policy-${POLICY_COMMIT}"
export ROBOT_DOMAIN_ID=0
export ZED_DOMAIN_ID=1
export ZED_BRIDGE_NAME="tmr-task2-zed-${POLICY_COMMIT}"

test ! -e "$POLICY_CHECKOUT"
git clone "$POLICY_REPOSITORY" "$POLICY_CHECKOUT"
cd "$POLICY_CHECKOUT"
git checkout --detach "$POLICY_COMMIT"
test "$(git rev-parse HEAD)" = "$POLICY_COMMIT"
test -z "$(git status --porcelain)"

docker build --pull --no-cache \
  --build-arg POLICY_UID="$(id -u)" \
  --build-arg POLICY_GID="$(id -g)" \
  --build-arg POLICY_REVISION="$POLICY_COMMIT" \
  --tag "$POLICY_IMAGE" \
  "$POLICY_CHECKOUT"

# Fail immediately if an old image or the wrong build context was used. The
# revision label alone is insufficient because it is supplied as a build arg.
test "$(docker image inspect \
  --format='{{ index .Config.Labels "org.opencontainers.image.revision" }}' \
  "$POLICY_IMAGE")" = "$POLICY_COMMIT"
docker run --rm "$POLICY_IMAGE" help 2>&1 | grep -q 'zed-bridge'

docker volume create "tmr-task2-config-${POLICY_COMMIT}" >/dev/null
docker volume create "tmr-task2-outputs-${POLICY_COMMIT}" >/dev/null
docker volume create "tmr-task2-runtime-${POLICY_COMMIT}" >/dev/null
```

Validate the image without joining the robot graph or sending motion:

```bash
docker run --rm "$POLICY_IMAGE" preflight
```

After the normal Humble testbed services are running, first prove that the
evaluation computer can receive one ZED frame from `.50` in the vision domain:

```bash
docker run --rm \
  --network host \
  --env ROS_DOMAIN_ID="$ZED_DOMAIN_ID" \
  --env ROS_LOCALHOST_ONLY=0 \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  "$POLICY_IMAGE" shell -lc \
  'timeout 30 ros2 topic echo \
    /head_camera/zed/rgb/color/rect/image/compressed \
    sensor_msgs/msg/CompressedImage --once >/dev/null'
```

This probe requires the ZED publisher on `.50` to use
`ROS_LOCALHOST_ONLY=0`. If it fails, do not run the policy: verify the source
domain, topic name, DDS implementation, `.50` firewall, and ZED publisher
first.

After the source probe succeeds, launch the bundled read-only DDS domain
bridge. The reference testbed runs the high-bandwidth ZED stream in domain 1;
set
`ZED_DOMAIN_ID` to the actual ZED publisher domain if the testbed configuration
differs. The bridge forwards only one `sensor_msgs/msg/CompressedImage` topic
and never publishes a motion command:

```bash
docker run --detach --rm \
  --name "$ZED_BRIDGE_NAME" \
  --network host \
  --env ROS_LOCALHOST_ONLY=0 \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  --env TMR_ZED_DOMAIN_ID="$ZED_DOMAIN_ID" \
  --env TMR_ROBOT_DOMAIN_ID="$ROBOT_DOMAIN_ID" \
  "$POLICY_IMAGE" zed-bridge

docker run --rm \
  --network host \
  --env ROS_DOMAIN_ID="$ROBOT_DOMAIN_ID" \
  --env ROS_LOCALHOST_ONLY=0 \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  "$POLICY_IMAGE" shell -lc \
  'timeout 30 ros2 topic echo \
    /tmr_task2/zed/image/compressed \
    sensor_msgs/msg/CompressedImage --once >/dev/null'
```

The successful one-frame probe proves that the bridge can see the source ZED
publisher and republish it in the robot-control domain. Now perform the
non-motion graph, odometry, controller, and camera check:

```bash
docker run --rm \
  --network host \
  --env ROS_DOMAIN_ID="$ROBOT_DOMAIN_ID" \
  --env ROS_LOCALHOST_ONLY=0 \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  --env TMR_MAIN_CAMERA_TOPIC=/tmr_task2/zed/image/compressed \
  --env TMR_LEFT_CAMERA_TOPIC=/wrist_camera_left/camera/color/image_rect_raw \
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
  --env ROS_DOMAIN_ID="$ROBOT_DOMAIN_ID" \
  --env ROS_LOCALHOST_ONLY=0 \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  --env TMR_MAIN_CAMERA_TOPIC=/tmr_task2/zed/image/compressed \
  --env TMR_LEFT_CAMERA_TOPIC=/wrist_camera_left/camera/color/image_rect_raw \
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

The separate, non-motion ZED bridge entrypoint is:

```text
ros2 run domain_bridge domain_bridge --from ${TMR_ZED_DOMAIN_ID:-1} --to ${TMR_ROBOT_DOMAIN_ID:-0} /opt/tmr-task2/docker/zed_domain_bridge.yaml
```

The read-only check expects these MoveIt services:

```text
/compute_fk
/compute_ik
/check_state_validity
/plan_kinematic_path
```

The `zed-bridge` container subscribes to the evaluator's existing ZED stream,
`/head_camera/zed/rgb/color/rect/image/compressed`, in the ZED domain and
republishes only that stream as `/tmr_task2/zed/image/compressed` in the
robot-control domain. The policy subscribes directly to that ROS topic and to
the left wrist color topic:
`/wrist_camera_left/camera/color/image_rect_raw`. Avoiding the unused right
wrist stream reduces DDS and image-copy load during evaluation.

## Environment and dependencies

- Image base: digest-pinned `ros:humble-ros-base-jammy` (Ubuntu 22.04,
  ROS 2 Humble).
- DDS: `rmw_fastrtps_cpp` with `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, no custom
  peer list or DDS XML. UDP-only transport prevents Fast DDS from selecting
  shared memory between containers that share the host network but have
  separate IPC namespaces. The policy container
  uses robot-control domain 0. The independent `domain_bridge` participant
  receives the ZED topic from vision domain 1 and republishes only that topic
  into domain 0. Both containers and deployed graphs use Humble.
- Franka interfaces: `franka_ros2` v3.4.1 at upstream commit
  `b4164f555500fe50c3f44f24d4cccc452ffac442`, built in the image.
- Host: Linux with Docker Engine and host networking. Host IPC is deliberately
  not shared; DDS communication with `.50` and `.100` uses the testbed network.
- Existing services: the deployed Humble arm, gripper, Spine, MoveIt, wrist
  camera, ZED ROS publisher, odometry, and swerve controller services must
  already be running. The submitted image supplies the DDS topic bridge; no
  HTTP/JPEG exporter or unpublished service is required. It does not start
  duplicate hardware drivers.
- GPU/CUDA/ROCm: not used. No GPU or GPU runtime is required.
- Runtime Internet access: not required.
- Runtime testbed network access: required for the native Humble ROS graph.
  Blocking Internet access is supported; blocking the testbed network is not.
- Build-time Internet access is required to clone the repository, pull the
  base image, and install image packages.

## Hardware assumptions

- The ZED-M head camera publisher runs on `.50`. The two Franka FR3 arms,
  Robotiq grippers, Franka Spine, left/right RealSense D405 cameras, and their
  robot interfaces run on `.100`; the swerve-drive base uses the deployed
  testbed service. All are started by the standard testbed procedure.
- The base loop runs at approximately 40 Hz and its fail-closed watchdog at
  20 Hz. The compressed ZED stream is forwarded at its publisher rate with a
  best-effort, depth-1 sensor QoS. Arm real-time control remains in the
  deployed robot drivers.
- Camera mounting, hand-eye calibration, arm poses, object layout, and
  lighting must match the demonstrated Task 2 setup and `policy/config/`.
- Between rounds, restore both arms, grippers, Spine, base pose, thermal pad,
  black grasp base, and red placement pad; remove previous objects and clear
  all swept volumes.

After evaluation, stop the read-only bridge if it is still running:

```bash
docker stop "$ZED_BRIDGE_NAME"
```
