# Docker deployment

The image packages the Task 2 Python policy, OpenCV/NumPy, ROS 2 Jazzy command
line tools and standard messages, CycloneDDS, SSH, curl, and screen. Runtime
code, configuration, and calibration captures are grouped under `policy/`.

The deployed testbed's custom `franka_msgs`/`franka_spine_msgs` overlays and
DDS configuration remain the source of truth. At runtime, `/home/aup` is
mounted read-only at the same absolute path so `tmr_env.sh` and its overlay
prefixes work without copying hardware-specific build artifacts into the
image. Policy records, output images, and runtime state are mounted separately
as writable directories.

The container is fail-closed. Its default command is `check`, which performs
read-only ROS graph and camera checks. Physical motion starts only through the
explicit `execute` mode.

## Operator commands

Run these commands on robot host `.100` as user `aup`. They start from a clean
checkout of the pinned policy commit.

```bash
set -euo pipefail

export POLICY_REPOSITORY='https://github.com/L-F23/tmr_franka_task2_grasp.git'
export POLICY_COMMIT='010efaec68a8d5310a7516c7163fbca967971626'
export POLICY_IMAGE="tmr-task2-policy:${POLICY_COMMIT}"
export POLICY_CHECKOUT="$HOME/tmr-task2-policy-${POLICY_COMMIT}"

test -r /home/aup/tmr_env.sh
test -d /home/aup/tmr-mobile-manipulation
test -d /home/aup/.ssh
test -d /run/screen
test ! -e "$POLICY_CHECKOUT"
ssh -o BatchMode=yes -o ConnectTimeout=5 tmr-user@172.16.0.50 true

git clone "$POLICY_REPOSITORY" "$POLICY_CHECKOUT"
cd "$POLICY_CHECKOUT"
git checkout --detach "$POLICY_COMMIT"
test "$(git rev-parse HEAD)" = "$POLICY_COMMIT"
test -z "$(git status --porcelain)"

mkdir -p policy/outputs policy/runtime
touch /tmp/tmr_task2_motion.lock

docker build --pull \
  --build-arg POLICY_UID="$(id -u)" \
  --build-arg POLICY_GID="$(id -g)" \
  --build-arg POLICY_REVISION="$POLICY_COMMIT" \
  --tag "$POLICY_IMAGE" \
  .
```

Validate the image and the mounted custom ROS environment without contacting
the robot graph or sending motion:

```bash
docker run --rm \
  --network host \
  --ipc host \
  --mount type=bind,src=/home/aup,dst=/home/aup,readonly \
  --mount type=bind,src=/run/screen,dst=/run/screen \
  --mount type=bind,src=/tmp/tmr_task2_motion.lock,dst=/tmp/tmr_task2_motion.lock \
  --mount type=bind,src="$POLICY_CHECKOUT/policy/config",dst=/opt/tmr-task2/policy/config \
  --mount type=bind,src="$POLICY_CHECKOUT/policy/outputs",dst=/opt/tmr-task2/policy/outputs \
  --mount type=bind,src="$POLICY_CHECKOUT/policy/runtime",dst=/opt/tmr-task2/policy/runtime \
  "$POLICY_IMAGE" preflight
```

Run the read-only ROS graph and camera health check:

```bash
docker run --rm \
  --network host \
  --ipc host \
  --mount type=bind,src=/home/aup,dst=/home/aup,readonly \
  --mount type=bind,src=/run/screen,dst=/run/screen \
  --mount type=bind,src=/tmp/tmr_task2_motion.lock,dst=/tmp/tmr_task2_motion.lock \
  --mount type=bind,src="$POLICY_CHECKOUT/policy/config",dst=/opt/tmr-task2/policy/config \
  --mount type=bind,src="$POLICY_CHECKOUT/policy/outputs",dst=/opt/tmr-task2/policy/outputs \
  --mount type=bind,src="$POLICY_CHECKOUT/policy/runtime",dst=/opt/tmr-task2/policy/runtime \
  "$POLICY_IMAGE" check
```

After the operator has confirmed the required initial pose, gripper state,
clear workspace, and reachable emergency stop, launch the policy with:

```bash
docker run --rm --interactive --tty \
  --name tmr-task2-policy \
  --stop-timeout 10 \
  --network host \
  --ipc host \
  --mount type=bind,src=/home/aup,dst=/home/aup,readonly \
  --mount type=bind,src=/run/screen,dst=/run/screen \
  --mount type=bind,src=/tmp/tmr_task2_motion.lock,dst=/tmp/tmr_task2_motion.lock \
  --mount type=bind,src="$POLICY_CHECKOUT/policy/config",dst=/opt/tmr-task2/policy/config \
  --mount type=bind,src="$POLICY_CHECKOUT/policy/outputs",dst=/opt/tmr-task2/policy/outputs \
  --mount type=bind,src="$POLICY_CHECKOUT/policy/runtime",dst=/opt/tmr-task2/policy/runtime \
  "$POLICY_IMAGE" execute
```

The image entrypoint is:

```text
/usr/bin/tini -- /opt/tmr-task2/docker/entrypoint.sh
```

`docker/entrypoint.sh` changes to `/opt/tmr-task2/policy`, sources
`/opt/ros/jazzy/setup.bash` and the mounted `/home/aup/tmr_env.sh`, validates
every required custom message import, and then launches
`/usr/bin/python3 -u quick_start.py --execute` for `execute` mode.

No `--privileged` flag or device mount is required. The real-time FCI and base
controller processes remain on their respective robot hosts; this container is
the policy/orchestration client.

The bind-mounted `/tmp/tmr_task2_motion.lock` preserves the repository's
single-instance `flock` contract across native host processes and policy
containers. The `/run/screen` mount lets the policy reuse or replace the
existing host camera-viewer session instead of starting an untracked duplicate.
ROS logs and caches are redirected to container-local `/tmp` paths so the
testbed home directory can remain read-only.
