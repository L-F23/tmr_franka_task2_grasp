# syntax=docker/dockerfile:1.7

ARG ROS_IMAGE=ros:jazzy-ros-base-noble@sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4
FROM ${ROS_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG POLICY_USER=aup
ARG POLICY_UID=1000
ARG POLICY_GID=1000
ARG POLICY_REVISION=unknown

LABEL org.opencontainers.image.source="https://github.com/L-F23/tmr_franka_task2_grasp" \
      org.opencontainers.image.revision="${POLICY_REVISION}" \
      org.opencontainers.image.title="TMR Franka Task 2 policy"

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        openssh-client \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-opencv \
        ros-jazzy-cv-bridge \
        ros-jazzy-control-msgs \
        ros-jazzy-controller-manager-msgs \
        ros-jazzy-moveit-msgs \
        ros-jazzy-nav-msgs \
        ros-jazzy-realsense2-camera-msgs \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-rosidl-default-generators \
        tar \
        tini \
    && rm -rf /var/lib/apt/lists/*

RUN existing_group="$(getent group "${POLICY_GID}" | cut -d: -f1 || true)" \
    && if [ -z "${existing_group}" ]; then \
         groupadd --gid "${POLICY_GID}" "${POLICY_USER}"; \
       elif [ "${existing_group}" != "${POLICY_USER}" ]; then \
         groupmod --new-name "${POLICY_USER}" "${existing_group}"; \
       fi \
    && existing_user="$(getent passwd "${POLICY_UID}" | cut -d: -f1 || true)" \
    && if [ -z "${existing_user}" ]; then \
         useradd --uid "${POLICY_UID}" --gid "${POLICY_GID}" \
           --create-home --shell /bin/bash "${POLICY_USER}"; \
       elif [ "${existing_user}" != "${POLICY_USER}" ]; then \
         usermod --login "${POLICY_USER}" "${existing_user}"; \
         usermod --home "/home/${POLICY_USER}" --move-home \
           --gid "${POLICY_GID}" --shell /bin/bash "${POLICY_USER}"; \
       fi

ENV DEBIAN_FRONTEND= \
    HOME=/home/aup \
    POLICY_ROOT=/opt/tmr-task2/policy \
    PYTHONDONTWRITEBYTECODE=1 \
    ROS_DOMAIN_ID=0 \
    ROS_LOCALHOST_ONLY=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///opt/tmr-task2/docker/cyclonedds.xml \
    ROS_HOME=/tmp/tmr-ros \
    ROS_LOG_DIR=/tmp/tmr-ros/log \
    XDG_CACHE_HOME=/tmp/tmr-cache \
    PYTHONUNBUFFERED=1 \
    TMR_TASK2_POLICY_REVISION=${POLICY_REVISION}

COPY --chown=${POLICY_UID}:${POLICY_GID} . /opt/tmr-task2

# Build the exact Franka interface definitions used by the policy.  The
# Apache-2.0 sources and their pinned upstream revision live in third_party/.
RUN mkdir -p /opt/tmr-interfaces/src \
    && cp -a /opt/tmr-task2/third_party/franka_ros2_interfaces/franka_msgs \
        /opt/tmr-interfaces/src/ \
    && cp -a /opt/tmr-task2/third_party/franka_ros2_interfaces/franka_spine_msgs \
        /opt/tmr-interfaces/src/ \
    && . /opt/ros/jazzy/setup.sh \
    && cd /opt/tmr-interfaces \
    && colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF \
    && rm -rf build log \
    && chown -R "${POLICY_UID}:${POLICY_GID}" /opt/tmr-interfaces

RUN chmod 0755 /opt/tmr-task2/docker/entrypoint.sh \
        /opt/tmr-task2/policy/base_runtime/ensure_runtime.sh \
    && bash -n /opt/tmr-task2/docker/entrypoint.sh \
        /opt/tmr-task2/policy/base_runtime/ensure_runtime.sh \
    && mkdir -p /opt/tmr-task2/policy/outputs /opt/tmr-task2/policy/runtime \
    && chown -R "${POLICY_UID}:${POLICY_GID}" \
        /opt/tmr-task2/policy/outputs /opt/tmr-task2/policy/runtime \
    && /usr/bin/python3 -m compileall -q /opt/tmr-task2 \
    && /usr/bin/python3 -c "import cv2, numpy"

WORKDIR /opt/tmr-task2/policy
USER ${POLICY_UID}:${POLICY_GID}

ENTRYPOINT ["/usr/bin/tini", "--", "/opt/tmr-task2/docker/entrypoint.sh"]
CMD ["check"]
