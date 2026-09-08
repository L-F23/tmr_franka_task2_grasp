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
        python3-numpy \
        python3-opencv \
        ros-jazzy-control-msgs \
        ros-jazzy-controller-manager-msgs \
        ros-jazzy-moveit-msgs \
        ros-jazzy-realsense2-camera-msgs \
        ros-jazzy-rmw-cyclonedds-cpp \
        screen \
        tini \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid "${POLICY_GID}" "${POLICY_USER}" \
    && useradd --uid "${POLICY_UID}" --gid "${POLICY_GID}" \
        --create-home --shell /bin/bash "${POLICY_USER}"

ENV DEBIAN_FRONTEND= \
    HOME=/home/aup \
    POLICY_ROOT=/opt/tmr-task2 \
    PYTHONDONTWRITEBYTECODE=1 \
    ROS_ENV_FILE=/home/aup/tmr_env.sh \
    ROS_HOME=/tmp/tmr-ros \
    ROS_LOG_DIR=/tmp/tmr-ros/log \
    XDG_CACHE_HOME=/tmp/tmr-cache \
    PYTHONUNBUFFERED=1 \
    TMR_TASK2_POLICY_REVISION=${POLICY_REVISION}

WORKDIR /opt/tmr-task2
COPY --chown=${POLICY_UID}:${POLICY_GID} . /opt/tmr-task2

RUN chmod 0755 /opt/tmr-task2/docker/entrypoint.sh \
    && mkdir -p /opt/tmr-task2/outputs /opt/tmr-task2/runtime \
    && chown -R "${POLICY_UID}:${POLICY_GID}" \
        /opt/tmr-task2/outputs /opt/tmr-task2/runtime \
    && /usr/bin/python3 -m compileall -q /opt/tmr-task2 \
    && /usr/bin/python3 -c "import cv2, numpy"

USER ${POLICY_UID}:${POLICY_GID}

ENTRYPOINT ["/usr/bin/tini", "--", "/opt/tmr-task2/docker/entrypoint.sh"]
CMD ["check"]
