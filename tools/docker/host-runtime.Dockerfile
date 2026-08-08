ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS2_CONTROL_VERSION=2.54.0-1jammy.20260804.213423
ARG ROS2_CONTROLLERS_VERSION=2.53.3-1jammy.20260804.212633
ARG MECANUM_CONTROLLER_VERSION=2.53.3-1jammy.20260804.211244
ARG ACKERMANN_CONTROLLER_VERSION=2.53.3-1jammy.20260804.211824
ARG XACRO_VERSION=2.1.1-1jammy.20260304.195513
ARG FOXGLOVE_BRIDGE_VERSION=3.4.3-2jammy.20260726.140144

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       "ros-humble-ros2-control=${ROS2_CONTROL_VERSION}" \
       "ros-humble-ros2-controllers=${ROS2_CONTROLLERS_VERSION}" \
       "ros-humble-mecanum-drive-controller=${MECANUM_CONTROLLER_VERSION}" \
       "ros-humble-ackermann-steering-controller=${ACKERMANN_CONTROLLER_VERSION}" \
       "ros-humble-foxglove-bridge=${FOXGLOVE_BRIDGE_VERSION}" \
       "ros-humble-xacro=${XACRO_VERSION}" \
    && test "$(dpkg-query -W -f='${Version}' ros-humble-ros2-control)" = \
       "${ROS2_CONTROL_VERSION}" \
    && test "$(dpkg-query -W -f='${Version}' ros-humble-ros2-controllers)" = \
       "${ROS2_CONTROLLERS_VERSION}" \
    && test "$(dpkg-query -W -f='${Version}' ros-humble-foxglove-bridge)" = \
       "${FOXGLOVE_BRIDGE_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
ENTRYPOINT []
CMD ["/bin/bash"]
