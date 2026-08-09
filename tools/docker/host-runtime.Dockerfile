ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS2_CONTROL_VERSION=2.54.0-1jammy.20260804.213423
ARG ROS2_CONTROLLERS_VERSION=2.53.3-1jammy.20260804.212633
ARG MECANUM_CONTROLLER_VERSION=2.53.3-1jammy.20260804.211244
ARG ACKERMANN_CONTROLLER_VERSION=2.53.3-1jammy.20260804.211824
ARG XACRO_VERSION=2.1.1-1jammy.20260304.195513
ARG FOXGLOVE_BRIDGE_VERSION=3.4.3-2jammy.20260726.140144
ARG OH_MY_ZSH_COMMIT=97b27bb2ec0701330b18c2d3e340b22e742b3fa8
ARG GIT_VERSION=1:2.34.1-1ubuntu1.17
ARG ZSH_PACKAGE_VERSION=5.8.1-1
ARG ZSH_AUTOSUGGESTIONS_VERSION=0.7.0-1
ARG ZSH_SYNTAX_HIGHLIGHTING_VERSION=0.7.1-2

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       "ros-humble-ros2-control=${ROS2_CONTROL_VERSION}" \
       "ros-humble-ros2-controllers=${ROS2_CONTROLLERS_VERSION}" \
       "ros-humble-mecanum-drive-controller=${MECANUM_CONTROLLER_VERSION}" \
       "ros-humble-ackermann-steering-controller=${ACKERMANN_CONTROLLER_VERSION}" \
       "ros-humble-foxglove-bridge=${FOXGLOVE_BRIDGE_VERSION}" \
       "ros-humble-xacro=${XACRO_VERSION}" \
       "git=${GIT_VERSION}" \
       psmisc \
       udev \
       "zsh=${ZSH_PACKAGE_VERSION}" \
       "zsh-autosuggestions=${ZSH_AUTOSUGGESTIONS_VERSION}" \
       "zsh-syntax-highlighting=${ZSH_SYNTAX_HIGHLIGHTING_VERSION}" \
    && test "$(dpkg-query -W -f='${Version}' ros-humble-ros2-control)" = \
       "${ROS2_CONTROL_VERSION}" \
    && test "$(dpkg-query -W -f='${Version}' ros-humble-ros2-controllers)" = \
       "${ROS2_CONTROLLERS_VERSION}" \
    && test "$(dpkg-query -W -f='${Version}' ros-humble-foxglove-bridge)" = \
       "${FOXGLOVE_BRIDGE_VERSION}" \
    && test "$(dpkg-query -W -f='${Version}' git)" = "${GIT_VERSION}" \
    && test "$(dpkg-query -W -f='${Version}' zsh)" = \
       "${ZSH_PACKAGE_VERSION}" \
    && test "$(dpkg-query -W -f='${Version}' zsh-autosuggestions)" = \
       "${ZSH_AUTOSUGGESTIONS_VERSION}" \
    && test "$(dpkg-query -W -f='${Version}' zsh-syntax-highlighting)" = \
       "${ZSH_SYNTAX_HIGHLIGHTING_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --filter=blob:none --no-checkout \
       https://github.com/ohmyzsh/ohmyzsh.git /opt/mentor_pi/oh-my-zsh \
    && git -C /opt/mentor_pi/oh-my-zsh checkout --detach \
       "${OH_MY_ZSH_COMMIT}" \
    && test "$(git -C /opt/mentor_pi/oh-my-zsh rev-parse HEAD)" = \
       "${OH_MY_ZSH_COMMIT}" \
    && printf '%s\n' "${OH_MY_ZSH_COMMIT}" \
       >/opt/mentor_pi/oh-my-zsh/MENTOR-PI-COMMIT \
    && rm -rf /opt/mentor_pi/oh-my-zsh/.git

COPY tools/docker/host-runtime.zshrc /opt/mentor_pi/zsh/.zshrc

RUN zsh --version \
    && zsh -n /opt/mentor_pi/zsh/.zshrc \
    && test -r /usr/share/zsh-autosuggestions/zsh-autosuggestions.zsh \
    && test -r /usr/share/zsh-syntax-highlighting/zsh-syntax-highlighting.zsh \
    && test "$(cat /opt/mentor_pi/oh-my-zsh/MENTOR-PI-COMMIT)" = \
       "${OH_MY_ZSH_COMMIT}"

WORKDIR /workspace
ENV HOME=/tmp/mentor-pi-home \
    ZDOTDIR=/opt/mentor_pi/zsh
ENTRYPOINT []
CMD ["/usr/bin/zsh"]
