ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH
ARG ROS_SNAPSHOT_DATE
ARG ROS2_CONTROL_VERSION
ARG ROS2_CONTROLLERS_VERSION
ARG MECANUM_CONTROLLER_VERSION
ARG ACKERMANN_CONTROLLER_VERSION
ARG FOXGLOVE_BRIDGE_VERSION
ARG XACRO_VERSION
ARG OH_MY_ZSH_COMMIT=97b27bb2ec0701330b18c2d3e340b22e742b3fa8

RUN test -n "${ROS_SNAPSHOT_DATE}" \
    && test -n "${ROS2_CONTROL_VERSION}" \
    && test -n "${ROS2_CONTROLLERS_VERSION}" \
    && test -n "${MECANUM_CONTROLLER_VERSION}" \
    && test -n "${ACKERMANN_CONTROLLER_VERSION}" \
    && test -n "${FOXGLOVE_BRIDGE_VERSION}" \
    && test -n "${XACRO_VERSION}" \
    && sed -i \
       "s#http://packages.ros.org/ros2/ubuntu#http://snapshots.ros.org/humble/${ROS_SNAPSHOT_DATE}/ubuntu#g" \
       /etc/apt/sources.list.d/ros2.sources \
    && grep -Fq \
       "http://snapshots.ros.org/humble/${ROS_SNAPSHOT_DATE}/ubuntu" \
       /etc/apt/sources.list.d/ros2.sources \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       "ros-humble-ros2-control=${ROS2_CONTROL_VERSION}" \
       "ros-humble-ros2-controllers=${ROS2_CONTROLLERS_VERSION}" \
       "ros-humble-mecanum-drive-controller=${MECANUM_CONTROLLER_VERSION}" \
       "ros-humble-ackermann-steering-controller=${ACKERMANN_CONTROLLER_VERSION}" \
       "ros-humble-foxglove-bridge=${FOXGLOVE_BRIDGE_VERSION}" \
       "ros-humble-xacro=${XACRO_VERSION}" \
       build-essential=12.9ubuntu3 \
       ca-certificates=20240203~22.04.1 \
       cmake=3.22.1-1ubuntu1.22.04.2 \
       curl=7.81.0-1ubuntu1.20 \
       g++=4:11.2.0-1ubuntu1 \
       git=1:2.34.1-1ubuntu1.17 \
       libasan6=11.4.0-1ubuntu1~22.04.2 \
       libasio-dev=1:1.18.1-1 \
       libeigen3-dev=3.4.0-2ubuntu2 \
       libfmt-dev=8.1.1+ds1-2 \
       libgtest-dev=1.11.0-3 \
       libssl-dev=3.0.2-0ubuntu1.19 \
       libtinyxml2-dev=9.0.0+dfsg-3 \
       libubsan1=12.3.0-1ubuntu1~22.04.2 \
       libyaml-cpp-dev=0.7.0+dfsg-8build1 \
       ninja-build=1.10.1-1 \
       patch=2.7.6-7build2 \
       psmisc=23.4-2build3 \
       python3-pytest=6.2.5-1ubuntu2 \
       python3-yaml=5.4.1-1ubuntu1 \
       udev=249.11-0ubuntu3.21 \
       xz-utils=5.2.5-2ubuntu1 \
       zsh=5.8.1-1 \
       zsh-autosuggestions=0.7.0-1 \
       zsh-syntax-highlighting=0.7.1-2 \
    && test "$(dpkg-query -W -f='${Version}' ros-humble-ros2-control)" = \
       "${ROS2_CONTROL_VERSION}" \
    && test "$(dpkg-query -W -f='${Version}' ros-humble-ros2-controllers)" = \
       "${ROS2_CONTROLLERS_VERSION}" \
    && test "$(dpkg-query -W -f='${Version}' ros-humble-mecanum-drive-controller)" = \
       "${MECANUM_CONTROLLER_VERSION}" \
    && test "$(dpkg-query -W -f='${Version}' ros-humble-ackermann-steering-controller)" = \
       "${ACKERMANN_CONTROLLER_VERSION}" \
    && test "$(dpkg-query -W -f='${Version}' ros-humble-foxglove-bridge)" = \
       "${FOXGLOVE_BRIDGE_VERSION}" \
    && test "$(dpkg-query -W -f='${Version}' ros-humble-xacro)" = \
       "${XACRO_VERSION}" \
    && test "$(dpkg-query -W -f='${Version}' build-essential)" = \
       "12.9ubuntu3" \
    && test "$(dpkg-query -W -f='${Version}' libeigen3-dev)" = \
       "3.4.0-2ubuntu2" \
    && test "$(dpkg-query -W -f='${Version}' libfmt-dev)" = \
       "8.1.1+ds1-2" \
    && test "$(dpkg-query -W -f='${Version}' libasan6)" = \
       "11.4.0-1ubuntu1~22.04.2" \
    && test "$(dpkg-query -W -f='${Version}' libubsan1)" = \
       "12.3.0-1ubuntu1~22.04.2" \
    && rm -rf /var/lib/apt/lists/*

RUN case "${TARGETARCH}" in \
      amd64) \
        toolchain_host=x86_64; \
        toolchain_sha256=6cd1bbc1d9ae57312bcd169ae283153a9572bd6a8e4eeae2fedfbc33b115fdbb \
        ;; \
      arm64) \
        toolchain_host=aarch64; \
        toolchain_sha256=8fd8b4a0a8d44ab2e195ccfbeef42223dfb3ede29d80f14dcf2183c34b8d199a \
        ;; \
      *) echo "Unsupported build architecture: ${TARGETARCH}" >&2; exit 1 ;; \
    esac \
    && toolchain_archive="arm-gnu-toolchain-13.2.rel1-${toolchain_host}-arm-none-eabi.tar.xz" \
    && curl -fL --retry 5 --retry-all-errors --retry-delay 2 \
       --continue-at - --connect-timeout 30 --max-time 1800 \
       "https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu/13.2.rel1/binrel/${toolchain_archive}" \
       -o "/tmp/${toolchain_archive}" \
    && echo "${toolchain_sha256}  /tmp/${toolchain_archive}" \
       | sha256sum --check --strict - \
    && mkdir -p /opt/arm-gnu-toolchain \
    && tar -xJf "/tmp/${toolchain_archive}" \
       -C /opt/arm-gnu-toolchain --strip-components=1 \
    && rm -f "/tmp/${toolchain_archive}" \
    && test "$(/opt/arm-gnu-toolchain/bin/arm-none-eabi-gcc -dumpfullversion)" = "13.2.1" \
    && test "$(/opt/arm-gnu-toolchain/bin/arm-none-eabi-g++ -dumpfullversion)" = "13.2.1"

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
    && test -r /usr/share/zsh-syntax-highlighting/zsh-syntax-highlighting.zsh

ENV PATH="/opt/arm-gnu-toolchain/bin:${PATH}" \
    SOURCE_DATE_EPOCH=0 \
    HOME=/tmp/mentor-pi-home \
    ZDOTDIR=/opt/mentor_pi/zsh
WORKDIR /workspace
ENTRYPOINT []
CMD ["/usr/bin/zsh"]
