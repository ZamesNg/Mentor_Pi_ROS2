FROM microros/micro_ros_static_library_builder:humble

ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH
ARG OH_MY_ZSH_COMMIT=97b27bb2ec0701330b18c2d3e340b22e742b3fa8
ARG ZSH_AUTOSUGGESTIONS_COMMIT=85919cd1ffa7d2d5412f6d3fe437ebdbeeec4fc5
ARG ZSH_SYNTAX_HIGHLIGHTING_COMMIT=c4d95591843d49838b7ad30081e7aba3135a6703
ARG USERNAME=mentor
ARG USER_UID=1000
ARG USER_GID=1000

RUN test "$(. /etc/os-release && printf '%s:%s' "${ID}" "${VERSION_ID}")" = \
      "ubuntu:22.04" \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      curl \
      git \
      libasio-dev \
      libeigen3-dev \
      libfmt-dev \
      libssl-dev \
      libtinyxml2-dev \
      libyaml-cpp-dev \
      ninja-build \
      patch \
      psmisc \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      python3-yaml \
      ros-humble-ackermann-steering-controller \
      ros-humble-foxglove-bridge \
      ros-humble-mecanum-drive-controller \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-xacro \
      sudo \
      udev \
      xz-utils \
      zsh \
    && rm -rf /var/lib/apt/lists/* \
    && test -f \
      /uros_ws/install/micro_ros_setup/share/micro_ros_setup/package.xml

RUN case "${TARGETARCH}" in \
      amd64) \
        toolchain_host=x86_64; \
        toolchain_sha256=6cd1bbc1d9ae57312bcd169ae283153a9572bd6a8e4eeae2fedfbc33b115fdbb \
        ;; \
      arm64) \
        toolchain_host=aarch64; \
        toolchain_sha256=8fd8b4a0a8d44ab2e195ccfbeef42223dfb3ede29d80f14dcf2183c34b8d199a \
        ;; \
      *) echo "Unsupported development architecture: ${TARGETARCH}" >&2; exit 1 ;; \
    esac \
    && archive="arm-gnu-toolchain-13.2.rel1-${toolchain_host}-arm-none-eabi.tar.xz" \
    && curl -fL --retry 5 --retry-all-errors --connect-timeout 30 \
      --max-time 1800 \
      "https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu/13.2.rel1/binrel/${archive}" \
      -o "/tmp/${archive}" \
    && echo "${toolchain_sha256}  /tmp/${archive}" | sha256sum --check --strict - \
    && mkdir -p /opt/arm-gnu-toolchain \
    && tar -xJf "/tmp/${archive}" -C /opt/arm-gnu-toolchain \
      --strip-components=1 \
    && rm -f "/tmp/${archive}" \
    && test "$(/opt/arm-gnu-toolchain/bin/arm-none-eabi-gcc -dumpfullversion)" = \
      13.2.1

RUN set -eu; \
    groupadd --gid "${USER_GID}" "${USERNAME}"; \
    useradd --create-home --uid "${USER_UID}" --gid "${USER_GID}" \
      --shell /usr/bin/zsh "${USERNAME}"; \
    usermod --append --groups sudo "${USERNAME}"; \
    printf '%s ALL=(root) NOPASSWD:ALL\n' "${USERNAME}" \
      >"/etc/sudoers.d/${USERNAME}"; \
    chmod 0440 "/etc/sudoers.d/${USERNAME}"; \
    install_checkout() { \
      repository="$1"; \
      revision="$2"; \
      destination="$3"; \
      git init "${destination}"; \
      git -C "${destination}" remote add origin "${repository}"; \
      git -C "${destination}" fetch --depth 1 origin "${revision}"; \
      git -C "${destination}" checkout --detach FETCH_HEAD; \
      test "$(git -C "${destination}" rev-parse HEAD)" = "${revision}"; \
    }; \
    user_home="/home/${USERNAME}"; \
    install_checkout \
      https://github.com/ohmyzsh/ohmyzsh.git \
      "${OH_MY_ZSH_COMMIT}" "${user_home}/.oh-my-zsh"; \
    mkdir -p "${user_home}/.oh-my-zsh/custom/plugins"; \
    install_checkout \
      https://github.com/zsh-users/zsh-autosuggestions.git \
      "${ZSH_AUTOSUGGESTIONS_COMMIT}" \
      "${user_home}/.oh-my-zsh/custom/plugins/zsh-autosuggestions"; \
    install_checkout \
      https://github.com/zsh-users/zsh-syntax-highlighting.git \
      "${ZSH_SYNTAX_HIGHLIGHTING_COMMIT}" \
      "${user_home}/.oh-my-zsh/custom/plugins/zsh-syntax-highlighting"; \
    mkdir -p /workspaces/Mentor_Pi; \
    chown -R "${USERNAME}:${USERNAME}" "${user_home}" /workspaces; \
    test -f /etc/ros/rosdep/sources.list.d/20-default.list; \
    sudo -H -u "${USERNAME}" rosdep update; \
    test -s "${user_home}/.ros/rosdep/sources.cache/index"

ENV HOME="/home/${USERNAME}" \
    PATH="/opt/arm-gnu-toolchain/bin:${PATH}" \
    ROS_DISTRO=humble \
    SHELL=/usr/bin/zsh \
    USER="${USERNAME}" \
    SOURCE_DATE_EPOCH=0

RUN printf '%s\n' \
      'mentor_pi_restore_nounset=0' \
      'case "$-" in *u*) mentor_pi_restore_nounset=1; set +u ;; esac' \
      '. /opt/ros/humble/setup.sh' \
      '. /uros_ws/install/setup.sh' \
      'if [ "${mentor_pi_restore_nounset}" -eq 1 ]; then set -u; fi' \
      'unset mentor_pi_restore_nounset' \
      >/etc/profile.d/mentor-pi-ros.sh

COPY --chown=${USERNAME}:${USERNAME} .devcontainer/zshrc \
  /home/${USERNAME}/.zshrc

WORKDIR /workspaces/Mentor_Pi

# The builder image entrypoint only generates a library from /project and is
# not an interactive development entrypoint.  VS Code may replace CMD, but it
# must never inherit that project-specific entrypoint.
ENTRYPOINT []
CMD ["sleep", "infinity"]

# Keep small developer-tool additions after the checksummed Arm toolchain and
# shell setup so changing them does not invalidate those download-heavy layers.
RUN apt-get update \
    && apt-get install -y --no-install-recommends ripgrep \
    && rm -rf /var/lib/apt/lists/*

USER ${USERNAME}
