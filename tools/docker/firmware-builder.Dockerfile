FROM ubuntu:24.04@sha256:561618e2c15bf2397621dd04f96926663a3b5616c189cf7e38db7e82f5c538ea

ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       ca-certificates \
       clang-18 \
       cmake \
       curl \
       libclang-rt-18-dev \
       ninja-build \
       llvm-18 \
       xz-utils \
    && clang++-18 --version | grep -F "version 18." \
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

# Keep this gate in a layer after the large cross-toolchain download. CMake
# uses Git at configure time to re-verify the two detached dependency commits.
RUN apt-get update \
    && apt-get install -y --no-install-recommends git \
    && rm -rf /var/lib/apt/lists/*

ENV PATH="/opt/arm-gnu-toolchain/bin:${PATH}"
ENV SOURCE_DATE_EPOCH=0
WORKDIR /workspace
ENTRYPOINT []
CMD ["/bin/bash"]
