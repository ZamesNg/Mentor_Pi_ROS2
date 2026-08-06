FROM microros/micro_ros_static_library_builder:jazzy@sha256:1482f3df56184ecc5d4a9d45ad9be0a17a84a91fca947d07f20d1678b23f6243

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       clang-tidy-18 \
       cmake \
       gcc-arm-none-eabi \
       jq \
       libnewlib-arm-none-eabi \
       libstdc++-arm-none-eabi-dev \
       ninja-build \
    && test "$(arm-none-eabi-gcc -dumpfullversion)" = "13.2.1" \
    && clang-tidy-18 --version | grep -F "LLVM version 18." \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
ENTRYPOINT []
CMD ["/bin/bash"]
